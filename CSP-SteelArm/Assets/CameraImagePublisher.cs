using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using Unity.Collections;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

#if !ROS2
#error ROS-TCP-Connector is in ROS1 mode. Set Robotics > ROS Settings > Protocol to ROS2.
#endif

public class CameraImagePublisher : MonoBehaviour
{
    public struct FrameInfo
    {
        public Pose cameraPose;
        public Vector2Int imageSize;
        public ulong sequence;
        public bool isMock;
    }

    const int k_MaxRememberedFrames = 30;
    static readonly DateTime k_UnixEpoch = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);

    [SerializeField] Meta.XR.PassthroughCameraAccess cameraAccess;
    [Header("Calibration bridge")]
    [SerializeField] bool useMockCameraInEditor;
    [SerializeField] int mockWidth = 640;
    [SerializeField] int mockHeight = 480;
    ROSConnection ros;
    public string topicName = "/quest/camera/image_raw";
    public string cameraInfoTopic = "/quest/camera/camera_info";
    public string cameraPoseTopic = "/quest/camera/pose";
    public float publishRateHz = 10f;
    public int downsampleFactor = 2;
    float timer;
    bool readbackPending;
    ulong sequence;
    ICameraFrameProvider frameProvider;

    readonly Dictionary<long, FrameInfo> recentFrames = new Dictionary<long, FrameInfo>();
    readonly Queue<long> recentFrameOrder = new Queue<long>();

    public Meta.XR.PassthroughCameraAccess CameraAccess => cameraAccess;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);
        ros.RegisterPublisher<CameraInfoMsg>(cameraInfoTopic);
        ros.RegisterPublisher<PoseStampedMsg>(cameraPoseTopic);
#if UNITY_EDITOR
        frameProvider = useMockCameraInEditor ? new MockCameraFrameProvider(mockWidth, mockHeight) : new QuestCameraFrameProvider(cameraAccess);
#else
        if (useMockCameraInEditor) Debug.LogError("Mock camera was selected in a player build; refusing to publish calibration frames.");
        frameProvider = useMockCameraInEditor ? null : new QuestCameraFrameProvider(cameraAccess);
#endif
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer < 1f / publishRateHz || readbackPending) return;
        Texture texture = null;
        CameraFrameMetadata metadata = default;
        if (frameProvider == null || !frameProvider.TryGetFrame(out texture, out metadata))
        {
            Debug.LogWarning($"Quest calibration camera unavailable: {metadata.Status}");
            return;
        }
        timer = 0f;
        // Capture texture, physical RGB-camera pose, sensor intrinsics and source timestamp as one frame record.
        long ticks = (metadata.TimestampUtc - k_UnixEpoch).Ticks;
        readbackPending = true;
        AsyncGPUReadback.Request(texture, 0, request =>
        {
            readbackPending = false;
            if (request.hasError) { Debug.LogWarning("Quest calibration GPU readback failed; frame rejected."); return; }
            Publish(request.GetData<Color32>(), request.width, request.height, ticks, metadata);
        });
    }

    void Publish(NativeArray<Color32> pixels, int srcWidth, int srcHeight, long ticks, CameraFrameMetadata metadata)
    {
        int dstWidth = srcWidth / downsampleFactor;
        int dstHeight = srcHeight / downsampleFactor;
        byte[] data = new byte[dstWidth * dstHeight * 3];
        int idx = 0;
        for (int y = 0; y < dstHeight; y++)
        {
            // Texture rows are stored bottom-up, ROS images are top-down.
            int srcRow = ((dstHeight - 1 - y) * downsampleFactor) * srcWidth;
            for (int x = 0; x < dstWidth; x++)
            {
                Color32 c = pixels[srcRow + x * downsampleFactor];
                data[idx++] = c.r;
                data[idx++] = c.g;
                data[idx++] = c.b;
            }
        }

        var stamp = new TimeMsg((int)(ticks / TimeSpan.TicksPerSecond), (uint)(ticks % TimeSpan.TicksPerSecond * 100));
        var header = new HeaderMsg { stamp = stamp, frame_id = metadata.IsMock ? "quest_camera_optical_mock" : "quest_camera_optical" };
        var id = ++sequence;
        RememberFrame(ticks, new FrameInfo { cameraPose = metadata.WorldPose, imageSize = new Vector2Int(dstWidth, dstHeight), sequence = id, isMock = metadata.IsMock });

        var msg = new ImageMsg
        {
            header = header,
            height = (uint)dstHeight,
            width = (uint)dstWidth,
            encoding = "rgb8",
            step = (uint)(dstWidth * 3),
            data = data
        };
        ros.Publish(topicName, msg);
        PublishMetadata(header, srcWidth, srcHeight, dstWidth, dstHeight, metadata);
    }

    void PublishMetadata(HeaderMsg header, int srcWidth, int srcHeight, int width, int height, CameraFrameMetadata metadata)
    {
        // No crop or rectification is applied: rows are flipped and the image is uniformly decimated only.
        // Intrinsics are scaled from the exact PCA sensor resolution to the transmitted resolution.
        if (metadata.SensorResolution.x <= 0 || metadata.SensorResolution.y <= 0) { Debug.LogWarning("CameraInfo rejected: invalid sensor resolution."); return; }
        float sx = (float)srcWidth / metadata.SensorResolution.x, sy = (float)srcHeight / metadata.SensorResolution.y;
        float dx = (float)width / srcWidth, dy = (float)height / srcHeight;
        double fx = metadata.FocalLength.x * sx * dx, fy = metadata.FocalLength.y * sy * dy;
        double cx = metadata.PrincipalPoint.x * sx * dx, cy = metadata.PrincipalPoint.y * sy * dy;
        var info = new CameraInfoMsg { header = header, width = (uint)width, height = (uint)height, distortion_model = "unknown",
            d = Array.Empty<double>(), k = new[] { fx, 0d, cx, 0d, fy, cy, 0d, 0d, 1d }, r = new[] { 1d,0d,0d,0d,1d,0d,0d,0d,1d },
            p = new[] { fx,0d,cx,0d,0d,fy,cy,0d,0d,0d,1d,0d } };
        ros.Publish(cameraInfoTopic, info);
        Pose rosPose = UnityWorldCameraToRos(metadata.WorldPose);
        ros.Publish(cameraPoseTopic, new PoseStampedMsg { header = header, pose = new PoseMsg { position = new PointMsg(rosPose.position.x, rosPose.position.y, rosPose.position.z), orientation = new QuaternionMsg(rosPose.rotation.x, rosPose.rotation.y, rosPose.rotation.z, rosPose.rotation.w) } });
    }

    static Pose UnityWorldCameraToRos(Pose unity)
    {
        // Unity world (right, up, forward; LH) -> ROS world (forward, left, up; RH).
        // Camera local Unity -> ROS optical is (right, down, forward). Full basis conversion is validated in ROS tests.
        var p = unity.position; return new Pose(new Vector3(p.z, -p.x, p.y), new Quaternion(unity.rotation.z, -unity.rotation.x, unity.rotation.y, unity.rotation.w).normalized);
    }

    public bool TryGetFrame(TimeMsg stamp, out FrameInfo frame)
    {
        long ticks = stamp.sec * TimeSpan.TicksPerSecond + stamp.nanosec / 100;
        return recentFrames.TryGetValue(ticks, out frame);
    }

    void RememberFrame(long ticks, FrameInfo frame)
    {
        if (recentFrames.ContainsKey(ticks)) return;
        recentFrames.Add(ticks, frame);
        recentFrameOrder.Enqueue(ticks);
        if (recentFrameOrder.Count > k_MaxRememberedFrames)
            recentFrames.Remove(recentFrameOrder.Dequeue());
    }
}
