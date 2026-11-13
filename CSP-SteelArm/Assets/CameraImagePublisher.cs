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
        public float angularSpeed;   // how fast the camera turned since the previous sent frame (deg/s; 0 = unknown)
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
    [Tooltip("Camera image topic. YOLO and the calibration nodes both read this one topic (the image is sent only once).")]
    public string topicName = "/quest/camera/image_raw";
    public string cameraInfoTopic = "/quest/camera/camera_info";
    public string cameraPoseTopic = "/quest/camera/pose";
    [Tooltip("Publish camera frames continuously at publishRateHz. When off, a frame is only sent when RequestScan() is called.")]
    public bool continuousPublish = true;
    public float publishRateHz = 10f;
    public int downsampleFactor = 2;
    [Header("Head pose (independent of the camera frames)")]
    public string headPoseTopic = "/quest/head/pose";
    public float headPoseRateHz = 30f;
    float headPoseTimer;
    bool headCameraWarned;
    float timer;
    bool readbackPending;
    int pendingScanFrames;
    long lastScanTicks = -1;
    long lastSentTicks = -1;
    Quaternion lastSentRotation;
    ulong sequence;
    ICameraFrameProvider frameProvider;
    string lastUnavailableStatus;   // so the 'camera unavailable' warning is logged once per reason, not every frame

    readonly Dictionary<long, FrameInfo> recentFrames = new Dictionary<long, FrameInfo>();
    readonly Queue<long> recentFrameOrder = new Queue<long>();

    public Meta.XR.PassthroughCameraAccess CameraAccess => cameraAccess;

    void Awake()
    {
        // PassthroughCameraAccess never requests the camera permission itself; it just waits for it.
        // Request it here (Awake runs before every Start) so a fresh install still gets the prompt.
        // Scene is requested in the same dialog because Android drops a second request made while one is pending.
        OVRPermissionsRequester.Request(new[]
        {
            OVRPermissionsRequester.Permission.PassthroughCameraAccess,
            OVRPermissionsRequester.Permission.Scene
        });
    }

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);
        ros.RegisterPublisher<CameraInfoMsg>(cameraInfoTopic);
        ros.RegisterPublisher<PoseStampedMsg>(cameraPoseTopic);
        ros.RegisterPublisher<PoseStampedMsg>(headPoseTopic);
#if UNITY_EDITOR
        frameProvider = useMockCameraInEditor ? new MockCameraFrameProvider(mockWidth, mockHeight) : new QuestCameraFrameProvider(cameraAccess);
#else
        if (useMockCameraInEditor) Debug.LogError("Mock camera was selected in a player build; refusing to publish calibration frames.");
        frameProvider = useMockCameraInEditor ? null : new QuestCameraFrameProvider(cameraAccess);
#endif
    }

    // Head position for pointing: sent every few frames from the headset camera transform, so it keeps
    // flowing when the (large) passthrough images are slow or unavailable.
    void PublishHeadPose()
    {
        headPoseTimer += Time.deltaTime;
        if (headPoseTimer < 1f / headPoseRateHz) return;
        var cam = Camera.main;
        if (cam == null)
        {
            foreach (var c in Camera.allCameras)
                if (c.enabled && c.stereoEnabled) { cam = c; break; }
        }
        if (cam == null)
        {
            if (!headCameraWarned) { Debug.LogWarning("Head pose not published: no main or stereo camera found."); headCameraWarned = true; }
            return;
        }
        headPoseTimer = 0f;
        var t = cam.transform;
        Pose rosPose = UnityWorldCameraToRos(new Pose(t.position, t.rotation));
        long ticks = (DateTime.UtcNow - k_UnixEpoch).Ticks;
        var stamp = new TimeMsg((int)(ticks / TimeSpan.TicksPerSecond), (uint)(ticks % TimeSpan.TicksPerSecond * 100));
        ros.Publish(headPoseTopic, new PoseStampedMsg
        {
            header = new HeaderMsg { stamp = stamp, frame_id = "quest_head" },
            pose = new PoseMsg
            {
                position = new PointMsg(rosPose.position.x, rosPose.position.y, rosPose.position.z),
                orientation = new QuaternionMsg(rosPose.rotation.x, rosPose.rotation.y, rosPose.rotation.z, rosPose.rotation.w)
            }
        });
    }

    // Sends the next camera frame as a scan frame. Returns false when the camera is not running yet.
    public bool RequestScan()
    {
        if (!cameraAccess.IsPlaying)
            return false;
        pendingScanFrames = 1;
        return true;
    }

    // True when the stamp belongs to the most recent scan frame.
    public bool IsScanFrame(TimeMsg stamp, out long ticks)
    {
        ticks = ToTicks(stamp);
        return ticks == lastScanTicks;
    }

    static long ToTicks(TimeMsg stamp) => stamp.sec * TimeSpan.TicksPerSecond + stamp.nanosec / 100;

    void Update()
    {
        PublishHeadPose();
        timer += Time.deltaTime;
        if (readbackPending) return;
        bool scan = pendingScanFrames > 0;
        if (!scan && (!continuousPublish || timer < 1f / publishRateHz)) return;

        // Texture, physical RGB-camera pose, sensor intrinsics and source timestamp are captured as one frame record.
        Texture texture = null;
        CameraFrameMetadata metadata = default;
        if (frameProvider == null || !frameProvider.TryGetFrame(out texture, out metadata))
        {
            string status = metadata.Status ?? "no frame provider";
            if (status != lastUnavailableStatus)
            {
                lastUnavailableStatus = status;
                Debug.LogWarning($"Quest camera unavailable: {status}");
            }
            return; // a requested scan stays pending and is sent with the first frame that is available
        }
        lastUnavailableStatus = null;
        timer = 0f;
        if (scan) pendingScanFrames--;
        long ticks = (metadata.TimestampUtc - k_UnixEpoch).Ticks;
        readbackPending = true;
        AsyncGPUReadback.Request(texture, 0, request =>
        {
            readbackPending = false;
            if (request.hasError)
            {
                Debug.LogWarning("Quest camera GPU readback failed; frame rejected.");
                if (scan) pendingScanFrames++; // retry the scan with the next frame
                return;
            }
            if (scan) lastScanTicks = ticks;
            Publish(request.GetData<Color32>(), request.width, request.height, ticks, metadata);
        });
    }

    void Publish(NativeArray<Color32> pixels, int srcWidth, int srcHeight, long ticks, CameraFrameMetadata metadata)
    {
        // Debug.Log("[steelarm] Cam Image publishing");
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

        // Head turn rate between this frame and the previous one sent: results of fast-turning frames are not used to measure.
        Pose cameraPose = metadata.WorldPose;
        float angularSpeed = 0f;
        if (lastSentTicks > 0 && ticks > lastSentTicks)
        {
            float dt = (ticks - lastSentTicks) / (float)TimeSpan.TicksPerSecond;
            if (dt < 1f)
                angularSpeed = Quaternion.Angle(lastSentRotation, cameraPose.rotation) / dt;
        }
        lastSentTicks = ticks;
        lastSentRotation = cameraPose.rotation;

        var stamp = new TimeMsg((int)(ticks / TimeSpan.TicksPerSecond), (uint)(ticks % TimeSpan.TicksPerSecond * 100));
        var header = new HeaderMsg { stamp = stamp, frame_id = metadata.IsMock ? "quest_camera_optical_mock" : "quest_camera_optical" };
        var id = ++sequence;
        RememberFrame(ticks, new FrameInfo
        {
            cameraPose = cameraPose,
            imageSize = new Vector2Int(dstWidth, dstHeight),
            angularSpeed = angularSpeed,
            sequence = id,
            isMock = metadata.IsMock
        });

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

    bool intrinsicsLogged;
    void PublishMetadata(HeaderMsg header, int srcWidth, int srcHeight, int width, int height, CameraFrameMetadata metadata)
    {
        // No crop or rectification is applied: rows are flipped and the image is uniformly decimated only.
        // Intrinsics are scaled from the exact PCA sensor resolution to the transmitted resolution (see crop handling below).
        if (metadata.SensorResolution.x <= 0 || metadata.SensorResolution.y <= 0) { Debug.LogWarning("CameraInfo rejected: invalid sensor resolution."); return; }
        // Intrinsics are defined for the full sensor. The Quest texture is not squeezed (pixels stay square, so fx == fy); when its aspect
        // differs from the sensor it is a centred crop: use ONE uniform scale and shift the principal point by the crop offset.
        float s = Mathf.Max((float)srcWidth / metadata.SensorResolution.x, (float)srcHeight / metadata.SensorResolution.y);
        float cropX = (metadata.SensorResolution.x * s - srcWidth) / 2f, cropY = (metadata.SensorResolution.y * s - srcHeight) / 2f;
        if (!intrinsicsLogged)
        {
            intrinsicsLogged = true;
            Debug.Log($"[CameraInfo] sensor={metadata.SensorResolution} texture={srcWidth}x{srcHeight} sent={width}x{height} rawF={metadata.FocalLength} rawC={metadata.PrincipalPoint} scale={s} crop=({cropX},{cropY})");
        }
        float dx = (float)width / srcWidth, dy = (float)height / srcHeight;
        double fx = metadata.FocalLength.x * s * dx, fy = metadata.FocalLength.y * s * dy;
        double cx = (metadata.PrincipalPoint.x * s - cropX) * dx, cy = (metadata.PrincipalPoint.y * s - cropY) * dy;
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
        return recentFrames.TryGetValue(ToTicks(stamp), out frame);
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
