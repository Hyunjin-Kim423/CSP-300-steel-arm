using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using Unity.Collections;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Sensor;
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
    }

    const int k_MaxRememberedFrames = 30;
    static readonly DateTime k_UnixEpoch = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);

    [SerializeField] Meta.XR.PassthroughCameraAccess cameraAccess;
    ROSConnection ros;
    public string topicName = "camera/rgb/image_raw";
    [Tooltip("Publish camera frames continuously at publishRateHz. When off, a frame is only sent when RequestScan() is called.")]
    public bool continuousPublish = true;
    public float publishRateHz = 10f;
    public int downsampleFactor = 2;
    float timer;
    bool readbackPending;
    int pendingScanFrames;
    long lastScanTicks = -1;
    long lastSentTicks = -1;
    Quaternion lastSentRotation;

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
        timer += Time.deltaTime;
        if (readbackPending || !cameraAccess.IsPlaying) return;
        bool scan = pendingScanFrames > 0;
        if (!scan && (!continuousPublish || timer < 1f / publishRateHz)) return;
        timer = 0f;
        if (scan) pendingScanFrames--;

        Pose cameraPose = cameraAccess.GetCameraPose();
        long ticks = (cameraAccess.Timestamp - k_UnixEpoch).Ticks;
        readbackPending = true;
        AsyncGPUReadback.Request(cameraAccess.GetTexture(), 0, request =>
        {
            readbackPending = false;
            if (request.hasError)
            {
                if (scan) pendingScanFrames++; // retry the scan with the next frame
                return;
            }
            if (scan) lastScanTicks = ticks;
            Publish(request.GetData<Color32>(), request.width, request.height, ticks, cameraPose);
        });
    }

    void Publish(NativeArray<Color32> pixels, int srcWidth, int srcHeight, long ticks, Pose cameraPose)
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
        float angularSpeed = 0f;
        if (lastSentTicks > 0 && ticks > lastSentTicks)
        {
            float dt = (ticks - lastSentTicks) / (float)TimeSpan.TicksPerSecond;
            if (dt < 1f)
                angularSpeed = Quaternion.Angle(lastSentRotation, cameraPose.rotation) / dt;
        }
        lastSentTicks = ticks;
        lastSentRotation = cameraPose.rotation;

        RememberFrame(ticks, new FrameInfo { cameraPose = cameraPose, imageSize = new Vector2Int(dstWidth, dstHeight), angularSpeed = angularSpeed });

        var msg = new ImageMsg
        {
            header = new HeaderMsg
            {
                stamp = new TimeMsg((int)(ticks / TimeSpan.TicksPerSecond), (uint)(ticks % TimeSpan.TicksPerSecond * 100)),
                frame_id = "quest_camera"
            },
            height = (uint)dstHeight,
            width = (uint)dstWidth,
            encoding = "rgb8",
            step = (uint)(dstWidth * 3),
            data = data
        };
        ros.Publish(topicName, msg);
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
