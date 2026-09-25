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
    }

    const int k_MaxRememberedFrames = 30;
    static readonly DateTime k_UnixEpoch = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);

    [SerializeField] Meta.XR.PassthroughCameraAccess cameraAccess;
    ROSConnection ros;
    public string topicName = "camera/rgb/image_raw";
    public float publishRateHz = 10f;
    public int downsampleFactor = 2;
    float timer;
    bool readbackPending;

    readonly Dictionary<long, FrameInfo> recentFrames = new Dictionary<long, FrameInfo>();
    readonly Queue<long> recentFrameOrder = new Queue<long>();

    public Meta.XR.PassthroughCameraAccess CameraAccess => cameraAccess;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer < 1f / publishRateHz || readbackPending) return;
        if (!cameraAccess.IsPlaying) return;
        timer = 0f;

        Pose cameraPose = cameraAccess.GetCameraPose();
        long ticks = (cameraAccess.Timestamp - k_UnixEpoch).Ticks;
        readbackPending = true;
        AsyncGPUReadback.Request(cameraAccess.GetTexture(), 0, request =>
        {
            readbackPending = false;
            if (request.hasError) return;
            Publish(request.GetData<Color32>(), request.width, request.height, ticks, cameraPose);
        });
    }

    void Publish(NativeArray<Color32> pixels, int srcWidth, int srcHeight, long ticks, Pose cameraPose)
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

        RememberFrame(ticks, new FrameInfo { cameraPose = cameraPose, imageSize = new Vector2Int(dstWidth, dstHeight) });

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
