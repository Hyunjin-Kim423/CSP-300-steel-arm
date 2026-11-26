using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

// TEST: sends the position of the selected object to ROS, so the robot arm can point at it.
//
// - Select an object with the trigger (the menu still opens as before). Its center is sent to
//   /quest/selected_object (PointStamped, frame quest_world) 10 times a second while it stays selected.
// - Selecting empty space, or the object disappearing, stops sending.
// - The object keeps its last world position between YOLO results (the tracker anchors it in the world),
//   so slow scans do not stop the stream; only losing the object does.
//
// Not placed in any scene: it creates itself at start (see Create). Set Enabled = false to turn it off.
public class SelectedObjectPublisher : MonoBehaviour
{
    public static bool Enabled = true;
    const string Topic = "/quest/selected_object";
    const float RateHz = 10f;
    static readonly DateTime k_UnixEpoch = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);

    ROSConnection ros;
    ObjectCube selected;
    float timer;

    // The object being sent right now (null = none). Used by DebugHud.
    public static ObjectCube Selected { get; private set; }

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void Create()
    {
        if (!Enabled || FindAnyObjectByType<SelectedObjectPublisher>() != null) return;
        var go = new GameObject(nameof(SelectedObjectPublisher));
        DontDestroyOnLoad(go);
        go.AddComponent<SelectedObjectPublisher>();
    }

    void OnEnable() => YoloBoxVisualizer.ObjectSelected += OnSelected;
    void OnDisable() => YoloBoxVisualizer.ObjectSelected -= OnSelected;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PointStampedMsg>(Topic);
    }

    void OnSelected(ObjectCube cube)
    {
        selected = Selected = cube;
        timer = 1f; // send at once
        if (AppSettings.Current.logSelection)
            Debug.Log(cube != null
                ? $"[steelarm] Selected {cube.className} #{cube.serial} at {cube.center} (Unity world): sent to the robot"
                : "[steelarm] Selection cleared");
    }

    void Update()
    {
        if (selected == null || ros == null) return;
        if (selected.Dying)
        {
            if (AppSettings.Current.logSelection)
                Debug.Log($"[steelarm] Selected {selected.className} #{selected.serial} lost: stopped sending");
            selected = Selected = null;
            return;
        }
        timer += Time.deltaTime;
        if (timer < 1f / RateHz) return;
        timer = 0f;

        // Same conversion as the head pose (CameraImagePublisher.UnityWorldCameraToRos):
        // Unity world (right, up, forward) -> ROS quest_world (forward, left, up).
        Vector3 p = selected.center;
        long ticks = (DateTime.UtcNow - k_UnixEpoch).Ticks;
        ros.Publish(Topic, new PointStampedMsg
        {
            header = new HeaderMsg
            {
                stamp = new TimeMsg((int)(ticks / TimeSpan.TicksPerSecond), (uint)(ticks % TimeSpan.TicksPerSecond * 100)),
                frame_id = "quest_world"
            },
            point = new PointMsg(p.z, -p.x, p.y)
        });
    }
}
