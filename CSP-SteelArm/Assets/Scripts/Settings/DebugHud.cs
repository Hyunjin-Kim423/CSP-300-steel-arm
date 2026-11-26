using System;
using System.Text;
using RosMessageTypes.Std;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.UI;

// Debug overlay and gizmos (Settings > Debug). Creates itself; nothing needs to be added to the scene.
//
// HUD: a plain text panel. The upper part is the PC dashboard's status table, received from /steelarm/status
//      (std_msgs/String, JSON, 2 Hz). The app does NOT judge anything there: it shows what the dashboard says.
//      If no status came for StaleSeconds, every row turns gray and the header says so. So "FAIL" always means
//      the PC side found a problem, and "no status" means the link to the dashboard is the problem.
//      The lower part shows what only the app knows (ROS connection, YOLO results, selected object).
// Gizmos: robot base after LOCK (from the same status message) and the target point of the selected object.
// It also applies the Debug log switches to the components that log.
public class DebugHud : MonoBehaviour
{
    const string StatusTopic = "/steelarm/status";
    const float StaleSeconds = 3f;
    const float Scale = 0.0005f;
    static readonly Vector3 ViewOffset = new Vector3(-0.2f, 0.1f, 0.8f); // head space (meters): upper left
    const float AxisLength = 0.12f;

    [Serializable] class Row { public string label; public string state; public string note; }
    [Serializable]
    class Status
    {
        public string mode;
        public Row[] rows;
        public string[] errors;
        public bool has_robot;
        public float[] robot_p;   // ROS quest_world: position of base_link
        public float[] robot_ax;  // robot +x axis (toward the user side)
        public float[] robot_az;  // robot +z axis (up)
    }

    GameObject panel;
    TMP_Text text;
    Transform head;
    Status status;
    float statusTime = -1f;      // Time.unscaledTime of the last status
    int statusCount;
    ROSConnection ros;
    readonly StringBuilder sb = new StringBuilder();
    float nextText;

    GameObject robotGizmo;
    GameObject targetGizmo;
    Material gizmoMaterial;

    static AppSettings S => AppSettings.Current;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
    static void CreateIfMissing()
    {
        if (FindAnyObjectByType<DebugHud>() == null)
            new GameObject(nameof(DebugHud)).AddComponent<DebugHud>();
    }

    void Awake()
    {
        BuildPanel();
        gizmoMaterial = new Material(Shader.Find("Sprites/Default")); // same shader as the silhouettes (kept in builds)
        robotGizmo = BuildRobotGizmo();
        targetGizmo = BuildTargetGizmo();
        AppSettings.Changed += Apply;
        Apply();
    }

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<StringMsg>(StatusTopic, OnStatus);
    }

    void OnDestroy() => AppSettings.Changed -= Apply;

    void Apply()
    {
        panel.SetActive(S.debugHud);
        if (YoloBoxVisualizer.Instance != null)
            YoloBoxVisualizer.Instance.debugLog = S.logScanStats;
    }

    void OnStatus(StringMsg msg)
    {
        try
        {
            status = JsonUtility.FromJson<Status>(msg.data);
            statusTime = Time.unscaledTime;
            statusCount++;
        }
        catch (Exception e)
        {
            Debug.LogWarning("[steelarm] Dashboard status could not be read: " + e.Message);
        }
    }

    bool Fresh => statusTime >= 0f && Time.unscaledTime - statusTime < StaleSeconds;

    void Update()
    {
        // The visualizer may appear after us: keep its log switch in sync.
        if (YoloBoxVisualizer.Instance != null && YoloBoxVisualizer.Instance.debugLog != S.logScanStats)
            YoloBoxVisualizer.Instance.debugLog = S.logScanStats;
        UpdateGizmos();
        if (panel.activeSelf && Time.unscaledTime >= nextText)
        {
            nextText = Time.unscaledTime + 0.25f;
            text.text = BuildText();
        }
    }

    // ---------------------------------------------------------------- text

    string BuildText()
    {
        sb.Clear();
        if (statusTime < 0f)
            sb.Append("<b>SteelArm</b>  <color=#9CA3AF>no status from the PC dashboard yet (is start_steelarm.bat running?)</color>\n");
        else if (!Fresh)
            sb.Append($"<b>SteelArm</b>  <color=#FCA5A5>NO STATUS for {Time.unscaledTime - statusTime:0} s: link to the PC dashboard lost. Rows below are OLD.</color>\n");
        else
            sb.Append($"<b>SteelArm</b>  {status.mode}  <color=#9CA3AF>status {Time.unscaledTime - statusTime:0.0} s ago</color>\n");

        if (status != null && status.rows != null)
        {
            foreach (Row r in status.rows)
            {
                string tag = Fresh ? Tag(r.state) : "<color=#6B7280> ?? </color>";
                string note = Fresh ? r.note : $"<color=#6B7280>{r.note}</color>";
                sb.Append($"{tag}<pos=9%>{r.label}<pos=30%>{note}\n");
            }
            if (Fresh && status.errors != null)
                foreach (string e in status.errors)
                    sb.Append($"<color=#FCA5A5>  ! {e}</color>\n");
        }

        // What only the app knows.
        sb.Append("<color=#9CA3AF>─── app ───</color>\n");
        string rosState = ros == null || !ros.HasConnectionThread ? "<color=#FCD34D>not started</color>"
            : ros.HasConnectionError ? "<color=#FCA5A5>connection error</color>" : "<color=#4ADE80>connected</color>";
        sb.Append($"ROS<pos=30%>{rosState}  {(ros != null ? $"{ros.RosIPAddress}:{ros.RosPort}" : "")}\n");
        YoloBoxVisualizer v = YoloBoxVisualizer.Instance;
        sb.Append("YOLO results<pos=30%>" + (v == null ? "no visualizer"
            : v.LastResultTime < 0f ? "none yet"
            : $"{Time.time - v.LastResultTime:0.0} s ago, {v.LastResultCount} object(s)") + "\n");
        ObjectCube sel = SelectedObjectPublisher.Selected;
        sb.Append("Selected<pos=30%>" + (sel == null ? "none" : $"{sel.className} #{sel.serial}  (sent to the robot)") + "\n");
        return sb.ToString();
    }

    static string Tag(string state)
    {
        switch (state)
        {
            case "OK": return "<color=#4ADE80> OK </color>";
            case "FAIL": return "<color=#F87171>FAIL</color>";
            case "TODO": return "<color=#FCD34D> >> </color>";
            case "WAIT": return "<color=#FCD34D>WAIT</color>";
            default: return "<color=#6B7280> -- </color>";
        }
    }

    void BuildPanel()
    {
        panel = new GameObject("DebugHudPanel", typeof(RectTransform));
        panel.transform.SetParent(transform, false);
        var rect = (RectTransform)panel.transform;
        rect.sizeDelta = new Vector2(900f, 420f);
        rect.localScale = Vector3.one * Scale;
        var canvas = panel.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.WorldSpace;
        panel.AddComponent<Image>().color = new Color(0.07f, 0.078f, 0.094f, 0.8f);

        var textObject = new GameObject("Text", typeof(RectTransform));
        textObject.transform.SetParent(panel.transform, false);
        var textRect = (RectTransform)textObject.transform;
        textRect.anchorMin = Vector2.zero;
        textRect.anchorMax = Vector2.one;
        textRect.offsetMin = new Vector2(16f, 12f);
        textRect.offsetMax = new Vector2(-16f, -12f);
        text = textObject.AddComponent<TextMeshProUGUI>();
        if (TMP_Settings.defaultFontAsset != null)
            text.font = TMP_Settings.defaultFontAsset;
        text.fontSize = 17f;
        text.alignment = TextAlignmentOptions.TopLeft;
        text.textWrappingMode = TextWrappingModes.NoWrap;
        text.overflowMode = TextOverflowModes.Ellipsis;
        text.raycastTarget = false;
    }

    void LateUpdate()
    {
        if (!panel.activeSelf)
            return;
        if (head == null)
        {
            if (Camera.main == null)
                return;
            head = Camera.main.transform;
        }
        // Same feel as the log panel: follows the head with a small lag.
        Vector3 target = head.position + head.rotation * ViewOffset;
        panel.transform.position = Vector3.Lerp(panel.transform.position, target, 1f - Mathf.Exp(-8f * Time.deltaTime));
        panel.transform.rotation = Quaternion.LookRotation(panel.transform.position - head.position, Vector3.up);
    }

    // ---------------------------------------------------------------- gizmos

    // ROS quest_world (forward, left, up) -> Unity world (right, up, forward). Inverse of the head pose conversion.
    static Vector3 RosToUnity(float[] v) => new Vector3(-v[1], v[2], v[0]);

    void UpdateGizmos()
    {
        bool showRobot = S.gizmoRobotBase && status != null && status.has_robot
                         && status.robot_p != null && status.robot_p.Length == 3
                         && status.robot_ax != null && status.robot_ax.Length == 3
                         && status.robot_az != null && status.robot_az.Length == 3;
        robotGizmo.SetActive(showRobot);
        if (showRobot)
        {
            Vector3 up = RosToUnity(status.robot_az), forward = RosToUnity(status.robot_ax);
            robotGizmo.transform.SetPositionAndRotation(RosToUnity(status.robot_p), Quaternion.LookRotation(forward, up));
        }

        ObjectCube sel = SelectedObjectPublisher.Selected;
        bool showTarget = S.gizmoTarget && sel != null && !sel.Dying;
        targetGizmo.SetActive(showTarget);
        if (showTarget)
            targetGizmo.transform.position = sel.center;
    }

    // Three axes in the robot frame: red = +x (toward the user side), green = +y (robot's left), blue = +z (up).
    // Built on the Unity axes: local +z = robot +x, local +y = robot +z, so robot +y = local -x.
    GameObject BuildRobotGizmo()
    {
        var root = new GameObject("RobotBaseGizmo");
        root.transform.SetParent(transform, false);
        Line(root.transform, Vector3.forward * AxisLength, new Color(0.97f, 0.33f, 0.33f));
        Line(root.transform, Vector3.left * AxisLength, new Color(0.29f, 0.87f, 0.5f));
        Line(root.transform, Vector3.up * AxisLength, new Color(0.38f, 0.65f, 0.98f));
        root.SetActive(false);
        return root;
    }

    GameObject BuildTargetGizmo()
    {
        var root = new GameObject("TargetPointGizmo");
        root.transform.SetParent(transform, false);
        var color = new Color(0.99f, 0.83f, 0.3f);
        const float r = 0.03f;
        Line(root.transform, Vector3.right * r, color, -Vector3.right * r);
        Line(root.transform, Vector3.up * r, color, -Vector3.up * r);
        Line(root.transform, Vector3.forward * r, color, -Vector3.forward * r);
        root.SetActive(false);
        return root;
    }

    void Line(Transform parent, Vector3 end, Color color, Vector3 start = default)
    {
        var go = new GameObject("Line");
        go.transform.SetParent(parent, false);
        var lr = go.AddComponent<LineRenderer>();
        lr.useWorldSpace = false;
        lr.positionCount = 2;
        lr.SetPosition(0, start);
        lr.SetPosition(1, end);
        lr.widthMultiplier = 0.006f;
        lr.material = gizmoMaterial;
        lr.startColor = lr.endColor = color;
        lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        lr.receiveShadows = false;
    }
}
