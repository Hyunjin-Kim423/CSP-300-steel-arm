using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.XR.CoreUtils;
using RosMessageTypes.Yolo;
using UnityEngine.XR.Interaction.Toolkit.Interactors;

// Shows the objects YOLO finds in the passthrough camera image as silhouettes fixed in the world, and lets the user
// pick one with a controller (the trigger opens its menu).
//
//   Live mode   camera frames are sent continuously; the objects follow the real ones.
//   Scan mode   A / X sends one camera frame; its objects stay where they were found until the next scan.
//
// This component only connects the parts and holds the settings. The work is done in (Assets/Scripts/YoloVisualizer):
//   CameraImagePublisher   sends camera frames to YOLO and remembers the camera pose of each
//   ObjectTracker          YOLO results -> list of objects (matching, creating, removing)
//   DepthMeasurer          where a result is in the world (depth rays)
//   ObjectCube             one object: its box in the world, and how its silhouette is animated
//   ObjectSilhouette       draws one object's outline and fill
//   MaskGrid               averages an object's masks and traces the outline
//   MaskOutline            YOLO mask point list -> separate regions
//   Polygon2D              point-in-polygon, fill triangulation
//   ScanFrame              image position <-> world position for one camera frame
//   OneEuroFilter3         position smoothing
//   ControllerInput        buttons, aim ray, "is this hand on the UI?"
//   ScanWaveEffect         the wave played while scanning
//
// Values that rarely need changing are constants at the top of the class that uses them.
[RequireComponent(typeof(CameraImagePublisher))]
public class YoloBoxVisualizer : MonoBehaviour
{
    // ===================== Inspector =====================

    [Tooltip("On: camera frames are sent continuously and the objects follow the real ones. Off: A / X scans once.")]
    public bool liveMode = true;
    [Tooltip("YOLO results topic.")]
    public string topicName = "/yolo/detections";

    [Header("YOLO confidence")]
    [Tooltip("A NEW object needs a YOLO score of at least this. The YOLO node's own threshold must be at or below Keep Score.")]
    [Range(0f, 1f)] public float createScore = 0.5f;
    [Tooltip("An object already shown is kept by results down to this score, so it does not flicker when YOLO is unsure.")]
    [Range(0f, 1f)] public float keepScore = 0.3f;

    [Header("Tracking")]
    [Tooltip("Live: an object that is in view but not found in this many results in a row is removed.")]
    public int lostMisses = 4;
    [Tooltip("Measured moves smaller than this (meters) are ignored, so a resting object stays perfectly still.")]
    public float deadband = 0.02f;
    [Tooltip("Position smoothing while an object is nearly still (Hz). Lower = steadier, more lag.")]
    public float minCutoff = 0.5f;

    [Header("Silhouette")]
    [Tooltip("Seconds the outline takes to follow a change while the object rests. Higher = calmer, slower to adapt when you walk around it.")]
    public float shapeTimeConstant = 0.8f;
    [Tooltip("Rounds off the outline (0 = off). Higher values can erase thin parts such as chair legs.")]
    [Range(0, 4)] public int outlineBlur = 1;

    [Header("Look")]
    public Color outlineColor = Color.white;
    [Tooltip("Pointed object that is listed in yolo_dectection_list.json (it has robot actions).")]
    public Color boxColor = Color.green;
    [Tooltip("Pointed object that is NOT in yolo_dectection_list.json.")]
    public Color unregisteredColor = Color.red;
    [Tooltip("Outline width in meters.")]
    public float lineWidth = 0.004f;
    [Tooltip("Objects fade in when they appear and out when they are removed (seconds; 0 = off).")]
    public float fadeDuration = 0.2f;

    [Header("Debug")]
    [Tooltip("Log objects appearing / jumping / disappearing, and a summary every 5 s (results, depth rays).")]
    public bool debugLog = true;

    // ===================== Constants =====================

    const float FillAlpha = 0.2f;              // opacity of the pointed object's fill
    const float SoftLineWidthScale = 1.8f;     // soft-edged lines look thinner, so they are drawn this much wider
    const float ClearWhenSilent = 3f;          // live: all objects are removed after this long without YOLO results
    const float ScanCooldown = 0.4f;           // scan presses closer together than this are ignored
    const float WaveInterval = 1.2f;           // while a scan result is awaited, another wave every this long
    const float ScanResultTimeout = 5f;        // stop waiting for a scan result after this long
    const float StatsInterval = 5f;

    // ===================== State =====================

    public static YoloBoxVisualizer Instance { get; private set; }

    // Shown in the settings menu (Advanced): where the results come from, and the latest one.
    public string ResultSourceDescription
        => !subscribed ? Loc.T("status.notStarted") : Loc.F(liveMode ? "status.detectionTopic" : "status.scanTopic", topicName);
    public float LastResultTime => lastResultTime;   // Time.time of the latest YOLO result (< 0: none yet)
    public int LastResultCount { get; private set; }

    CameraImagePublisher publisher;
    DepthMeasurer depth;
    ObjectTracker tracker;
    XROrigin xrOrigin;
    ControllerInput[] controllers;
    Material registeredFill;
    Material unregisteredFill;
    ObjectCube hovered;
    bool subscribed;
    float lastResultTime = -1f;

    // Scan mode
    ScanWaveEffect wave;
    long handledScanTicks = -1;
    float lastScanPress = float.NegativeInfinity;
    float scanPendingSince = -1f;   // < 0: no scan result expected
    float lastWaveTime;

    // Stats log
    int resultCount;
    float statsSince;

    // ===================== Unity lifecycle =====================

    void Awake()
    {
        if (Instance == null)
            Instance = this;
        else
            Destroy(this);
    }

    void Start()
    {
        publisher = GetComponent<CameraImagePublisher>();
        publisher.continuousPublish = liveMode; // scan mode: frames are only sent on request
        xrOrigin = FindAnyObjectByType<XROrigin>();
        controllers = new[]
        {
            new ControllerInput("RightHand", InteractorHandedness.Right),
            new ControllerInput("LeftHand", InteractorHandedness.Left)
        };

        Shader shader = Shader.Find("Sprites/Default");
        var lineMaterial = new Material(shader) { mainTexture = CreateSoftLineTexture() };
        registeredFill = new Material(shader) { color = WithAlpha(boxColor, FillAlpha) };
        unregisteredFill = new Material(shader) { color = WithAlpha(unregisteredColor, FillAlpha) };

        depth = new DepthMeasurer(gameObject);
        tracker = new ObjectTracker(this, depth, lineMaterial);
        if (!liveMode && !TryGetComponent(out wave))
            wave = gameObject.AddComponent<ScanWaveEffect>();

        statsSince = Time.time;
        Debug.Log("[steelarm] YOLO objects: " + (liveMode ? "live mode" : "scan mode (A / X)"));
    }

    void OnDestroy()
    {
        if (controllers == null)
            return;
        foreach (var controller in controllers)
            controller.Dispose();
    }

    void Update()
    {
        if (liveMode)
            ClearIfSilent();
        else
            UpdateScanInput();

        Camera cam = Camera.main;
        Vector3 head = cam != null ? cam.transform.position : Vector3.zero;
        tracker.Animate(Time.time, Time.deltaTime, head, outlineColor, lineWidth * SoftLineWidthScale, fadeDuration);

        UpdateHover();
        foreach (var controller in controllers)
        {
            if (controller.SelectPressed && !controller.IsPointingAtUI())
                Select(controller.AimRay(xrOrigin));
        }

        LogStats();
    }

    // ===================== YOLO results =====================

    // Called once at start (StartWithControllerButton, or the CoachingUI Start button). Extra calls are ignored.
    public void SubscribeROS()
    {
        if (subscribed)
            return;
        subscribed = true;
        ROSConnection.GetOrCreateInstance().Subscribe<DetectionArrayMsg>(topicName, OnDetections);
    }

    void OnDetections(DetectionArrayMsg msg)
    {
        if (tracker == null)
            return; // not started yet
        bool scan = !liveMode;
        if (scan)
        {
            // Only the result of the latest scan frame is used (other frames and duplicates are ignored).
            if (!publisher.IsScanFrame(msg.header.stamp, out long ticks) || ticks == handledScanTicks)
                return;
            handledScanTicks = ticks;
            scanPendingSince = -1f;
        }
        lastResultTime = Time.time;
        LastResultCount = msg.detections.Length;
        resultCount++;

        // The camera pose of the image this result belongs to (not remembered any more if the result is very late).
        if (!publisher.CameraAccess.IsPlaying || !publisher.TryGetFrame(msg.header.stamp, out var info))
            return;
        var frame = new ScanFrame(publisher.CameraAccess, info.cameraPose, info.imageSize);
        tracker.Process(msg.detections, frame, scan, info.angularSpeed, Time.time);

        if (scan)
            Debug.Log($"[steelarm] Scan: {msg.detections.Length} result(s), {tracker.VisibleCount} object(s) shown");
    }

    // Live: results stopped coming (YOLO node down?), so the objects on screen are stale.
    void ClearIfSilent()
    {
        if (tracker.Count == 0 || lastResultTime < 0f || Time.time - lastResultTime < ClearWhenSilent)
            return;
        Debug.LogWarning($"[steelarm] No YOLO results for {ClearWhenSilent:0.#} s; removing all objects. Is the YOLO node running?");
        tracker.RemoveAll();
    }

    // ===================== Scan mode =====================

    void UpdateScanInput()
    {
        foreach (var controller in controllers)
        {
            if (controller.ScanPressed)
            {
                RequestScan();
                break;
            }
        }

        // The result is still on its way: another wave now and then shows that the scan is running.
        if (scanPendingSince < 0f)
            return;
        if (Time.time - scanPendingSince > ScanResultTimeout)
        {
            scanPendingSince = -1f;
            Debug.LogWarning("[steelarm] No scan result arrived. Is the YOLO node running?");
        }
        else if (Time.time - lastWaveTime > WaveInterval)
        {
            PlayWave();
        }
    }

    void RequestScan()
    {
        if (Time.time - lastScanPress < ScanCooldown)
            return;
        lastScanPress = Time.time;

        // The open menu belongs to the old scan; its object may be replaced.
        if (ObjectMenuGenerator._instance != null)
            ObjectMenuGenerator._instance.CloseCurrent();

        if (!publisher.RequestScan())
        {
            Debug.LogWarning("[steelarm] Scan ignored: the passthrough camera is not running yet.");
            return;
        }
        Debug.Log("[steelarm] Scan requested");
        scanPendingSince = Time.time;
        PlayWave();
    }

    void PlayWave()
    {
        lastWaveTime = Time.time;
        Camera cam = Camera.main;
        if (wave != null && cam != null)
            wave.Play(cam.transform.position);
    }

    // ===================== Pointing and selecting =====================

    // Highlights the object either controller points at (nearest hit wins). A hand whose XRI ray is on the UI
    // (e.g. the object menu) is ignored, so the object behind the menu is not highlighted.
    void UpdateHover()
    {
        ObjectCube pointed = null;
        float nearest = float.MaxValue;
        foreach (var controller in controllers)
        {
            if (controller.IsPointingAtUI())
                continue;
            ObjectCube hit = tracker.FindPointed(controller.AimRay(xrOrigin), out float distance);
            if (hit != null && distance < nearest)
            {
                pointed = hit;
                nearest = distance;
            }
        }
        if (pointed == hovered)
            return;
        hovered?.SetHighlight(false, outlineColor);
        hovered = pointed;
        hovered?.SetHighlight(true, ColorFor(hovered), hovered.registered ? registeredFill : unregisteredFill);
    }

    void Select(Ray ray)
    {
        ObjectCube pointed = tracker.FindPointed(ray, out _);
        if (pointed != null && ObjectMenuGenerator._instance != null)
            ObjectMenuGenerator._instance.Open(MenuTarget.From(pointed, ColorFor(pointed)));
    }

    Color ColorFor(ObjectCube cube) => cube.registered ? boxColor : unregisteredColor;

    // ===================== Helpers =====================

    static Color WithAlpha(Color c, float a) => new Color(c.r, c.g, c.b, a);

    // White line texture whose edges (across the line) fade out: a solid core, then a smooth falloff.
    static Texture2D CreateSoftLineTexture()
    {
        const int height = 32;
        var texture = new Texture2D(2, height, TextureFormat.RGBA32, false)
        {
            wrapMode = TextureWrapMode.Clamp,
            filterMode = FilterMode.Bilinear,
            name = "SoftLine"
        };
        for (int y = 0; y < height; y++)
        {
            float across = Mathf.Abs((y + 0.5f) / height * 2f - 1f); // 0 at the center of the line, 1 at its edge
            var color = new Color(1f, 1f, 1f, 1f - Mathf.SmoothStep(0f, 1f, Mathf.InverseLerp(0.35f, 1f, across)));
            texture.SetPixel(0, y, color);
            texture.SetPixel(1, y, color);
        }
        texture.Apply();
        return texture;
    }

    // Every StatsInterval seconds: how many objects, results and depth rays (to check the cost).
    void LogStats()
    {
        float seconds = Time.time - statsSince;
        if (seconds < StatsInterval)
            return;
        if (debugLog)
        {
            int shown = tracker.VisibleCount;
            Debug.Log($"[steelarm] objects: {shown} shown, {tracker.Count - shown} tentative | last {seconds:0} s: {resultCount} results, " +
                      $"{depth.measurements} measurements, {depth.rays} depth rays ({depth.rays / seconds:0} rays/s), " +
                      $"{tracker.cutOffResults} cut off by the image border, {tracker.duplicatesSkipped} duplicates skipped");
        }
        resultCount = depth.measurements = depth.rays = tracker.cutOffResults = tracker.duplicatesSkipped = 0;
        statsSince = Time.time;
    }
}
