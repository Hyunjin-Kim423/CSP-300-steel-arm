using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.XR.CoreUtils;
using RosMessageTypes.Yolo;
using UnityEngine.XR.Interaction.Toolkit.Interactors;
using EnvironmentRaycastManager = Meta.XR.EnvironmentRaycastManager;

// Shows the objects YOLO finds in the passthrough camera image, fixed in the world, and lets the user pick one.
//
// Every object is a gravity-aligned box (ObjectCube) placed in WORLD space. The box does not follow the head: turning
// the head or walking around only changes the view. It is moved only when YOLO shows that the real object moved.
// The box is used for matching; what is DRAWN is (drawStyle):
//   Silhouette  the object's outline, anchored at the box's front face. Its position never changes with the view;
//               only its shape is refined: every matched mask is moved into the anchor's frame (with that image's
//               camera pose) and averaged (ObjectSilhouette). Resting objects average strongly, moving ones follow fast.
//   Box         the wireframe box.
//
// For every YOLO result (live mode: every message, scan mode: the one scan frame):
//   1. Project   every box is drawn into that camera image, using the camera pose of the moment the image was taken
//   2. Match     a result belongs to a box of the same class when they overlap (IoU >= matchIoU).
//                A yolo_ros track id (/yolo/tracking, if used) is trusted first, but is not needed:
//                matching works with /yolo/detections alone, and head movement does not disturb it.
//   3. Update    matched + still overlapping (IoU >= stillIoU)  -> nothing to do, no depth rays (most results)
//                matched but off for moveConfirmCount results     -> measure again with depth rays, filter, move
//                not matched                                      -> new tentative object; shown once seen confirmHits times
//                box in view but not found for lostMisses results -> removed (boxes out of view are kept)
//                every matched result also adds its mask to the object's silhouette (no depth rays needed)
//   4. Draw      each frame the drawn object eases to its box; the pointed one is highlighted, the trigger opens its menu.
//
// Measuring an object: a few depth rays inside its YOLO box (inside the mask if a -seg model sends one), median depth
// = front face; width / height from the YOLO box at that depth; front-to-back depth guessed from the width.
// Depth is measured only when an object is new or has moved, never just to redraw it.
//
// The old per-message silhouette code (DetectedObject, DetectedObjectPool) is still in the project but unused.
[RequireComponent(typeof(CameraImagePublisher))]
public class YoloBoxVisualizer : MonoBehaviour
{
    // ===================== Inspector =====================

    [Header("Mode")]
    [Tooltip("Live: camera frames are sent continuously and the boxes are kept up to date. Off = scan mode: A / X sends one frame.")]
    public bool liveMode = true;

    [Header("Topics")]
    [Tooltip("Untracked detections. Used in scan mode, when 'Use Tracking' is off, and while the tracking topic is silent.")]
    public string topicName = "/yolo/detections";
    [Tooltip("Live mode: also listen to the yolo_ros tracking topic and use its track ids to help matching (not required).")]
    public bool useTracking = true;
    public string trackingTopicName = "/yolo/tracking";
    [Tooltip("If the tracking topic sends nothing for this many seconds, the detections topic is used instead.")]
    public float trackingFallbackTimeout = 1.5f;

    [Header("Matching")]
    [Tooltip("A result and a box are the same object when the box, drawn into that camera image, overlaps the result at least this much (IoU 0-1).")]
    [Range(0.01f, 1f)] public float matchIoU = 0.15f;
    [Tooltip("Overlap at or above this means the object has not moved: the box is left alone (no depth rays).")]
    [Range(0.05f, 1f)] public float stillIoU = 0.4f;
    [Tooltip("Results in a row that must disagree with a box before it is measured again (it moved).")]
    public int moveConfirmCount = 2;
    [Tooltip("Live: a new object must be seen in this many results before its box is shown.")]
    public int confirmHits = 3;
    [Tooltip("Live: a new object that is not confirmed within this many seconds is dropped.")]
    public float tentativeTimeout = 1.0f;
    [Tooltip("Live: a box that is in view but not found in this many results in a row is removed. Boxes out of view are kept.")]
    public int lostMisses = 10;
    [Tooltip("Before counting a box as 'not found', check with one depth ray whether something is in front of it (e.g. a hand).")]
    public bool occlusionCheck = true;
    [Tooltip("Trust yolo_ros track ids first when matching (only with Use Tracking).")]
    public bool useTrackIds = true;
    [Tooltip("How far ahead (seconds) a moving box may be predicted when matching.")]
    public float maxPrediction = 0.3f;

    [Header("Measuring (depth rays)")]
    [Tooltip("Depth rays per measurement in live mode (the median is used).")]
    public int liveRays = 9;
    [Tooltip("Depth rays per object in scan mode (measured once, so more are affordable).")]
    public int scanRays = 25;
    [Tooltip("Aim the rays inside the segmentation mask (if YOLO sends one) so they hit the object, not the background.")]
    public bool raysInsideMask = true;
    [Tooltip("Depth (meters) used when no ray hits.")]
    public float fallbackDistance = 1.0f;
    [Tooltip("Front-to-back depth of a new box as a fraction of its width (the camera only sees the front).")]
    public float depthPerWidth = 1.0f;
    public float minBoxDepth = 0.05f;
    public float maxBoxDepth = 1.0f;
    [Tooltip("Live: no measurements from frames taken while the head turned faster than this (deg/s).")]
    public float maxHeadSpeed = 90f;

    [Header("Smoothing")]
    [Tooltip("Measured moves smaller than this (meters) are ignored, so a resting box stays perfectly still.")]
    public float deadband = 0.02f;
    [Tooltip("One Euro filter, smoothing while nearly still (Hz). Lower = steadier, more lag.")]
    public float minCutoff = 1.0f;
    [Tooltip("One Euro filter, how much fast moves relax the smoothing. Higher = less lag.")]
    public float beta = 2.0f;
    [Tooltip("How fast the drawn box eases to the box. Higher = snappier.")]
    public float followSpeed = 10f;

    [Header("Scan mode")]
    [Tooltip("Play a wave over the real surfaces when scanning (ScanWaveEffect, uses the headset's depth).")]
    public bool scanWave = true;
    [Tooltip("Scan presses closer together than this (seconds) are ignored.")]
    public float scanCooldown = 0.4f;
    [Tooltip("While the result has not arrived, another wave is sent every this many seconds.")]
    public float waitingWaveInterval = 1.2f;
    [Tooltip("Stop waiting for a scan result after this many seconds.")]
    public float scanResultTimeout = 5f;

    public enum DrawStyle
    {
        [Tooltip("The object's outline from the segmentation masks (needs a -seg model; without masks the YOLO box is used as the shape).")]
        Silhouette,
        [Tooltip("The wireframe box used for matching.")]
        Box
    }

    [Header("Silhouette")]
    public DrawStyle drawStyle = DrawStyle.Silhouette;
    [Tooltip("Weight of a new mask while the object rests (0-1). Lower = steadier shape, slower to change.")]
    [Range(0.05f, 1f)] public float shapeSmoothing = 0.25f;
    [Tooltip("Weight of a new mask while the object seems to move (it disagrees with its box).")]
    [Range(0.05f, 1f)] public float movingShapeWeight = 0.7f;
    [Tooltip("At this speed (m/s) or faster the newest mask is used as it is (no averaging).")]
    public float fastShapeSpeed = 0.5f;
    [Tooltip("If the view direction to an object jumps by more than this (degrees) between two masks, its average starts over.")]
    public float viewResetAngle = 40f;

    [Header("Look")]
    public Color outlineColor = Color.white;
    [Tooltip("Color of the pointed box when it is listed in yolo_dectection_list.json (it has robot actions).")]
    public Color boxColor = Color.green;
    [Tooltip("Color of the pointed box when it is NOT in yolo_dectection_list.json.")]
    public Color unregisteredColor = Color.red;
    [Tooltip("Opacity of the pointed silhouette's fill.")]
    [Range(0f, 1f)] public float fillAlpha = 0.2f;
    public float lineWidth = 0.004f;
    [Tooltip("New boxes flash in this color when they appear.")]
    public Color flashColor = new Color(0.55f, 0.9f, 1f, 1f);
    public float flashDuration = 0.5f;
    [Tooltip("Live: all boxes are removed if no YOLO results arrive for this many seconds (0 = keep them).")]
    public float clearWhenSilent = 3f;

    [Header("Debug")]
    [Tooltip("Log boxes being added / moved / removed.")]
    public bool logCubes = true;
    [Tooltip("Log a summary every 5 s: results, measurements and depth rays (to check the cost).")]
    public bool logStats = true;

    // ===================== State =====================

    public static YoloBoxVisualizer Instance { get; private set; }

    CameraImagePublisher publisher;
    EnvironmentRaycastManager environmentRaycast;
    XROrigin xrOrigin;
    ControllerInput[] controllers;
    Material lineMaterial;
    Material registeredFill;
    Material unregisteredFill;
    readonly List<ObjectCube> cubes = new List<ObjectCube>();
    ObjectCube hovered;
    int nextSerial = 1;
    bool subscribed;
    bool warnedNoMask;
    float lastResultTime = -1f;
    float lastTrackingTime = float.NegativeInfinity;
    bool usingTracking;

    // Scan mode
    long handledScanTicks = -1;
    ScanWaveEffect wave;
    float lastScanPress = float.NegativeInfinity;
    float scanPendingSince = -1f;   // < 0: no scan result expected
    float lastWaveTime;

    // Stats (logStats)
    int statResults, statMeasures, statRays;
    float statSince;

    // Scratch (reused every result, no allocations)
    struct Projection { public bool valid; public bool inView; public Rect rect; }
    struct Pair { public int det; public int cube; public float iou; public float score; }
    struct Measurement { public Vector3 front; public Vector3 forward; public float width; public float height; public float boxDepth; public bool hit; }
    readonly List<Projection> projections = new List<Projection>();
    readonly List<Pair> pairs = new List<Pair>();
    readonly List<bool> detUsed = new List<bool>();
    readonly List<bool> cubeUsed = new List<bool>();
    readonly List<Vector2> candidates = new List<Vector2>();
    readonly List<Vector2> samples = new List<Vector2>();
    readonly List<float> depths = new List<float>();
    readonly Vector3[] corners = new Vector3[8];
    static readonly System.Comparison<Pair> s_ByScore = (a, b) => b.score.CompareTo(a.score);
    static readonly List<Vector2> s_BoxRegion = new List<Vector2>();
    static readonly List<List<Vector2>> s_BoxRegions = new List<List<Vector2>> { s_BoxRegion };

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
        lineMaterial = new Material(Shader.Find("Sprites/Default"));
        registeredFill = new Material(Shader.Find("Sprites/Default")) { color = WithAlpha(boxColor, fillAlpha) };
        unregisteredFill = new Material(Shader.Find("Sprites/Default")) { color = WithAlpha(unregisteredColor, fillAlpha) };
        if (!liveMode && scanWave && !TryGetComponent(out wave))
            wave = gameObject.AddComponent<ScanWaveEffect>();

        if (EnvironmentRaycastManager.IsSupported)
        {
            OVRPermissionsRequester.Request(new[] { OVRPermissionsRequester.Permission.Scene });
            environmentRaycast = FindAnyObjectByType<EnvironmentRaycastManager>();
            if (environmentRaycast == null)
                environmentRaycast = gameObject.AddComponent<EnvironmentRaycastManager>();
        }
        else
        {
            Debug.LogWarning($"[steelarm] Environment depth is not supported; boxes are placed at {fallbackDistance} m.");
        }
        statSince = Time.time;
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
        if (!liveMode)
        {
            if (AnyScanPressed())
                RequestScan();
            KeepWavingWhileWaiting();
        }
        else if (clearWhenSilent > 0f && cubes.Count > 0 && lastResultTime >= 0f && Time.time - lastResultTime > clearWhenSilent)
        {
            Debug.LogWarning($"[steelarm] No YOLO results for {clearWhenSilent:0.#} s; removing all boxes. Is the YOLO node running?");
            RemoveAll();
        }

        float t = 1f - Mathf.Exp(-followSpeed * Time.deltaTime);
        Camera cam = Camera.main;
        Vector3 head = cam != null ? cam.transform.position : Vector3.zero;
        foreach (var cube in cubes)
            cube.Animate(t, Time.time, head, outlineColor, lineWidth, flashColor, flashDuration);

        UpdateHover();
        foreach (var controller in controllers)
        {
            if (controller.SelectPressed && !controller.IsPointingAtUI())
                Select(controller.AimRay(xrOrigin));
        }

        LogStats();
    }

    // ===================== Getting results =====================

    // Called once at start (StartWithControllerButton, or the CoachingUI Start button). Extra calls are ignored.
    public void SubscribeROS()
    {
        if (subscribed)
            return;
        subscribed = true;
        var ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<DetectionArrayMsg>(topicName, OnUntrackedDetections);
        if (useTracking)
            ros.Subscribe<DetectionArrayMsg>(trackingTopicName, OnTrackedDetections);
    }

    void OnTrackedDetections(DetectionArrayMsg msg)
    {
        // A single scan frame gives the tracker nothing to track, so scan mode uses the plain detections.
        if (!liveMode)
            return;
        lastTrackingTime = Time.time;
        if (!usingTracking)
        {
            usingTracking = true;
            Debug.Log($"[steelarm] Using '{trackingTopicName}' (track ids help matching).");
        }
        ProcessResult(msg, false);
    }

    void OnUntrackedDetections(DetectionArrayMsg msg)
    {
        if (!liveMode)
        {
            OnScanResult(msg);
            return;
        }
        // While tracking is alive its results are used; the untracked copy of the same frame is ignored.
        if (useTracking && Time.time - lastTrackingTime < trackingFallbackTimeout)
            return;
        if (usingTracking)
        {
            usingTracking = false;
            Debug.LogWarning($"[steelarm] No results on '{trackingTopicName}' for {trackingFallbackTimeout:0.0} s; using '{topicName}' (boxes are matched by position).");
        }
        // No need to start over: boxes are matched by position, not by track id.
        ProcessResult(msg, false);
    }

    // ===================== Scan mode =====================

    bool AnyScanPressed()
    {
        foreach (var controller in controllers)
        {
            if (controller.ScanPressed)
                return true;
        }
        return false;
    }

    void RequestScan()
    {
        if (Time.time - lastScanPress < scanCooldown)
            return;
        lastScanPress = Time.time;

        if (ObjectMenuGenerator._instance != null)
            ObjectMenuGenerator._instance.CloseCurrent();

        if (publisher.RequestScan())
        {
            Debug.Log("[steelarm] Scan requested");
            scanPendingSince = Time.time;
            PlayWave();
        }
        else
        {
            Debug.LogWarning("[steelarm] Scan ignored: the passthrough camera is not running yet.");
        }
    }

    void PlayWave()
    {
        lastWaveTime = Time.time;
        Camera cam = Camera.main;
        if (wave != null && cam != null)
            wave.Play(cam.transform.position);
    }

    void KeepWavingWhileWaiting()
    {
        if (scanPendingSince < 0f)
            return;
        if (Time.time - scanPendingSince > scanResultTimeout)
        {
            scanPendingSince = -1f;
            Debug.LogWarning("[steelarm] No scan result arrived. Is the YOLO node running?");
            return;
        }
        if (Time.time - lastWaveTime > waitingWaveInterval)
            PlayWave();
    }

    // Only the result of the latest scan frame is used (other frames and duplicates are ignored).
    void OnScanResult(DetectionArrayMsg msg)
    {
        if (!publisher.IsScanFrame(msg.header.stamp, out long ticks) || ticks == handledScanTicks)
            return;
        handledScanTicks = ticks;
        scanPendingSince = -1f;
        ProcessResult(msg, true);
        Debug.Log($"[steelarm] Scan: {msg.detections.Length} object(s), {CountVisible()} box(es) shown");
    }

    // ===================== Matching results to boxes =====================

    void ProcessResult(DetectionArrayMsg msg, bool scan)
    {
        lastResultTime = Time.time;
        statResults++;
        if (!publisher.CameraAccess.IsPlaying)
            return;
        if (!publisher.TryGetFrame(msg.header.stamp, out var info))
            return; // frame too old (not remembered any more)

        var frame = new ScanFrame(publisher.CameraAccess, info.cameraPose, info.imageSize);
        float now = Time.time;
        bool canMeasure = scan || info.angularSpeed <= maxHeadSpeed;
        DetectionMsg[] detections = msg.detections;

        // 1. Where every box appears in this image.
        projections.Clear();
        cubeUsed.Clear();
        foreach (var cube in cubes)
        {
            projections.Add(Project(cube, frame, now));
            cubeUsed.Add(false);
        }

        // 2. Candidate pairs (same class; overlap or same track id), best first.
        pairs.Clear();
        detUsed.Clear();
        for (int i = 0; i < detections.Length; i++)
        {
            detUsed.Add(false);
            DetectionMsg det = detections[i];
            Rect detRect = DetectionRect(det, frame);
            for (int j = 0; j < cubes.Count; j++)
            {
                ObjectCube cube = cubes[j];
                if (cube.className != det.class_name)
                    continue;
                bool sameId = useTracking && useTrackIds && !string.IsNullOrEmpty(det.id) && det.id == cube.trackId;
                float iou = projections[j].valid ? IoU(projections[j].rect, detRect) : 0f;
                if (!sameId && iou < matchIoU)
                    continue;
                pairs.Add(new Pair { det = i, cube = j, iou = iou, score = (sameId ? 1f : 0f) + iou });
            }
        }
        pairs.Sort(s_ByScore);

        // 3. Greedy one-to-one assignment.
        foreach (var pair in pairs)
        {
            if (detUsed[pair.det] || cubeUsed[pair.cube])
                continue;
            detUsed[pair.det] = true;
            cubeUsed[pair.cube] = true;
            UpdateMatched(cubes[pair.cube], detections[pair.det], pair.iou, frame, scan, canMeasure, now);
        }

        // 4. Boxes that should have been seen in this image but were not.
        int oldCount = projections.Count;
        for (int j = 0; j < oldCount; j++)
        {
            if (!cubeUsed[j] && projections[j].inView && !IsOccluded(cubes[j], frame))
                cubes[j].misses++;
        }

        // 5. Results that match no box: new objects.
        for (int i = 0; i < detections.Length; i++)
        {
            if (!detUsed[i])
                CreateCube(detections[i], frame, scan, canMeasure, now);
        }

        // 6. Remove what is gone.
        for (int j = cubes.Count - 1; j >= 0; j--)
        {
            if (ShouldRemove(cubes[j], scan, now))
                RemoveAt(j, scan ? "not found in the new scan" : "not seen any more");
        }
    }

    void UpdateMatched(ObjectCube cube, DetectionMsg det, float iou, ScanFrame frame, bool scan, bool canMeasure, float now)
    {
        cube.detection = det;
        cube.trackId = det.id ?? "";
        cube.hits++;
        cube.misses = 0;
        cube.lastSeen = now;

        if (cube.state == ObjectCube.State.Tentative)
        {
            // Not shown yet: every sighting is measured and averaged for a good first placement.
            if (canMeasure && Measure(det, frame, liveRays, out var m) && UsableDepth(m, false))
                cube.AddSample(m.front + m.forward * (cube.size.z * 0.5f), new Vector3(m.width, m.height, cube.size.z));
            if (cube.hits >= confirmHits)
            {
                Confirm(cube, now);
                UpdateSilhouette(cube, det, frame, 1f);
            }
            return;
        }

        if (scan)
        {
            // A new scan of a known object refines its position (one measurement, no 'moved?' check needed).
            if (Measure(det, frame, scanRays, out var m) && m.hit)
                MoveCube(cube, m, now);
            UpdateSilhouette(cube, det, frame, 0.5f);
            return;
        }

        if (iou >= stillIoU)
        {
            // Agrees with the box: the object has not moved. Nothing to measure; the shape is averaged.
            cube.moveStreak = 0;
            cube.velocity *= 0.5f;
            float speed01 = Mathf.InverseLerp(0f, Mathf.Max(0.01f, fastShapeSpeed), cube.velocity.magnitude);
            UpdateSilhouette(cube, det, frame, Mathf.Lerp(shapeSmoothing, 1f, speed01));
            return;
        }

        // Disagrees: measure again only when it keeps disagreeing (one off result is just YOLO noise).
        cube.moveStreak++;
        if (cube.moveStreak >= moveConfirmCount && canMeasure)
        {
            cube.moveStreak = 0;
            if (Measure(det, frame, liveRays, out var moved) && moved.hit)
                MoveCube(cube, moved, now);
        }
        // It may be moving: follow the newest mask closely instead of averaging it away.
        UpdateSilhouette(cube, det, frame, movingShapeWeight);
    }

    // Adds a matched mask to the object's silhouette (no depth rays: the mask is moved to the object's fixed anchor).
    void UpdateSilhouette(ObjectCube cube, DetectionMsg det, ScanFrame frame, float weight)
    {
        if (drawStyle != DrawStyle.Silhouette || !cube.Visible)
            return;
        cube.silhouette.AddMask(MaskRegions(det, frame), frame, cube.FrontCenter, weight, viewResetAngle);
    }

    // The mask outline(s) in viewport coordinates, or the YOLO box when there is no mask.
    IReadOnlyList<List<Vector2>> MaskRegions(DetectionMsg det, ScanFrame frame)
    {
        var points = det.mask?.data;
        if (points != null && points.Length >= 3)
        {
            var regions = MaskOutline.SplitIntoRegions(points, frame.imageWidth, frame.imageHeight);
            if (regions.Count > 0)
                return regions;
        }
        if (!warnedNoMask)
        {
            warnedNoMask = true;
            Debug.LogWarning("[steelarm] YOLO results carry no masks; silhouettes use the boxes. Check that a -seg model is running.");
        }
        Rect r = DetectionRect(det, frame);
        s_BoxRegion.Clear();
        s_BoxRegion.Add(new Vector2(r.xMin, r.yMin));
        s_BoxRegion.Add(new Vector2(r.xMax, r.yMin));
        s_BoxRegion.Add(new Vector2(r.xMax, r.yMax));
        s_BoxRegion.Add(new Vector2(r.xMin, r.yMax));
        return s_BoxRegions;
    }

    void MoveCube(ObjectCube cube, Measurement m, float now)
    {
        // Size and orientation stay as first measured; only the position follows the object.
        Vector3 measuredCenter = m.front + m.forward * (cube.size.z * 0.5f);
        if (cube.ApplyMeasurement(measuredCenter, now, deadband) && logCubes)
            Debug.Log($"[steelarm] ~ {cube.className} #{cube.serial} moved to {Distance(cube.center):0.00} m");
    }

    void CreateCube(DetectionMsg det, ScanFrame frame, bool scan, bool canMeasure, float now)
    {
        if (!canMeasure)
            return; // head turning fast: wait for a sharper frame
        if (!Measure(det, frame, scan ? scanRays : liveRays, out var m) || !UsableDepth(m, scan))
            return;

        var cube = new ObjectCube(nextSerial++, det.class_name, lineMaterial)
        {
            detection = det,
            trackId = det.id ?? "",
            registered = RobotActionRegistry.GetActions(det.class_name) != null,
            drawBox = drawStyle == DrawStyle.Box,
            rotation = Quaternion.LookRotation(m.forward, Vector3.up),
            createdTime = now,
            lastSeen = now,
            hits = 1
        };
        cube.AddSample(m.front + m.forward * (m.boxDepth * 0.5f), new Vector3(m.width, m.height, m.boxDepth));
        cubes.Add(cube);
        if (!m.hit)
            Debug.LogWarning($"[steelarm] No depth for {det.class_name}; placed at {fallbackDistance} m.");
        if (scan || cube.hits >= confirmHits)
        {
            Confirm(cube, now);
            UpdateSilhouette(cube, det, frame, 1f);
        }
    }

    // Live: a box placed without real depth (depth not ready yet, ray missed) would sit at fallbackDistance and,
    // seen from the same spot, still overlap perfectly, so it would never be corrected. Wait for real depth instead.
    // Scan mode has only this one result, so the fallback is used there (with a warning).
    bool UsableDepth(Measurement m, bool scan) => m.hit || scan || environmentRaycast == null;

    void Confirm(ObjectCube cube, float now)
    {
        cube.Confirm(now, minCutoff, beta);
        if (logCubes)
            Debug.Log($"[steelarm] + {cube.className} #{cube.serial} at {Distance(cube.center):0.00} m, size {cube.size.x:0.00} x {cube.size.y:0.00} x {cube.size.z:0.00} m");
    }

    bool ShouldRemove(ObjectCube cube, bool scan, float now)
    {
        if (cube.state == ObjectCube.State.Tentative)
            return now - cube.createdTime > tentativeTimeout || cube.misses >= 2;
        return cube.misses >= (scan ? 1 : lostMisses);
    }

    void RemoveAt(int index, string reason)
    {
        ObjectCube cube = cubes[index];
        if (cube == hovered)
            hovered = null;
        if (logCubes && cube.Visible)
            Debug.Log($"[steelarm] - {cube.className} #{cube.serial} ({reason})");
        cube.Destroy();
        cubes.RemoveAt(index);
    }

    void RemoveAll()
    {
        for (int j = cubes.Count - 1; j >= 0; j--)
            RemoveAt(j, "cleared");
    }

    // ===================== Geometry =====================

    // The box (at its predicted position) drawn into this camera image, as a viewport rectangle.
    Projection Project(ObjectCube cube, ScanFrame frame, float now)
    {
        var p = new Projection();
        Vector3 c = cube.PredictedCenter(now, maxPrediction);
        ObjectCube.GetCorners(c, cube.size, cube.rotation, corners);
        float xMin = float.MaxValue, yMin = float.MaxValue, xMax = float.MinValue, yMax = float.MinValue;
        for (int k = 0; k < 8; k++)
        {
            if (!frame.WorldToViewport(corners[k], out Vector2 v))
                return p; // partly behind the camera: not comparable, and not 'in view'
            xMin = Mathf.Min(xMin, v.x); xMax = Mathf.Max(xMax, v.x);
            yMin = Mathf.Min(yMin, v.y); yMax = Mathf.Max(yMax, v.y);
        }
        p.valid = true;
        p.rect = Rect.MinMaxRect(xMin, yMin, xMax, yMax);
        // In view = its center is clearly inside the image (an object at the edge may be cut off and missed by YOLO).
        const float margin = 0.05f;
        p.inView = frame.WorldToViewport(c, out Vector2 vc)
                   && vc.x > margin && vc.x < 1f - margin && vc.y > margin && vc.y < 1f - margin;
        return p;
    }

    // Something (a hand, a person) is clearly in front of the box in this image.
    bool IsOccluded(ObjectCube cube, ScanFrame frame)
    {
        if (!occlusionCheck || environmentRaycast == null)
            return false;
        if (!frame.WorldToViewport(cube.center, out Vector2 v))
            return false;
        Ray ray = frame.ViewportRay(v);
        statRays++;
        if (!environmentRaycast.Raycast(ray, out var hit))
            return false;
        float hitDepth = Vector3.Dot(hit.point - ray.origin, frame.forward);
        float cubeDepth = Vector3.Dot(cube.center - ray.origin, frame.forward);
        return hitDepth < cubeDepth - Mathf.Max(0.15f, cube.size.z);
    }

    Rect DetectionRect(DetectionMsg det, ScanFrame frame)
    {
        var center = det.bbox.center.position;
        var size = det.bbox.size;
        Vector2 leftBottom = frame.PixelToViewport(center.x - size.x / 2, center.y + size.y / 2);
        Vector2 rightTop = frame.PixelToViewport(center.x + size.x / 2, center.y - size.y / 2);
        return Rect.MinMaxRect(leftBottom.x, leftBottom.y, rightTop.x, rightTop.y);
    }

    static float IoU(Rect a, Rect b)
    {
        float w = Mathf.Min(a.xMax, b.xMax) - Mathf.Max(a.xMin, b.xMin);
        float h = Mathf.Min(a.yMax, b.yMax) - Mathf.Max(a.yMin, b.yMin);
        if (w <= 0f || h <= 0f)
            return 0f;
        float inter = w * h;
        return inter / (a.width * a.height + b.width * b.height - inter);
    }

    // Measures one detection: front face center, width and height (gravity-aligned), facing the camera horizontally.
    bool Measure(DetectionMsg det, ScanFrame frame, int rayCount, out Measurement m)
    {
        m = new Measurement();
        Rect r = DetectionRect(det, frame);
        if (r.width <= 0f || r.height <= 0f)
            return false;
        statMeasures++;

        // Depth: median of rays inside the object.
        BuildSamplePoints(det, frame, r, Mathf.Max(1, rayCount));
        depths.Clear();
        if (environmentRaycast != null)
        {
            foreach (var v in samples)
            {
                Ray ray = frame.ViewportRay(v);
                statRays++;
                if (environmentRaycast.Raycast(ray, out var hit))
                {
                    float d = Vector3.Dot(hit.point - ray.origin, frame.forward);
                    if (d > 0.05f)
                        depths.Add(d);
                }
            }
        }
        m.hit = depths.Count > 0;
        float front = m.hit ? Median(depths) : fallbackDistance;

        // Horizontal facing direction and a gravity-aligned frame.
        Vector3 forward = Vector3.ProjectOnPlane(frame.forward, Vector3.up);
        if (forward.sqrMagnitude < 1e-4f)
            forward = Vector3.ProjectOnPlane(frame.up, Vector3.up); // looking straight down / up
        forward.Normalize();
        Vector3 right = Vector3.Cross(Vector3.up, forward);

        // The YOLO box at that depth; its extent along right / up gives width / height.
        float rMin = float.MaxValue, rMax = float.MinValue, yMin = float.MaxValue, yMax = float.MinValue, fSum = 0f;
        for (int k = 0; k < 4; k++)
        {
            var corner = new Vector2(k == 0 || k == 1 ? r.xMin : r.xMax, k == 0 || k == 3 ? r.yMin : r.yMax);
            Vector3 p = frame.ViewportToWorld(corner, front);
            float pr = Vector3.Dot(p, right);
            rMin = Mathf.Min(rMin, pr); rMax = Mathf.Max(rMax, pr);
            yMin = Mathf.Min(yMin, p.y); yMax = Mathf.Max(yMax, p.y);
            fSum += Vector3.Dot(p, forward);
        }

        m.forward = forward;
        m.width = Mathf.Max(0.03f, rMax - rMin);
        m.height = Mathf.Max(0.03f, yMax - yMin);
        m.boxDepth = Mathf.Clamp(m.width * depthPerWidth, minBoxDepth, maxBoxDepth);
        m.front = right * ((rMin + rMax) * 0.5f) + forward * (fSum * 0.25f) + Vector3.up * ((yMin + yMax) * 0.5f);
        return true;
    }

    // Viewport points to aim the depth rays at: a grid over the middle of the YOLO box, kept inside the mask when
    // there is one (so a chair's legs or a mug's handle area don't send rays to the wall behind).
    void BuildSamplePoints(DetectionMsg det, ScanFrame frame, Rect r, int count)
    {
        candidates.Clear();
        samples.Clear();
        int grid = Mathf.Max(1, Mathf.CeilToInt(Mathf.Sqrt(count * 2f)));
        const float inset = 0.15f;
        for (int gy = 0; gy < grid; gy++)
        {
            for (int gx = 0; gx < grid; gx++)
            {
                float u = grid == 1 ? 0.5f : Mathf.Lerp(inset, 1f - inset, gx / (float)(grid - 1));
                float w = grid == 1 ? 0.5f : Mathf.Lerp(inset, 1f - inset, gy / (float)(grid - 1));
                candidates.Add(new Vector2(Mathf.Lerp(r.xMin, r.xMax, u), Mathf.Lerp(r.yMin, r.yMax, w)));
            }
        }

        var maskPoints = det.mask?.data;
        if (raysInsideMask && maskPoints != null && maskPoints.Length >= 3)
        {
            var regions = MaskOutline.SplitIntoRegions(maskPoints, frame.imageWidth, frame.imageHeight);
            foreach (var c in candidates)
            {
                foreach (var region in regions)
                {
                    if (Polygon2D.Contains(region, c))
                    {
                        samples.Add(c);
                        break;
                    }
                }
            }
        }
        if (samples.Count < 3)
        {
            samples.Clear();
            samples.AddRange(candidates); // no (usable) mask: the whole grid
        }

        // Thin out evenly to the ray budget.
        if (samples.Count > count)
        {
            candidates.Clear();
            for (int k = 0; k < count; k++)
                candidates.Add(samples[k * samples.Count / count]);
            samples.Clear();
            samples.AddRange(candidates);
        }
    }

    static float Median(List<float> values)
    {
        values.Sort();
        int n = values.Count;
        return n % 2 == 1 ? values[n / 2] : (values[n / 2 - 1] + values[n / 2]) * 0.5f;
    }

    float Distance(Vector3 point)
    {
        Camera cam = Camera.main;
        return cam != null ? Vector3.Distance(cam.transform.position, point) : 0f;
    }

    int CountVisible()
    {
        int n = 0;
        foreach (var cube in cubes)
        {
            if (cube.Visible)
                n++;
        }
        return n;
    }

    // ===================== Interact =====================

    // Highlights the box either controller points at (nearest hit wins). A hand whose XRI ray is on the UI
    // (e.g. the object menu) is ignored.
    void UpdateHover()
    {
        ObjectCube pointed = null;
        float nearest = float.MaxValue;
        foreach (var controller in controllers)
        {
            if (controller.IsPointingAtUI())
                continue;
            ObjectCube hit = FindPointedCube(controller.AimRay(xrOrigin), out float distance);
            if (hit != null && distance < nearest)
            {
                pointed = hit;
                nearest = distance;
            }
        }

        if (pointed != hovered)
        {
            hovered?.SetHighlight(false, outlineColor);
            hovered = pointed;
            hovered?.SetHighlight(true, ColorFor(hovered), hovered.registered ? registeredFill : unregisteredFill);
        }
    }

    void Select(Ray ray)
    {
        ObjectCube pointed = FindPointedCube(ray, out _);
        if (pointed == null || ObjectMenuGenerator._instance == null)
            return;
        ObjectMenuGenerator._instance.Open(MenuTarget.From(pointed, ColorFor(pointed)));
    }

    ObjectCube FindPointedCube(Ray ray, out float nearest)
    {
        ObjectCube pointed = null;
        nearest = float.MaxValue;
        foreach (var cube in cubes)
        {
            if (cube.TryHit(ray, out float distance) && distance < nearest)
            {
                pointed = cube;
                nearest = distance;
            }
        }
        return pointed;
    }

    Color ColorFor(ObjectCube cube) => cube.registered ? boxColor : unregisteredColor;

    static Color WithAlpha(Color c, float a) => new Color(c.r, c.g, c.b, a);

    // ===================== Debug =====================

    void LogStats()
    {
        if (!logStats || Time.time - statSince < 5f)
            return;
        float seconds = Time.time - statSince;
        int tentative = cubes.Count - CountVisible();
        Debug.Log($"[steelarm] boxes: {CountVisible()} shown, {tentative} tentative | last {seconds:0} s: {statResults} results, {statMeasures} measurements, {statRays} depth rays ({statRays / seconds:0} rays/s)");
        statResults = statMeasures = statRays = 0;
        statSince = Time.time;
    }
}
