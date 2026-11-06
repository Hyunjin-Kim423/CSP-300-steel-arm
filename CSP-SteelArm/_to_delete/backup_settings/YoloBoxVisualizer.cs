using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.XR.CoreUtils;
using RosMessageTypes.Yolo;
using UnityEngine.XR.Interaction.Toolkit.Interactors;
using EnvironmentRaycastManager = Meta.XR.EnvironmentRaycastManager;

// Shows the objects YOLO finds in the passthrough camera image and lets the user pick one.
//
//   1. Get results   Scan mode: A / X sends one camera frame, the result stays fixed in place.
//                    Live mode: frames are sent continuously; /yolo/tracking is preferred, /yolo/detections is the fallback.
//   2. Place them    Each detection is placed in the world at the depth measured at its box center,
//                    as a white outline (segmentation silhouette, or the bounding box).
//   3. Interact      The object a controller points at is filled (green: in yolo_dectection_list.json, red: not);
//                    the trigger opens its menu and label.
//
// Helpers (Assets/Script/YoloVisualizer):
//   DetectedObject / DetectedObjectPool   one drawn object and the pool that reuses them
//   MaskOutline                           mask point list -> clean outlines
//   Polygon2D                             fill triangulation, point-in-outline
//   ScanFrame                             image position -> world position for one camera frame
//   ControllerInput                       buttons, aim ray, "is this hand on the UI?"
//   ObjectLabel                           "cup 0.87" text
[RequireComponent(typeof(CameraImagePublisher))]
public class YoloBoxVisualizer : MonoBehaviour
{
    public enum HighlightStyle
    {
        [Tooltip("Bounding boxes only.")]
        BoundingBox,
        [Tooltip("Segmentation outlines (needs a -seg model, e.g. yolov8n-seg.pt). Falls back to the box when a detection has no mask.")]
        Silhouette
    }

    // ===================== Inspector =====================

    [Header("Scan")]
    [Tooltip("Scan only when A (right) or X (left) is pressed and keep the result fixed in place. Off = live scanning.")]
    public bool scanOnButton = true;
    [Tooltip("A new scan replaces only the objects currently in view; objects elsewhere stay. Off = a scan replaces everything.")]
    public bool replaceOnlyInView = true;

    [Header("Scan effect")]
    [Tooltip("Play a wave over the real surfaces when scanning (ScanWaveEffect, uses the headset's depth).")]
    public bool scanWave = true;
    [Tooltip("Scan presses closer together than this (seconds) are ignored.")]
    public float scanCooldown = 0.4f;
    [Tooltip("While the result has not arrived, another wave is sent every this many seconds.")]
    public float waitingWaveInterval = 1.2f;
    [Tooltip("Stop waiting for a scan result after this many seconds.")]
    public float scanResultTimeout = 5f;
    [Tooltip("New outlines flash in this color when a scan result arrives.")]
    public Color flashColor = new Color(0.55f, 0.9f, 1f, 1f);
    public float flashDuration = 0.5f;

    [Tooltip("Untracked detections topic. Used in scan mode, when 'Use Tracking' is off, and as a fallback while the tracking topic is silent.")]
    public string topicName = "/yolo/detections";

    [Header("Tracking & smoothing (live mode)")]
    [Tooltip("Subscribe to the yolo_ros tracking topic so each object keeps an id across frames (needed for smoothing).")]
    public bool useTracking = true;
    public string trackingTopicName = "/yolo/tracking";
    [Tooltip("If the tracking topic sends nothing for this many seconds, results from the detections topic are used instead (e.g. the tracking node crashed). Tracking takes over again as soon as it sends.")]
    public float trackingFallbackTimeout = 1.5f;
    [Tooltip("How fast the drawn outline follows each new YOLO result. Higher = snappier, lower = smoother.")]
    public float followSpeed = 8f;
    [Tooltip("Weight of a new depth measurement (0-1). Lower = steadier distance, slower to react.")]
    [Range(0.05f, 1f)] public float depthSmoothing = 0.35f;
    [Tooltip("A tracked object that stops appearing in the results is hidden after this many seconds.")]
    public float lostTrackTimeout = 1.5f;

    [Header("Placement")]
    [Tooltip("Depth (meters) used when the environment depth raycast misses.")]
    public float fallbackDistance = 1.0f;
    public float lineWidth = 0.004f;

    [Header("Look")]
    public HighlightStyle highlightStyle = HighlightStyle.Silhouette;
    [Tooltip("Outline color for every detected object.")]
    public Color outlineColor = Color.white;
    [Tooltip("Fill color of the pointed object when it is listed in yolo_dectection_list.json (it has robot actions).")]
    public Color boxColor = Color.green;
    [Tooltip("Fill color of the pointed object when it is NOT in yolo_dectection_list.json.")]
    public Color unregisteredColor = Color.red;
    [Tooltip("Opacity of the pointed object's fill.")]
    [Range(0f, 1f)] public float fillAlpha = 0.35f;
    [Tooltip("Outline points per object (shared across its regions; at least 12 per region).")]
    public int maxMaskPoints = 80;

    [Header("Label")]
    [Tooltip("Label text height in meters for an object 1 m away; scales with distance.")]
    public float labelHeight = 0.03f;
    [Tooltip("How long the label stays after pulling the trigger (seconds).")]
    public float labelDuration = 3f;
    [Tooltip("Live mode: everything is hidden if no new results arrive within this many seconds.")]
    public float boxLifetime = 2f;

    // ===================== State =====================

    public static YoloBoxVisualizer Instance { get; private set; }

    CameraImagePublisher publisher;
    EnvironmentRaycastManager environmentRaycast;
    XROrigin xrOrigin;
    ControllerInput[] controllers;
    Material registeredFillMaterial;
    Material unregisteredFillMaterial;
    DetectedObjectPool objects;
    ObjectLabel label;
    DetectedObject hovered;
    float lastDetectionTime;
    bool warnedNoMask;
    bool subscribed;

    // Scan mode
    int scanCount;
    long handledScanTicks = -1;
    ScanWaveEffect wave;
    float lastScanPress = float.NegativeInfinity;
    float scanPendingSince = -1f;   // < 0: no scan result expected
    float lastWaveTime;

    // Live mode: which topic the shown objects come from
    enum Source { None, Tracking, Detections }
    Source currentSource = Source.None;
    float lastTrackingTime = float.NegativeInfinity;

    static readonly List<Vector2> s_Resampled = new List<Vector2>();

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
        publisher.continuousPublish = !scanOnButton; // scan mode: frames are only sent on request
        xrOrigin = FindAnyObjectByType<XROrigin>();
        controllers = new[]
        {
            new ControllerInput("RightHand", InteractorHandedness.Right),
            new ControllerInput("LeftHand", InteractorHandedness.Left)
        };

        var lineMaterial = new Material(Shader.Find("Sprites/Default"));
        registeredFillMaterial = new Material(Shader.Find("Sprites/Default")) { color = WithAlpha(boxColor, fillAlpha) };
        unregisteredFillMaterial = new Material(Shader.Find("Sprites/Default")) { color = WithAlpha(unregisteredColor, fillAlpha) };
        objects = new DetectedObjectPool(lineMaterial, registeredFillMaterial, outlineColor, lineWidth);
        label = new ObjectLabel(labelHeight, labelDuration, boxColor);
        if (scanWave && !TryGetComponent(out wave))
            wave = gameObject.AddComponent<ScanWaveEffect>();

        if (EnvironmentRaycastManager.IsSupported)
        {
            OVRPermissionsRequester.Request(new[] { OVRPermissionsRequester.Permission.Scene });
            environmentRaycast = FindAnyObjectByType<EnvironmentRaycastManager>();
            if (environmentRaycast == null)
                environmentRaycast = gameObject.AddComponent<EnvironmentRaycastManager>();
        }
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
        if (scanOnButton)
        {
            if (AnyScanPressed())
                RequestScan();
            KeepWavingWhileWaiting();
        }
        else
        {
            // Live mode: drop everything when results stop coming, and objects that stopped appearing.
            if (Time.time - lastDetectionTime > boxLifetime)
                objects.RemoveAll();
            else
                objects.RemoveWhere(obj => Time.time - obj.lastSeen > lostTrackTimeout);
        }

        label.Tick();

        float t = 1f - Mathf.Exp(-followSpeed * Time.deltaTime);
        foreach (var obj in objects.Active)
        {
            obj.Animate(t);
            obj.UpdateFlash(Time.time, flashDuration, flashColor, outlineColor, lineWidth);
        }

        UpdateHover();

        foreach (var controller in controllers)
        {
            if (controller.SelectPressed && !controller.IsPointingAtUI())
                Select(controller.AimRay(xrOrigin));
        }
    }

    // ===================== 1. Get results =====================

    // Called once at start (StartWithControllerButton, or the CoachingUI Start button). Extra calls are ignored,
    // so results are never subscribed twice.
    public void SubscribeROS()
    {
        if (subscribed)
            return;
        subscribed = true;
        var ros = ROSConnection.GetOrCreateInstance();
        // Both topics are subscribed: tracking is preferred, detections keep things working if tracking stops.
        ros.Subscribe<DetectionArrayMsg>(topicName, OnUntrackedDetections);
        if (useTracking)
            ros.Subscribe<DetectionArrayMsg>(trackingTopicName, OnTrackedDetections);
    }

    void OnTrackedDetections(DetectionArrayMsg msg)
    {
        // A single scan frame gives the tracker nothing to track, so scan mode uses the plain detections.
        if (scanOnButton)
            return;
        lastTrackingTime = Time.time;
        UseSource(Source.Tracking);
        ShowResult(msg);
    }

    void OnUntrackedDetections(DetectionArrayMsg msg)
    {
        if (scanOnButton)
        {
            OnScanResult(msg);
            return;
        }
        // While tracking is alive its results win; the untracked copy of the same frame is ignored.
        if (useTracking && Time.time - lastTrackingTime < trackingFallbackTimeout)
            return;
        UseSource(Source.Detections);
        ShowResult(msg);
    }

    void UseSource(Source source)
    {
        if (source == currentSource)
            return;
        // Object keys differ between the sources (track ids vs. list positions), so start over to avoid duplicates.
        objects.RemoveAll();
        if (currentSource != Source.None)
        {
            if (source == Source.Detections)
                Debug.LogWarning($"[steelarm] No results on '{trackingTopicName}' for {trackingFallbackTimeout:0.0}s; using '{topicName}' (no tracking/smoothing). Is the tracking node running?");
            else
                Debug.Log($"[steelarm] '{trackingTopicName}' is sending again; using tracked results.");
        }
        currentSource = source;
    }

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
        // Ignore rapid repeated presses.
        if (Time.time - lastScanPress < scanCooldown)
            return;
        lastScanPress = Time.time;

        // The open menu belongs to the old scan; its object may be replaced, so close it first.
        if (ObjectMenuGenerator._instance != null)
            ObjectMenuGenerator._instance.CloseCurrent();
        label.Hide();

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

    // The result is still on its way: send another wave now and then, so the player sees the scan is running.
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

    // Builds fixed outlines from the result of the latest scan frame (other frames and duplicates are ignored).
    void OnScanResult(DetectionArrayMsg msg)
    {
        if (!publisher.IsScanFrame(msg.header.stamp, out long ticks) || ticks == handledScanTicks)
            return;
        handledScanTicks = ticks;
        scanCount++;

        // The new scan replaces what is in view (it was just scanned again), or everything.
        Camera cam = Camera.main;
        if (replaceOnlyInView && cam != null)
            objects.RemoveWhere(obj => IsInView(cam, obj.Center));
        else
            objects.RemoveAll();

        scanPendingSince = -1f;
        ShowResult(msg);

        // The new outlines flash once as they appear.
        string scanPrefix = $"s{scanCount}:";
        foreach (var obj in objects.Active)
        {
            if (obj.key.StartsWith(scanPrefix))
                obj.flashStart = Time.time;
        }
        Debug.Log($"[steelarm] Scan {scanCount}: {msg.detections.Length} object(s)");
    }

    static bool IsInView(Camera cam, Vector3 point)
    {
        Vector3 viewport = cam.WorldToViewportPoint(point);
        return viewport.z > 0f && viewport.x >= 0f && viewport.x <= 1f && viewport.y >= 0f && viewport.y <= 1f;
    }

    // ===================== 2. Place them =====================

    void ShowResult(DetectionArrayMsg msg)
    {
        if (!publisher.CameraAccess.IsPlaying) return;
        if (!publisher.TryGetFrame(msg.header.stamp, out var info)) return;
        var frame = new ScanFrame(publisher.CameraAccess, info.cameraPose, info.imageSize);

        bool anyMask = false;
        for (int index = 0; index < msg.detections.Length; index++)
            anyMask |= PlaceObject(msg.detections[index], index, frame);

        // Untracked live results (#n) only live for one message.
        objects.RemoveWhere(obj => obj.key.StartsWith("#") && obj.lastSeen < Time.time);
        lastDetectionTime = Time.time;

        if (!warnedNoMask && highlightStyle == HighlightStyle.Silhouette && msg.detections.Length > 0 && !anyMask)
        {
            warnedNoMask = true;
            Debug.LogWarning("[steelarm] YOLO results carry no masks; drawing boxes. Check that a -seg model is running.");
        }

        // Outlines changed: rebuild the hovered object's fill with the new shape.
        if (hovered != null && hovered.active)
            hovered.ShowFill(FillMaterialFor(hovered));
    }

    // Creates or updates the drawn object for one detection. Returns true when it got a mask outline.
    bool PlaceObject(DetectionMsg det, int index, ScanFrame frame)
    {
        // Scan mode: every scan gets its own keys, so earlier scans outside the view are kept.
        // Live mode: tracked results carry a stable id; untracked ones are keyed by their position in the message.
        string key = scanOnButton ? $"s{scanCount}:{index}"
            : string.IsNullOrEmpty(det.id) ? "#" + index : det.id;
        DetectedObject obj = objects.GetOrCreate(key, out bool isNew);
        // Without a stable id (#n), index n can be a different object next time, so never blend those.
        bool sameObject = !isNew && !key.StartsWith("#");

        // Bounding box in viewport coordinates.
        var center = det.bbox.center.position;
        var size = det.bbox.size;
        Vector2 leftBottom = frame.PixelToViewport(center.x - size.x / 2, center.y + size.y / 2);
        Vector2 rightTop = frame.PixelToViewport(center.x + size.x / 2, center.y - size.y / 2);

        // Distance, measured at the box center (blended with the previous value for the same object).
        float depth = MeasureDepth(frame, (leftBottom + rightTop) / 2);
        if (sameObject && obj.hasDepth)
            depth = Mathf.Lerp(obj.depth, depth, depthSmoothing);
        obj.depth = depth;
        obj.hasDepth = true;

        obj.targetCorners[0] = frame.ViewportToWorld(new Vector2(leftBottom.x, leftBottom.y), depth);
        obj.targetCorners[1] = frame.ViewportToWorld(new Vector2(leftBottom.x, rightTop.y), depth);
        obj.targetCorners[2] = frame.ViewportToWorld(new Vector2(rightTop.x, rightTop.y), depth);
        obj.targetCorners[3] = frame.ViewportToWorld(new Vector2(rightTop.x, leftBottom.y), depth);

        obj.detection = det;
        obj.text = $"{det.class_name} {det.score:0.00}";
        // Green if the object has robot actions in yolo_dectection_list.json, red otherwise.
        obj.color = RobotActionRegistry.GetActions(det.class_name) != null ? boxColor : unregisteredColor;
        obj.cameraPosition = frame.CameraPosition;
        obj.lastSeen = Time.time;

        // Outline: silhouette from the mask, or the bounding box.
        bool hadMask = obj.hasMask;
        int previousPartCount = obj.partCount;
        obj.hasMask = highlightStyle == HighlightStyle.Silhouette && SetSilhouetteTarget(obj, det.mask, frame, depth);
        if (!obj.hasMask)
            SetBoxTarget(obj);

        // New object or a different outline layout: jump to the new shape instead of morphing.
        if (!sameObject || hadMask != obj.hasMask || previousPartCount != obj.partCount)
            obj.SnapToTarget();
        obj.ShowOutline();
        return obj.hasMask;
    }

    // Distance from the camera to the real surface seen at this viewport position (along the camera's optical axis).
    float MeasureDepth(ScanFrame frame, Vector2 viewport)
    {
        Ray ray = frame.ViewportRay(viewport);
        if (environmentRaycast != null && environmentRaycast.Raycast(ray, out var hit))
            return Vector3.Dot(hit.point - ray.origin, frame.forward);
        return fallbackDistance;
    }

    void SetBoxTarget(DetectedObject obj)
    {
        DetectedObject.SetTarget(objects.GetPart(obj, 0), (Vector3[])obj.targetCorners.Clone(), align: false);
        obj.partCount = 1;
    }

    // One outline part per mask region, placed on the plane at the object's depth. Returns false without a usable mask.
    bool SetSilhouetteTarget(DetectedObject obj, MaskMsg mask, ScanFrame frame, float depth)
    {
        var points = mask?.data;
        if (points == null || points.Length < 3)
            return false;

        var regions = MaskOutline.SplitIntoRegions(points, frame.imageWidth, frame.imageHeight);
        if (regions.Count == 0)
            return false;

        // A fixed point count per region lets consecutive outlines be morphed point by point.
        int pointsPerRegion = Mathf.Max(12, maxMaskPoints / regions.Count);
        bool sameLayout = obj.hasMask && obj.partCount == regions.Count;
        for (int r = 0; r < regions.Count; r++)
        {
            MaskOutline.Resample(regions[r], pointsPerRegion, s_Resampled);
            var world = new Vector3[s_Resampled.Count];
            for (int i = 0; i < world.Length; i++)
                world[i] = frame.ViewportToWorld(s_Resampled[i], depth);
            DetectedObject.SetTarget(objects.GetPart(obj, r), world, align: sameLayout);
        }
        obj.partCount = regions.Count;
        return true;
    }

    // ===================== 3. Interact =====================

    // Fills the object either controller points at (nearest hit wins). A hand whose XRI ray is on the UI
    // (e.g. the object menu) is ignored, so the object behind the menu is not highlighted or selected.
    void UpdateHover()
    {
        if (hovered != null && !hovered.active)
            hovered = null;

        DetectedObject pointed = null;
        float nearest = float.MaxValue;
        foreach (var controller in controllers)
        {
            if (controller.IsPointingAtUI())
                continue;
            DetectedObject hit = FindPointedObject(controller.AimRay(xrOrigin), out float distance);
            if (hit != null && distance < nearest)
            {
                pointed = hit;
                nearest = distance;
            }
        }

        if (pointed != hovered)
        {
            hovered?.HideFill();
            hovered = pointed;
            hovered?.ShowFill(FillMaterialFor(hovered));
        }
        hovered?.UpdateFill();
    }

    void Select(Ray ray)
    {
        DetectedObject pointed = FindPointedObject(ray, out _);
        if (pointed == null)
        {
            // label.Hide();
            return;
        }
        ObjectMenuGenerator._instance.Open(MenuTarget.From(pointed));
        // label.Show(pointed);
    }

    DetectedObject FindPointedObject(Ray ray, out float nearest)
    {
        DetectedObject pointed = null;
        nearest = float.MaxValue;
        foreach (var obj in objects.Active)
        {
            if (obj.TryHit(ray, out float distance) && distance < nearest)
            {
                pointed = obj;
                nearest = distance;
            }
        }
        return pointed;
    }

    Material FillMaterialFor(DetectedObject obj) => obj.color == boxColor ? registeredFillMaterial : unregisteredFillMaterial;

    static Color WithAlpha(Color c, float a) => new Color(c.r, c.g, c.b, a);
}
