using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.InputSystem;
using Unity.Robotics.ROSTCPConnector;
using Unity.XR.CoreUtils;
using RosMessageTypes.Yolo;
using UnityEngine.XR.Interaction.Toolkit.Interactors;
using UnityEngine.XR.Interaction.Toolkit.UI;
using EnvironmentRaycastManager = Meta.XR.EnvironmentRaycastManager;
using PassthroughCameraAccess = Meta.XR.PassthroughCameraAccess;

// Draws every YOLO detection as a thin white outline (segmentation silhouette, or the bounding box when there is no mask).
// Only the object the controller is pointing at gets its silhouette filled, colored by whether it is listed in
// yolo_dectection_list.json.
//
// Temporal smoothing: detections come from /yolo/tracking (ByteTrack in yolo_ros), so the same object keeps the same id.
// Each tracked object keeps its own outline; new results only set a target shape and the drawn shape follows it smoothly
// every frame (outlines are resampled to a fixed point count so they can be morphed point by point).
[RequireComponent(typeof(CameraImagePublisher))]
public class YoloBoxVisualizer : MonoBehaviour
{
    // One closed outline region (+ its fill). A person partly hidden behind a chair can have several.
    class Piece
    {
        public LineRenderer line;
        public MeshFilter fill;
        public Vector3[] target = new Vector3[0];   // latest shape from YOLO (world space)
        public Vector3[] current = new Vector3[0];  // drawn shape, follows target every frame
        public int[] triangles;                     // fill triangulation of the target shape (same vertex order)
    }

    class Box
    {
        public string key;
        public bool active;
        public float lastSeen;
        public bool hasMask;
        public readonly List<Piece> pieces = new List<Piece>();
        public int pieceCount;
        public Color color;                                   // fill/label color (registered or not)
        public readonly Vector3[] targetCorners = new Vector3[4];
        public readonly Vector3[] corners = new Vector3[4];   // drawn box: 0 left-bottom, 1 left-top, 2 right-top, 3 right-bottom
        public bool hasDepth;
        public float depth;
        public string text;
        public Vector3 cameraPosition;
        public DetectionMsg detectionMsg;
    }

    class Controller
    {
        public readonly InputAction trigger;
        public readonly InputAction position;
        public readonly InputAction rotation;
        public readonly InteractorHandedness handedness;

        public Controller(string hand, InteractorHandedness handedness)
        {
            this.handedness = handedness;
            trigger = new InputAction(type: InputActionType.Button, binding: $"<XRController>{{{hand}}}/{{TriggerButton}}");
            position = new InputAction(binding: $"<XRController>{{{hand}}}/pointerPosition");
            rotation = new InputAction(binding: $"<XRController>{{{hand}}}/pointerRotation");
            trigger.Enable();
            position.Enable();
            rotation.Enable();
        }

        public void Dispose()
        {
            trigger.Dispose();
            position.Dispose();
            rotation.Dispose();
        }
    }

    public enum HighlightStyle
    {
        [Tooltip("Bounding boxes only.")]
        BoundingBox,
        [Tooltip("Segmentation outlines (needs a -seg model, e.g. yolov8n-seg.pt). Falls back to the box when a detection has no mask.")]
        Silhouette
    }

    [Tooltip("Untracked detections topic. Used when 'Use Tracking' is off, and as a fallback while the tracking topic is silent.")]
    public string topicName = "/yolo/detections";

    [Header("Tracking & smoothing")]
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
    [Tooltip("Everything is hidden if no new results arrive within this many seconds.")]
    public float boxLifetime = 2f;

    // XRI interactors that can point at UI (Near-Far / Ray interactors on the controllers and hands).
    readonly List<XRBaseInteractor> uiInteractors = new List<XRBaseInteractor>();

    CameraImagePublisher publisher;
    EnvironmentRaycastManager environmentRaycast;
    XROrigin xrOrigin;
    Controller[] controllers;
    Material lineMaterial;
    Material registeredFillMaterial;
    Material unregisteredFillMaterial;
    TextMeshPro label;
    readonly List<Box> boxes = new List<Box>();                       // every box object ever created (pooled)
    readonly Dictionary<string, Box> activeBoxes = new Dictionary<string, Box>();
    readonly Stack<Box> freeBoxes = new Stack<Box>();
    readonly List<Box> scratchBoxes = new List<Box>();
    Box hoveredBox;
    float lastDetectionTime;
    float labelHideTime;
    bool warnedNoMask;

    enum Source { None, Tracking, Detections }
    Source currentSource = Source.None;
    float lastTrackingTime = float.NegativeInfinity;

    public static YoloBoxVisualizer Instance { get; private set; }

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
        xrOrigin = FindAnyObjectByType<XROrigin>();
        controllers = new[]
        {
            new Controller("RightHand", InteractorHandedness.Right),
            new Controller("LeftHand", InteractorHandedness.Left)
        };
        CacheUIInteractors();
        lineMaterial = new Material(Shader.Find("Sprites/Default"));
        registeredFillMaterial = new Material(Shader.Find("Sprites/Default")) { color = WithAlpha(boxColor, fillAlpha) };
        unregisteredFillMaterial = new Material(Shader.Find("Sprites/Default")) { color = WithAlpha(unregisteredColor, fillAlpha) };

        label = new GameObject("YoloLabel", typeof(RectTransform)).AddComponent<TextMeshPro>();
        label.rectTransform.pivot = Vector2.zero;
        label.rectTransform.sizeDelta = Vector2.zero;
        label.alignment = TextAlignmentOptions.BottomLeft;
        label.textWrappingMode = TextWrappingModes.NoWrap;
        label.color = boxColor;
        label.outlineWidth = 0.2f;
        label.outlineColor = new Color32(0, 0, 0, 255);
        label.gameObject.SetActive(false);

        if (EnvironmentRaycastManager.IsSupported)
        {
            OVRPermissionsRequester.Request(new[] { OVRPermissionsRequester.Permission.Scene });
            environmentRaycast = FindAnyObjectByType<EnvironmentRaycastManager>();
            if (environmentRaycast == null)
                environmentRaycast = gameObject.AddComponent<EnvironmentRaycastManager>();
        }
    }

    public void SubscribeROS()
    {
        var ros = ROSConnection.GetOrCreateInstance();
        // Both topics are subscribed: tracking is preferred, detections keep things working if tracking stops.
        ros.Subscribe<DetectionArrayMsg>(topicName, OnUntrackedDetections);
        if (useTracking)
            ros.Subscribe<DetectionArrayMsg>(trackingTopicName, OnTrackedDetections);
    }

    void OnTrackedDetections(DetectionArrayMsg msg)
    {
        lastTrackingTime = Time.time;
        UseSource(Source.Tracking);
        OnDetections(msg);
    }

    void OnUntrackedDetections(DetectionArrayMsg msg)
    {
        // While tracking is alive its results win; the untracked copy of the same frame is ignored.
        if (useTracking && Time.time - lastTrackingTime < trackingFallbackTimeout)
            return;
        UseSource(Source.Detections);
        OnDetections(msg);
    }

    void UseSource(Source source)
    {
        if (source == currentSource)
            return;
        // Object keys differ between the sources (track ids vs. list positions), so start over to avoid duplicates.
        DeactivateAll();
        if (currentSource != Source.None)
        {
            if (source == Source.Detections)
                Debug.LogWarning($"[steelarm] No results on '{trackingTopicName}' for {trackingFallbackTimeout:0.0}s; using '{topicName}' (no tracking/smoothing). Is the tracking node running?");
            else
                Debug.Log($"[steelarm] '{trackingTopicName}' is sending again; using tracked results.");
        }
        currentSource = source;
    }

    void OnDestroy()
    {
        foreach (var controller in controllers)
            controller.Dispose();
    }

    void Update()
    {
        if (Time.time - lastDetectionTime > boxLifetime)
            DeactivateAll();
        if (Time.time > labelHideTime)
            label.gameObject.SetActive(false);

        AnimateBoxes();

        // Hover: fill the object either controller points at (nearest hit wins).
        // A controller whose XRI ray is on a UI element (e.g. the object menu) is ignored here, so the
        // object behind the menu is neither highlighted nor selected while the user presses a menu button.
        Box pointed = null;
        float nearest = float.MaxValue;
        foreach (var controller in controllers)
        {
            if (IsPointingAtUI(controller.handedness))
                continue;
            Box hit = FindPointedBox(AimRay(controller), out float distance);
            if (hit != null && distance < nearest)
            {
                pointed = hit;
                nearest = distance;
            }
        }
        SetHovered(pointed);
        if (hoveredBox != null)
            UpdateFillVertices(hoveredBox);

        foreach (var controller in controllers)
        {
            if (controller.trigger.WasPressedThisFrame() && !IsPointingAtUI(controller.handedness))
                SelectPointedBox(AimRay(controller));
        }
    }

    // Moves every drawn outline a step towards its latest target and drops tracks that stopped appearing.
    void AnimateBoxes()
    {
        float t = 1f - Mathf.Exp(-followSpeed * Time.deltaTime);
        scratchBoxes.Clear();
        scratchBoxes.AddRange(activeBoxes.Values);
        foreach (var box in scratchBoxes)
        {
            if (Time.time - box.lastSeen > lostTrackTimeout)
            {
                Deactivate(box);
                continue;
            }
            for (int c = 0; c < 4; c++)
                box.corners[c] = Vector3.Lerp(box.corners[c], box.targetCorners[c], t);
            for (int i = 0; i < box.pieceCount; i++)
            {
                Piece piece = box.pieces[i];
                for (int k = 0; k < piece.current.Length; k++)
                    piece.current[k] = Vector3.Lerp(piece.current[k], piece.target[k], t);
                piece.line.SetPositions(piece.current);
            }
        }
    }

    void CacheUIInteractors()
    {
        uiInteractors.Clear();
        foreach (var interactor in FindObjectsByType<XRBaseInteractor>(FindObjectsInactive.Include, FindObjectsSortMode.None))
        {
            if (interactor is IUIInteractor)
                uiInteractors.Add(interactor);
        }
        if (uiInteractors.Count == 0)
            Debug.LogWarning("[steelarm] No XRI UI interactors found; menu clicks may also select objects behind the menu.");
    }

    // True when an active XRI interactor of this hand currently has its ray on a UI element.
    bool IsPointingAtUI(InteractorHandedness handedness)
    {
        foreach (var interactor in uiInteractors)
        {
            if (interactor == null || !interactor.isActiveAndEnabled || interactor.handedness != handedness)
                continue;
            if (((IUIInteractor)interactor).TryGetUIModel(out TrackedDeviceModel model) && model.currentRaycast.isValid)
                return true;
        }
        return false;
    }

    Ray AimRay(Controller controller)
    {
        // Controller poses are reported in tracking space, relative to the XR Origin's camera offset.
        Transform trackingSpace = xrOrigin.CameraFloorOffsetObject.transform;
        Vector3 position = trackingSpace.TransformPoint(controller.position.ReadValue<Vector3>());
        Quaternion rotation = trackingSpace.rotation * controller.rotation.ReadValue<Quaternion>();
        return new Ray(position, rotation * Vector3.forward);
    }

    // The nearest object whose drawn silhouette (or box, when it has no mask) contains the ray's hit point.
    // Testing the silhouette instead of the box keeps touching/overlapping objects apart.
    Box FindPointedBox(Ray ray, out float nearest)
    {
        Box pointed = null;
        nearest = float.MaxValue;
        foreach (var box in activeBoxes.Values)
        {
            if (!RayHitsBox(ray, box.corners, out float distance, out Vector2 boxUV) || distance >= nearest)
                continue;
            if (box.hasMask && !SilhouetteContains(box, boxUV))
                continue;
            pointed = box;
            nearest = distance;
        }
        return pointed;
    }

    // Box-plane coordinates (0..1 across the box) of a world point, using the drawn box.
    static Vector2 ToBoxUV(Vector3[] corners, Vector3 p)
    {
        Vector3 d = p - corners[0];
        Vector3 right = corners[3] - corners[0];
        Vector3 up = corners[1] - corners[0];
        return new Vector2(Vector3.Dot(d, right) / right.sqrMagnitude, Vector3.Dot(d, up) / up.sqrMagnitude);
    }

    static readonly List<Vector2> s_PolygonUV = new List<Vector2>();

    static bool SilhouetteContains(Box box, Vector2 uv)
    {
        for (int i = 0; i < box.pieceCount; i++)
        {
            s_PolygonUV.Clear();
            foreach (var p in box.pieces[i].current)
                s_PolygonUV.Add(ToBoxUV(box.corners, p));
            if (PointInPolygon(s_PolygonUV, uv))
                return true;
        }
        return false;
    }

    // Even-odd rule.
    static bool PointInPolygon(List<Vector2> poly, Vector2 p)
    {
        bool inside = false;
        for (int i = 0, j = poly.Count - 1; i < poly.Count; j = i++)
        {
            Vector2 a = poly[i], b = poly[j];
            if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)
                inside = !inside;
        }
        return inside;
    }

    void SelectPointedBox(Ray ray)
    {
        Box pointed = FindPointedBox(ray, out float nearest);
        if (pointed == null)
        {
            label.gameObject.SetActive(false);
            return;
        }

        // 가장 가까운 대상이 확정된 뒤 한 번만 메뉴 생성
        ObjectMenuGenerator._instance.GenerateUI(ray, nearest, pointed.detectionMsg);

        Vector3 topLeft = pointed.corners[1];
        label.text = pointed.text;
        label.color = pointed.color;
        label.fontSize = labelHeight * pointed.depth * 10f;
        label.transform.SetPositionAndRotation(topLeft, Quaternion.LookRotation(topLeft - pointed.cameraPosition));
        label.gameObject.SetActive(true);
        labelHideTime = Time.time + labelDuration;
    }

    static bool RayHitsBox(Ray ray, Vector3[] corners, out float distance, out Vector2 uv)
    {
        uv = default;
        if (!new Plane(corners[0], corners[1], corners[2]).Raycast(ray, out distance))
            return false;
        uv = ToBoxUV(corners, ray.GetPoint(distance));
        return uv.x >= 0f && uv.x <= 1f && uv.y >= 0f && uv.y <= 1f;
    }

    void OnDetections(DetectionArrayMsg msg)
    {
        var cameraAccess = publisher.CameraAccess;
        if (!cameraAccess.IsPlaying) return;
        if (!publisher.TryGetFrame(msg.header.stamp, out var frame)) return;

        Pose pose = frame.cameraPose;
        Vector3 forward = pose.rotation * Vector3.forward;
        float w = frame.imageSize.x;
        float h = frame.imageSize.y;
        bool anyMask = false;

        for (int index = 0; index < msg.detections.Length; index++)
        {
            var det = msg.detections[index];
            // Tracked results carry a stable id; untracked ones are keyed by their position in this message.
            string key = string.IsNullOrEmpty(det.id) ? "#" + index : det.id;
            Box box = GetOrCreateBox(key, out bool isNew);

            var center = det.bbox.center.position;
            var size = det.bbox.size;
            // YOLO pixels have a top-left origin; the camera viewport has a bottom-left origin.
            float left = (float)(center.x - size.x / 2) / w;
            float right = (float)(center.x + size.x / 2) / w;
            float top = 1f - (float)(center.y - size.y / 2) / h;
            float bottom = 1f - (float)(center.y + size.y / 2) / h;

            Ray centerRay = cameraAccess.ViewportPointToRay(new Vector2((left + right) / 2, (top + bottom) / 2), pose);
            float depth = DepthAlong(centerRay, forward);
            // Same object as before: blend the new distance in, so one noisy depth reading doesn't make it jump.
            if (!isNew && box.hasDepth)
                depth = Mathf.Lerp(box.depth, depth, depthSmoothing);
            box.depth = depth;
            box.hasDepth = true;

            box.targetCorners[0] = PointAtDepth(cameraAccess, pose, forward, left, bottom, depth);
            box.targetCorners[1] = PointAtDepth(cameraAccess, pose, forward, left, top, depth);
            box.targetCorners[2] = PointAtDepth(cameraAccess, pose, forward, right, top, depth);
            box.targetCorners[3] = PointAtDepth(cameraAccess, pose, forward, right, bottom, depth);
            // Fill/label color: green if the object has robot actions in yolo_dectection_list.json, red otherwise.
            box.color = RobotActionRegistry.GetActions(det.class_name) != null ? boxColor : unregisteredColor;

            bool hadMask = box.hasMask;
            int previousPieceCount = box.pieceCount;
            box.hasMask = highlightStyle == HighlightStyle.Silhouette
                && BuildSilhouetteTargets(box, det.mask, cameraAccess, pose, forward, depth, w, h);
            if (!box.hasMask)
                BuildBoxTargets(box);
            anyMask |= box.hasMask;

            // New object or different topology (region count / box vs mask): jump straight to the new shape.
            bool snap = isNew || hadMask != box.hasMask || previousPieceCount != box.pieceCount;
            if (snap)
            {
                System.Array.Copy(box.targetCorners, box.corners, 4);
                for (int i = 0; i < box.pieceCount; i++)
                    System.Array.Copy(box.pieces[i].target, box.pieces[i].current, box.pieces[i].target.Length);
            }
            for (int i = 0; i < box.pieceCount; i++)
            {
                Piece piece = box.pieces[i];
                piece.line.gameObject.SetActive(true);
                piece.line.positionCount = piece.current.Length;
                piece.line.SetPositions(piece.current);
            }
            HidePiecesFrom(box, box.pieceCount);

            box.text = $"{det.class_name} {det.score:0.00}";
            box.cameraPosition = pose.position;
            box.detectionMsg = det;
            box.lastSeen = Time.time;
        }

        // Untracked boxes only live for one message.
        scratchBoxes.Clear();
        foreach (var box in activeBoxes.Values)
        {
            if (box.key.StartsWith("#") && box.lastSeen < Time.time)
                scratchBoxes.Add(box);
        }
        foreach (var box in scratchBoxes)
            Deactivate(box);

        lastDetectionTime = Time.time;

        if (!warnedNoMask && highlightStyle == HighlightStyle.Silhouette && msg.detections.Length > 0 && !anyMask)
        {
            warnedNoMask = true;
            Debug.LogWarning($"[steelarm] Detections on '{(currentSource == Source.Tracking ? trackingTopicName : topicName)}' carry no masks; drawing boxes. Check that a -seg model is running.");
        }

        // Outlines changed: rebuild the hovered object's fill with the new shape.
        if (hoveredBox != null && hoveredBox.active)
            ShowFill(hoveredBox);
    }

    // Returns the object's distance along the camera's optical axis.
    float DepthAlong(Ray ray, Vector3 forward)
    {
        if (environmentRaycast != null && environmentRaycast.Raycast(ray, out var hit))
            return Vector3.Dot(hit.point - ray.origin, forward);
        return fallbackDistance;
    }

    static Vector3 PointAtDepth(PassthroughCameraAccess cameraAccess, Pose pose, Vector3 forward, float x, float y, float depth)
    {
        Ray ray = cameraAccess.ViewportPointToRay(new Vector2(x, y), pose);
        return ray.origin + ray.direction * (depth / Vector3.Dot(ray.direction, forward));
    }

    static Color WithAlpha(Color c, float a) => new Color(c.r, c.g, c.b, a);

    // ---------- target shapes ----------

    void BuildBoxTargets(Box box)
    {
        Piece piece = GetPiece(box, 0);
        SetTarget(piece, box.targetCorners, align: false);
        box.pieceCount = 1;
    }

    static readonly List<Vector2> s_Resampled = new List<Vector2>();

    // Splits the mask into its regions and turns each into a fixed-size outline. Returns false when there is no usable mask.
    bool BuildSilhouetteTargets(Box box, MaskMsg mask, PassthroughCameraAccess cameraAccess, Pose pose, Vector3 forward, float depth, float w, float h)
    {
        var points = mask?.data;
        if (points == null || points.Length < 3)
            return false;

        SplitIntoRegions(points, w, h);
        if (s_Regions.Count == 0)
            return false;

        // A fixed point count per region (for a given region count) lets consecutive shapes be morphed point by point.
        int pointsPerRegion = Mathf.Max(12, maxMaskPoints / s_Regions.Count);
        bool sameTopology = box.hasMask && box.pieceCount == s_Regions.Count;
        for (int r = 0; r < s_Regions.Count; r++)
        {
            ResampleClosed(s_Regions[r], pointsPerRegion, s_Resampled);
            var world = new Vector3[s_Resampled.Count];
            for (int i = 0; i < world.Length; i++)
                world[i] = PointAtDepth(cameraAccess, pose, forward, s_Resampled[i].x, s_Resampled[i].y, depth);
            SetTarget(GetPiece(box, r), world, align: sameTopology);
        }
        box.pieceCount = s_Regions.Count;
        return true;
    }

    // Stores a new target shape. With align, the points are rotated so point k lands next to the currently drawn
    // point k, which keeps the morph from sliding around the outline.
    static void SetTarget(Piece piece, Vector3[] target, bool align)
    {
        int n = target.Length;
        if (align && piece.current.Length == n)
        {
            int bestShift = 0;
            float bestCost = float.MaxValue;
            for (int shift = 0; shift < n; shift++)
            {
                float cost = 0f;
                for (int k = 0; k < n && cost < bestCost; k++)
                    cost += (target[(k + shift) % n] - piece.current[k]).sqrMagnitude;
                if (cost < bestCost)
                {
                    bestCost = cost;
                    bestShift = shift;
                }
            }
            var rotated = new Vector3[n];
            for (int k = 0; k < n; k++)
                rotated[k] = target[(k + bestShift) % n];
            target = rotated;
        }
        else if (piece.current.Length != n)
        {
            piece.current = (Vector3[])target.Clone();
        }
        piece.target = target;
        piece.triangles = null; // fill must be re-triangulated for the new shape
    }

    // Resamples a closed polygon to exactly n points evenly spaced along its perimeter, counter-clockwise.
    static void ResampleClosed(List<Vector2> poly, int n, List<Vector2> result)
    {
        result.Clear();
        int count = poly.Count;
        float area = 0f, perimeter = 0f;
        for (int i = 0; i < count; i++)
        {
            Vector2 a = poly[i], b = poly[(i + 1) % count];
            area += a.x * b.y - b.x * a.y;
            perimeter += Vector2.Distance(a, b);
        }
        bool reverse = area < 0f;
        Vector2 P(int i) => poly[reverse ? (count - 1 - (i % count)) : (i % count)];

        if (perimeter <= 0f)
        {
            for (int k = 0; k < n; k++)
                result.Add(P(0));
            return;
        }

        float spacing = perimeter / n;
        int edge = 0;
        float edgeStart = 0f;
        float edgeLength = Vector2.Distance(P(0), P(1));
        for (int k = 0; k < n; k++)
        {
            float s = k * spacing;
            while (edgeStart + edgeLength < s && edge < count - 1)
            {
                edgeStart += edgeLength;
                edge++;
                edgeLength = Vector2.Distance(P(edge), P(edge + 1));
            }
            float t = edgeLength > 0f ? Mathf.Clamp01((s - edgeStart) / edgeLength) : 0f;
            result.Add(Vector2.Lerp(P(edge), P(edge + 1), t));
        }
    }

    // ---------- fill (pointed object only) ----------

    void SetHovered(Box box)
    {
        if (box == hoveredBox)
            return;
        if (hoveredBox != null)
            HideFill(hoveredBox);
        hoveredBox = box;
        if (hoveredBox != null)
            ShowFill(hoveredBox);
    }

    void ShowFill(Box box)
    {
        Material fillMaterial = box.color == boxColor ? registeredFillMaterial : unregisteredFillMaterial;
        for (int i = 0; i < box.pieceCount; i++)
        {
            Piece piece = box.pieces[i];
            Mesh mesh = piece.fill.sharedMesh;
            if (piece.triangles == null)
            {
                // Triangulate the target shape once; the morphing vertices reuse the same triangles.
                s_PolygonUV.Clear();
                foreach (var p in piece.target)
                    s_PolygonUV.Add(ToBoxUV(box.targetCorners, p));
                piece.triangles = Triangulate(s_PolygonUV);
                mesh.Clear();
                mesh.vertices = piece.current;
                mesh.triangles = piece.triangles;
            }
            var renderer = piece.fill.GetComponent<MeshRenderer>();
            if (renderer.sharedMaterial != fillMaterial)
                renderer.sharedMaterial = fillMaterial;
            piece.fill.gameObject.SetActive(true);
        }
        HideFillFrom(box, box.pieceCount);
    }

    // Keeps the hovered fill glued to the morphing outline.
    static void UpdateFillVertices(Box box)
    {
        for (int i = 0; i < box.pieceCount; i++)
        {
            Piece piece = box.pieces[i];
            if (piece.triangles == null)
                continue;
            Mesh mesh = piece.fill.sharedMesh;
            mesh.vertices = piece.current;
            mesh.RecalculateBounds();
        }
    }

    static void HideFill(Box box) => HideFillFrom(box, 0);

    static void HideFillFrom(Box box, int index)
    {
        for (int i = index; i < box.pieces.Count; i++)
            box.pieces[i].fill.gameObject.SetActive(false);
    }

    // ---------- mask geometry ----------

    static readonly List<List<Vector2>> s_Regions = new List<List<Vector2>>();
    static readonly Stack<List<Vector2>> s_RegionPool = new Stack<List<Vector2>>();

    static readonly List<List<Vector2>> s_Runs = new List<List<Vector2>>();

    // How Ultralytics builds the mask outline (ultralytics ops.masks2segments + converter.merge_multi_segment):
    //  - Each region outline comes from cv2.findContours(CHAIN_APPROX_SIMPLE), so every edge INSIDE a region is
    //    exactly horizontal, vertical or 45-degree diagonal (in mask pixels; uniform scaling keeps that).
    //  - Several regions are merged into one list by connecting the closest points of neighbouring regions with
    //    straight "bridges", walking out over them and later back. Bridges are generally NOT 0/45/90 degrees.
    // So: cut the list at every non-0/45/90 edge, then chain the pieces back together where one ends exactly where
    // another starts (middle regions arrive in two parts: one on the way out, the rest on the way back).
    // Each chain that returns to its start is one region. Output: s_Regions in viewport coordinates (bottom-left origin).
    // Limitation: a 0/45/90-degree bridge that leaves a region before it is closed is not detected and stays as a thin slit.
    static void SplitIntoRegions(Point2DMsg[] points, float w, float h)
    {
        RecycleLists(s_Regions);
        RecycleLists(s_Runs);

        int n = points.Length;

        // 1. Cut into runs. The merged list starts at the first region's start point, and merge_multi_segment
        //    repeats a region's start point when that region is closed, so a run also ends when it returns to its
        //    own start. This catches the bridges that happen to be exactly 0/45/90 degrees after a closed region.
        var run = RentList();
        for (int i = 0; i < n; i++)
        {
            var point = new Vector2((float)points[i].x, (float)points[i].y);
            run.Add(point);
            bool closes = run.Count >= 4 && SamePoint(point, run[0]);
            if (i == n - 1 || closes || IsBridge(points[i], points[i + 1]))
            {
                s_Runs.Add(run);
                run = RentList();
            }
        }
        s_RegionPool.Push(run); // always empty here

        // 2. Chain runs whose end point is another run's start point, until each chain closes on itself.
        while (s_Runs.Count > 0)
        {
            var chain = s_Runs[0];
            s_Runs.RemoveAt(0);

            while (!(chain.Count >= 4 && SamePoint(chain[0], chain[chain.Count - 1])))
            {
                int next = -1;
                for (int r = 0; r < s_Runs.Count; r++)
                {
                    if (SamePoint(chain[chain.Count - 1], s_Runs[r][0]))
                    {
                        next = r;
                        break;
                    }
                }
                if (next < 0)
                    break; // could not be closed (e.g. an undetected bridge); draw what we have

                var part = s_Runs[next];
                s_Runs.RemoveAt(next);
                for (int k = 1; k < part.Count; k++) // skip the shared joint point
                    chain.Add(part[k]);
                part.Clear();
                s_RegionPool.Push(part);
            }

            // 3. Drop the repeated closing point and convert pixels -> viewport.
            if (chain.Count > 1 && SamePoint(chain[0], chain[chain.Count - 1]))
                chain.RemoveAt(chain.Count - 1);
            if (chain.Count < 3)
            {
                chain.Clear();
                s_RegionPool.Push(chain);
                continue;
            }
            for (int k = 0; k < chain.Count; k++)
                chain[k] = new Vector2(chain[k].x / w, 1f - chain[k].y / h);
            s_Regions.Add(chain);
        }

        // Largest region first, so piece 0 (the box's own line/fill) is the main body.
        if (s_Regions.Count > 1)
            s_Regions.Sort((x, y) => y.Count.CompareTo(x.Count));
    }

    // True when the edge a -> b is not horizontal, vertical or 45-degree diagonal.
    static bool IsBridge(Point2DMsg a, Point2DMsg b)
    {
        double dx = System.Math.Abs(b.x - a.x);
        double dy = System.Math.Abs(b.y - a.y);
        double eps = 0.01 + 1e-3 * (dx + dy);
        return dx > eps && dy > eps && System.Math.Abs(dx - dy) > eps;
    }

    static bool SamePoint(Vector2 a, Vector2 b) => (a - b).sqrMagnitude < 0.25f; // within half a pixel

    static List<Vector2> RentList() => s_RegionPool.Count > 0 ? s_RegionPool.Pop() : new List<Vector2>();

    static void RecycleLists(List<List<Vector2>> lists)
    {
        foreach (var list in lists)
        {
            list.Clear();
            s_RegionPool.Push(list);
        }
        lists.Clear();
    }

    // Ear-clipping triangulation of a simple (possibly concave) polygon.
    // If the outline self-intersects and no ear can be found, the remaining part is left unfilled.
    static int[] Triangulate(List<Vector2> pts)
    {
        int n = pts.Count;
        float area = 0f;
        for (int i = 0; i < n; i++)
        {
            Vector2 a = pts[i], b = pts[(i + 1) % n];
            area += a.x * b.y - b.x * a.y;
        }

        // Work on a counter-clockwise index list.
        var idx = new List<int>(n);
        if (area > 0f)
            for (int i = 0; i < n; i++) idx.Add(i);
        else
            for (int i = n - 1; i >= 0; i--) idx.Add(i);

        var tris = new List<int>((n - 2) * 3);
        while (idx.Count > 3)
        {
            bool clipped = false;
            for (int i = 0; i < idx.Count; i++)
            {
                int prev = idx[(i - 1 + idx.Count) % idx.Count];
                int cur = idx[i];
                int next = idx[(i + 1) % idx.Count];
                if (!IsEar(pts, idx, prev, cur, next))
                    continue;
                tris.Add(prev);
                tris.Add(cur);
                tris.Add(next);
                idx.RemoveAt(i);
                clipped = true;
                break;
            }
            if (!clipped)
                break;
        }
        if (idx.Count == 3)
        {
            tris.Add(idx[0]);
            tris.Add(idx[1]);
            tris.Add(idx[2]);
        }
        return tris.ToArray();
    }

    static bool IsEar(List<Vector2> pts, List<int> idx, int prev, int cur, int next)
    {
        Vector2 a = pts[prev], b = pts[cur], c = pts[next];
        if (Cross(b - a, c - b) <= 0f)
            return false; // reflex (or degenerate) vertex
        foreach (int k in idx)
        {
            if (k == prev || k == cur || k == next)
                continue;
            Vector2 p = pts[k];
            if (Cross(b - a, p - a) >= 0f && Cross(c - b, p - b) >= 0f && Cross(a - c, p - c) >= 0f)
                return false; // another vertex lies inside this triangle
        }
        return true;
    }

    static float Cross(Vector2 u, Vector2 v) => u.x * v.y - u.y * v.x;


    // ---------- pooling ----------

    LineRenderer CreateLine(string name)
    {
        var line = new GameObject(name).AddComponent<LineRenderer>();
        line.useWorldSpace = true;
        line.loop = true;
        line.positionCount = 4;
        line.widthMultiplier = lineWidth;
        line.material = lineMaterial;
        line.startColor = outlineColor;
        line.endColor = outlineColor;
        return line;
    }

    MeshFilter CreateFill(string name)
    {
        // Vertices are written in world space, so the object stays at the origin.
        var fillObject = new GameObject(name);
        var fill = fillObject.AddComponent<MeshFilter>();
        fill.sharedMesh = new Mesh { name = name };
        fill.sharedMesh.MarkDynamic();
        var renderer = fillObject.AddComponent<MeshRenderer>();
        renderer.sharedMaterial = registeredFillMaterial;
        renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        renderer.receiveShadows = false;
        fillObject.SetActive(false);
        return fill;
    }

    Box GetOrCreateBox(string key, out bool isNew)
    {
        if (activeBoxes.TryGetValue(key, out Box box))
        {
            isNew = false;
            return box;
        }
        isNew = true;
        if (freeBoxes.Count > 0)
        {
            box = freeBoxes.Pop();
        }
        else
        {
            box = new Box();
            boxes.Add(box);
        }
        box.key = key;
        box.active = true;
        box.hasMask = false;
        box.hasDepth = false;
        box.pieceCount = 0;
        activeBoxes.Add(key, box);
        return box;
    }

    Piece GetPiece(Box box, int index)
    {
        while (box.pieces.Count <= index)
            box.pieces.Add(new Piece { line = CreateLine("YoloOutline"), fill = CreateFill("YoloFill") });
        return box.pieces[index];
    }

    static void HidePiecesFrom(Box box, int index)
    {
        for (int i = index; i < box.pieces.Count; i++)
        {
            box.pieces[i].line.gameObject.SetActive(false);
            box.pieces[i].fill.gameObject.SetActive(false);
        }
    }

    void Deactivate(Box box)
    {
        if (!box.active)
            return;
        box.active = false;
        box.pieceCount = 0;
        HidePiecesFrom(box, 0);
        activeBoxes.Remove(box.key);
        freeBoxes.Push(box);
        if (hoveredBox == box)
            hoveredBox = null;
    }

    void DeactivateAll()
    {
        if (activeBoxes.Count == 0)
            return;
        scratchBoxes.Clear();
        scratchBoxes.AddRange(activeBoxes.Values);
        foreach (var box in scratchBoxes)
            Deactivate(box);
    }
}
