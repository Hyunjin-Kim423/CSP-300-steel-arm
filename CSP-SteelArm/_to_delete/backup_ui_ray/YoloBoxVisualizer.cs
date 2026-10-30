using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.InputSystem;
using Unity.Robotics.ROSTCPConnector;
using Unity.XR.CoreUtils;
using RosMessageTypes.Yolo;
using EnvironmentRaycastManager = Meta.XR.EnvironmentRaycastManager;
using PassthroughCameraAccess = Meta.XR.PassthroughCameraAccess;

// Draws every YOLO detection as a thin white outline (segmentation silhouette, or the bounding box when there is no mask).
// Only the object the controller is pointing at gets its silhouette filled, colored by whether it is listed in
// yolo_dectection_list.json. Triangulating the fill is the expensive part, so it is done for that one object only.
[RequireComponent(typeof(CameraImagePublisher))]
public class YoloBoxVisualizer : MonoBehaviour
{
    // One closed outline region (+ its fill). A person partly hidden behind a chair can have several.
    class Piece
    {
        public LineRenderer line;
        public MeshFilter fill;
        public readonly List<Vector2> outline2D = new List<Vector2>(); // viewport coordinates (bottom-left origin)
        public Vector3[] world = new Vector3[0];                        // same points in world space
        public bool fillValid;                                           // fill mesh matches the current outline
    }

    class Box
    {
        public bool active;
        public bool hasMask;
        public readonly List<Piece> pieces = new List<Piece>();
        public int pieceCount;
        public Color color;                     // fill/label color (registered or not)
        public readonly Vector3[] corners = new Vector3[4]; // 0 left-bottom, 1 left-top, 2 right-top, 3 right-bottom
        public float left, right, top, bottom;  // bounding box in viewport coordinates
        public string text;
        public float depth;
        public Vector3 cameraPosition;
        public DetectionMsg detectionMsg;
    }

    class Controller
    {
        public readonly InputAction trigger;
        public readonly InputAction position;
        public readonly InputAction rotation;

        public Controller(string hand)
        {
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

    public string topicName = "/yolo/detections";
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
    [Tooltip("Mask outlines are down-sampled to at most this many points (total, across all regions).")]
    public int maxMaskPoints = 80;

    [Header("Label")]
    [Tooltip("Label text height in meters for an object 1 m away; scales with distance.")]
    public float labelHeight = 0.03f;
    [Tooltip("How long the label stays after pulling the trigger (seconds).")]
    public float labelDuration = 3f;
    [Tooltip("Boxes are hidden if no new detections arrive within this many seconds.")]
    public float boxLifetime = 2f;

    CameraImagePublisher publisher;
    EnvironmentRaycastManager environmentRaycast;
    XROrigin xrOrigin;
    Controller[] controllers;
    Material lineMaterial;
    Material registeredFillMaterial;
    Material unregisteredFillMaterial;
    TextMeshPro label;
    readonly List<Box> boxes = new List<Box>();
    Box hoveredBox;
    float lastDetectionTime;
    float labelHideTime;

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
        controllers = new[] { new Controller("RightHand"), new Controller("LeftHand") };
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
        ROSConnection.GetOrCreateInstance().Subscribe<DetectionArrayMsg>(topicName, OnDetections);
    }

    void OnDestroy()
    {
        foreach (var controller in controllers)
            controller.Dispose();
    }

    void Update()
    {
        if (Time.time - lastDetectionTime > boxLifetime)
            HideBoxesFrom(0);
        if (Time.time > labelHideTime)
            label.gameObject.SetActive(false);

        // Hover: fill the object either controller points at (nearest hit wins).
        Box pointed = null;
        float nearest = float.MaxValue;
        foreach (var controller in controllers)
        {
            Box hit = FindPointedBox(AimRay(controller), out float distance);
            if (hit != null && distance < nearest)
            {
                pointed = hit;
                nearest = distance;
            }
        }
        SetHovered(pointed);

        foreach (var controller in controllers)
        {
            if (controller.trigger.WasPressedThisFrame())
                SelectPointedBox(AimRay(controller));
        }
    }

    Ray AimRay(Controller controller)
    {
        // Controller poses are reported in tracking space, relative to the XR Origin's camera offset.
        Transform trackingSpace = xrOrigin.CameraFloorOffsetObject.transform;
        Vector3 position = trackingSpace.TransformPoint(controller.position.ReadValue<Vector3>());
        Quaternion rotation = trackingSpace.rotation * controller.rotation.ReadValue<Quaternion>();
        return new Ray(position, rotation * Vector3.forward);
    }

    // The nearest object whose silhouette (or box, when it has no mask) contains the ray's hit point.
    // Testing the silhouette instead of the box keeps touching/overlapping objects apart.
    Box FindPointedBox(Ray ray, out float nearest)
    {
        Box pointed = null;
        nearest = float.MaxValue;
        foreach (var box in boxes)
        {
            if (!box.active || !RayHitsBox(ray, box.corners, out float distance, out Vector2 boxUV) || distance >= nearest)
                continue;

            if (box.hasMask)
            {
                // The silhouette lies on the box plane, and viewport -> plane is linear, so map the hit to viewport.
                var viewport = new Vector2(Mathf.Lerp(box.left, box.right, boxUV.x), Mathf.Lerp(box.bottom, box.top, boxUV.y));
                if (!SilhouetteContains(box, viewport))
                    continue;
            }

            pointed = box;
            nearest = distance;
        }
        return pointed;
    }

    static bool SilhouetteContains(Box box, Vector2 p)
    {
        for (int i = 0; i < box.pieceCount; i++)
        {
            if (PointInPolygon(box.pieces[i].outline2D, p))
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
        Vector3 hit = ray.GetPoint(distance) - corners[0];
        Vector3 up = corners[1] - corners[0];
        Vector3 right = corners[3] - corners[0];
        uv = new Vector2(Vector3.Dot(hit, right) / right.sqrMagnitude, Vector3.Dot(hit, up) / up.sqrMagnitude);
        return uv.x >= 0f && uv.x <= 1f && uv.y >= 0f && uv.y <= 1f;
    }

    void OnDetections(DetectionArrayMsg msg)
    {
        // Debug.Log("[steelarm] On Yolo detection");
        var cameraAccess = publisher.CameraAccess;
        if (!cameraAccess.IsPlaying) return;
        if (!publisher.TryGetFrame(msg.header.stamp, out var frame)) return;

        Pose pose = frame.cameraPose;
        Vector3 forward = pose.rotation * Vector3.forward;
        float w = frame.imageSize.x;
        float h = frame.imageSize.y;
        int count = 0;
        foreach (var det in msg.detections)
        {
            var center = det.bbox.center.position;
            var size = det.bbox.size;
            // YOLO pixels have a top-left origin; the camera viewport has a bottom-left origin.
            float left = (float)(center.x - size.x / 2) / w;
            float right = (float)(center.x + size.x / 2) / w;
            float top = 1f - (float)(center.y - size.y / 2) / h;
            float bottom = 1f - (float)(center.y + size.y / 2) / h;

            Ray centerRay = cameraAccess.ViewportPointToRay(new Vector2((left + right) / 2, (top + bottom) / 2), pose);
            float depth = DepthAlong(centerRay, forward);

            var box = GetBox(count++);
            box.left = left;
            box.right = right;
            box.top = top;
            box.bottom = bottom;
            box.corners[0] = PointAtDepth(cameraAccess, pose, forward, left, bottom, depth);
            box.corners[1] = PointAtDepth(cameraAccess, pose, forward, left, top, depth);
            box.corners[2] = PointAtDepth(cameraAccess, pose, forward, right, top, depth);
            box.corners[3] = PointAtDepth(cameraAccess, pose, forward, right, bottom, depth);
            // Fill/label color: green if the object has robot actions in yolo_dectection_list.json, red otherwise.
            box.color = RobotActionRegistry.GetActions(det.class_name) != null ? boxColor : unregisteredColor;

            box.hasMask = highlightStyle == HighlightStyle.Silhouette
                && BuildSilhouetteOutline(box, det.mask, cameraAccess, pose, forward, depth, w, h);
            if (!box.hasMask)
                BuildBoxOutline(box);

            box.text = $"{det.class_name} {det.score:0.00}";
            box.depth = depth;
            box.cameraPosition = pose.position;
            box.detectionMsg = det;
        }

        HideBoxesFrom(count);
        lastDetectionTime = Time.time;

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

    // ---------- outlines (every object) ----------

    void BuildBoxOutline(Box box)
    {
        Piece piece = GetPiece(box, 0);
        piece.outline2D.Clear();
        piece.outline2D.Add(new Vector2(box.left, box.bottom));
        piece.outline2D.Add(new Vector2(box.left, box.top));
        piece.outline2D.Add(new Vector2(box.right, box.top));
        piece.outline2D.Add(new Vector2(box.right, box.bottom));
        SetPieceWorld(piece, (Vector3[])box.corners.Clone());
        box.pieceCount = 1;
        HidePiecesFrom(box, 1);
    }

    // Splits the mask into its regions and draws one closed outline per region. Returns false when there is no usable mask.
    bool BuildSilhouetteOutline(Box box, MaskMsg mask, PassthroughCameraAccess cameraAccess, Pose pose, Vector3 forward, float depth, float w, float h)
    {
        var points = mask?.data;
        if (points == null || points.Length < 3)
            return false;

        SplitIntoRegions(points, w, h);
        if (s_Regions.Count == 0)
            return false;

        // Shared point budget: one down-sampling step for all regions, but never below 3 points per region.
        int step = Mathf.Max(1, Mathf.CeilToInt(points.Length / (float)Mathf.Max(3, maxMaskPoints)));
        for (int r = 0; r < s_Regions.Count; r++)
        {
            var region = s_Regions[r];
            int regionStep = region.Count / step >= 3 ? step : 1;

            Piece piece = GetPiece(box, r);
            piece.outline2D.Clear();
            for (int i = 0; i < region.Count; i += regionStep)
                piece.outline2D.Add(region[i]);

            var world = new Vector3[piece.outline2D.Count];
            for (int i = 0; i < world.Length; i++)
                world[i] = PointAtDepth(cameraAccess, pose, forward, piece.outline2D[i].x, piece.outline2D[i].y, depth);
            SetPieceWorld(piece, world);
        }
        box.pieceCount = s_Regions.Count;
        HidePiecesFrom(box, s_Regions.Count);
        return true;
    }

    void SetPieceWorld(Piece piece, Vector3[] world)
    {
        piece.world = world;
        piece.fillValid = false;
        piece.line.gameObject.SetActive(true);
        piece.line.positionCount = world.Length;
        piece.line.SetPositions(world);
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
            if (!piece.fillValid)
            {
                Mesh mesh = piece.fill.sharedMesh;
                mesh.Clear();
                mesh.vertices = piece.world;
                mesh.triangles = Triangulate(piece.outline2D);
                mesh.RecalculateBounds();
                piece.fillValid = true;
            }
            var renderer = piece.fill.GetComponent<MeshRenderer>();
            if (renderer.sharedMaterial != fillMaterial)
                renderer.sharedMaterial = fillMaterial;
            piece.fill.gameObject.SetActive(true);
        }
    }

    static void HideFill(Box box)
    {
        foreach (var piece in box.pieces)
            piece.fill.gameObject.SetActive(false);
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

    Box GetBox(int index)
    {
        if (index == boxes.Count)
            boxes.Add(new Box());
        boxes[index].active = true;
        return boxes[index];
    }

    Piece GetPiece(Box box, int index)
    {
        if (index == box.pieces.Count)
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

    void HideBoxesFrom(int index)
    {
        for (int i = index; i < boxes.Count; i++)
        {
            boxes[i].active = false;
            boxes[i].pieceCount = 0;
            HidePiecesFrom(boxes[i], 0);
        }
        if (hoveredBox != null && !hoveredBox.active)
            hoveredBox = null;
    }
}
