using System.Collections.Generic;
using RosMessageTypes.Yolo;
using UnityEngine;
using Measurement = DepthMeasurer.Measurement;

// Keeps the list of objects (ObjectCube) up to date from the YOLO results.
//
// For every result (live mode: every message, scan mode: the one scan frame), Process does:
//   1. Project   every object's box is drawn into that camera image, using the camera pose of the moment the image
//                was taken. So head movement does not disturb the matching.
//   2. Match     a result belongs to an object of the same class when their boxes overlap in the image (IoU).
//   3. Update    matched, live -> measured again with depth rays, filtered, moved: the object follows the real one
//                                 and a wrong first depth is corrected. One far-off measurement is ignored (a ray
//                                 that hit the wall behind); two in a row are a real jump.
//                matched, scan -> measured again, refines the position
//                either way its mask is added to the object's silhouette
//   4. Miss      objects that should be in the image but were not found count a miss (unless something is in front)
//   5. Create    results that match nothing become new objects: tentative first, shown once seen ConfirmHits times
//   6. Remove    too many misses in a row, or not seen at all for UnseenTimeout
//
// Image border: an object cut off by the border gives a smaller box and mask than the whole object. So boxes are
// compared by their part inside the image, and a cut-off result only keeps its object alive: it creates nothing and
// changes neither position nor shape (unless it is so big that it could never fit, see EdgeBigArea).
// Duplicates: nothing is created where an existing object already overlaps the result almost completely (YOLO naming
// one thing twice or changing its mind about the class), or inside an object of the same class (YOLO found a part).
public class ObjectTracker
{
    // Matching
    const float MatchIoU = 0.15f;            // overlap needed for a result and an object to be the same thing
    const float StillIoU = 0.4f;             // overlap below this: the object is probably moving
    const float MaxPrediction = 0.3f;        // a moving object is predicted at most this far ahead (seconds)
    const float ViewMargin = 0.05f;          // 'in view' = center at least this far inside the image (0-1 of the image)
    // New objects
    const int ConfirmHits = 3;               // live: results in which a new object must be seen before it is shown
    const float TentativeTimeout = 1f;       // live: seconds a new object has to get confirmed
    const float EdgeMargin = 0.015f;         // a box this close to the image border (0-1 of the image) is cut off...
    const float EdgeBigArea = 0.25f;         // ...unless it covers this much of the image (too big to ever fit)
    const float DuplicateIoU = 0.8f;         // no new object where an existing one overlaps the result this much
    // Removing
    const float UnseenTimeout = 10f;         // live: seconds without any sighting, in view or not
    // Following (live)
    const float MaxHeadSpeed = 90f;          // no measurements from images taken while the head turned faster (deg/s)
    const float JumpDistance = 0.3f;         // a measurement farther than this from the expected position (meters)...
    const float JumpRatio = 0.25f;           // ...or than this share of its distance from the viewer, is suspicious
    const int JumpConfirmCount = 2;          // suspicious measurements in a row before the object jumps there
    const float SizeSmoothing = 0.3f;        // how much each measurement changes the box's size and facing (0-1)
    // Silhouette shape
    const float MovingShapeTime = 0.08f;     // seconds the shape takes to follow while the object seems to move
    const float FastShapeSpeed = 0.5f;       // at this speed (m/s) the newest mask is shown as it is

    struct Projection { public bool valid; public bool inView; public Rect rect; }
    struct Pair { public int det; public int cube; public float iou; }

    // Counters for the stats log (reset by the reader).
    public int cutOffResults;
    public int duplicatesSkipped;

    readonly YoloBoxVisualizer settings;     // the Inspector values
    readonly DepthMeasurer depth;
    readonly Material lineMaterial;
    readonly List<ObjectCube> cubes = new List<ObjectCube>();
    readonly List<ObjectCube> dying = new List<ObjectCube>();   // removed, fading out
    int nextSerial = 1;
    bool warnedNoMask;
    Vector3 viewer;                          // camera position of the result being processed (for the log)

    // Scratch, reused for every result
    readonly List<Projection> projections = new List<Projection>();
    readonly List<Pair> pairs = new List<Pair>();
    readonly List<bool> detUsed = new List<bool>();
    readonly List<bool> cubeUsed = new List<bool>();
    readonly Vector3[] corners = new Vector3[8];
    readonly List<Vector2> boxRegion = new List<Vector2>();
    readonly List<List<Vector2>> boxRegions = new List<List<Vector2>>();
    static readonly System.Comparison<Pair> s_BestFirst = (a, b) => b.iou.CompareTo(a.iou);

    public ObjectTracker(YoloBoxVisualizer settings, DepthMeasurer depth, Material lineMaterial)
    {
        this.settings = settings;
        this.depth = depth;
        this.lineMaterial = lineMaterial;
        boxRegions.Add(boxRegion);
    }

    public int Count => cubes.Count;

    public int VisibleCount
    {
        get
        {
            int n = 0;
            foreach (var cube in cubes)
            {
                if (cube.Visible)
                    n++;
            }
            return n;
        }
    }

    // ===================== One YOLO result =====================

    // frame: the camera image the result belongs to. headSpeed: how fast the head turned when it was taken (deg/s).
    public void Process(DetectionMsg[] detections, ScanFrame frame, bool scan, float headSpeed, float now)
    {
        bool canMeasure = scan || headSpeed <= MaxHeadSpeed;
        viewer = frame.CameraPosition;

        // 1. Where every object appears in this image.
        projections.Clear();
        cubeUsed.Clear();
        foreach (var cube in cubes)
        {
            projections.Add(Project(cube, frame, now));
            cubeUsed.Add(false);
        }
        int oldCount = cubes.Count;

        // 2. Candidate pairs (same class, overlapping), best first; then one result per object.
        pairs.Clear();
        detUsed.Clear();
        for (int i = 0; i < detections.Length; i++)
        {
            DetectionMsg det = detections[i];
            bool usable = det.score >= settings.keepScore;
            detUsed.Add(!usable); // too unsure to use at all
            if (!usable)
                continue;
            Rect rect = frame.BoxRect(det);
            for (int j = 0; j < oldCount; j++)
            {
                if (cubes[j].className != det.class_name || !projections[j].valid)
                    continue;
                float iou = IoU(projections[j].rect, rect);
                if (iou >= MatchIoU)
                    pairs.Add(new Pair { det = i, cube = j, iou = iou });
            }
        }
        pairs.Sort(s_BestFirst);

        // 3. Update the matched objects.
        foreach (var pair in pairs)
        {
            if (detUsed[pair.det] || cubeUsed[pair.cube])
                continue;
            detUsed[pair.det] = true;
            cubeUsed[pair.cube] = true;
            UpdateMatched(cubes[pair.cube], detections[pair.det], pair.iou, frame, scan, canMeasure, now);
        }

        // 4. Objects that should have been seen in this image but were not.
        for (int j = 0; j < oldCount; j++)
        {
            if (!cubeUsed[j] && projections[j].inView && !depth.IsOccluded(cubes[j], frame))
                cubes[j].misses++;
        }

        // 5. Results that match nothing: new objects.
        for (int i = 0; i < detections.Length; i++)
        {
            if (detUsed[i] || detections[i].score < settings.createScore)
                continue;
            Rect rect = frame.BoxRect(detections[i]);
            if (!scan && IsCutOff(rect))
                cutOffResults++;      // wait until it is fully in the image (a scan has only this one result)
            else if (OverlapsExisting(rect, oldCount))
                duplicatesSkipped++;
            else
                Create(detections[i], frame, scan, canMeasure, now);
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
        cube.misses = 0;
        cube.lastSeen = now;

        // Cut off by the image border: only part of the object is in this result. It shows the object is still
        // there, but its box and mask are too small to measure or draw from.
        if (IsCutOff(frame.BoxRect(det)))
        {
            cutOffResults++;
            return;
        }
        cube.detection = det;
        cube.hits++;

        if (cube.state == ObjectCube.State.Tentative)
        {
            // Not shown yet: every sighting is measured and averaged for a good first placement.
            if (canMeasure && depth.Measure(det, frame, DepthMeasurer.LiveRays, out var sample) && UsableDepth(sample, false))
                cube.AddSample(sample.CenterAt(cube.size.z), new Vector3(sample.width, sample.height, cube.size.z));
            if (cube.hits >= ConfirmHits)
                Confirm(cube, det, frame, now);
            return;
        }

        if (scan)
        {
            // A new scan of a known object refines its position. Its size and facing stay as first measured.
            if (depth.Measure(det, frame, DepthMeasurer.ScanRays, out var refined) && refined.hit)
                cube.ApplyMeasurement(refined.CenterAt(cube.size.z), now, settings.deadband);
            AddToSilhouette(cube, det, frame, 0.5f, now);
            return;
        }

        // Live: measured again on every result, so the object follows the real one.
        if (canMeasure && depth.Measure(det, frame, DepthMeasurer.LiveRays, out var measured) && measured.hit)
            Follow(cube, measured, now);

        // Shape: averaged strongly while resting; the newest mask counts more the faster the object moves.
        float speed01 = Mathf.InverseLerp(0f, FastShapeSpeed, cube.velocity.magnitude);
        float shapeTime = Mathf.Lerp(settings.shapeTimeConstant, 0f, speed01);
        if (iou < StillIoU)
            shapeTime = Mathf.Min(shapeTime, MovingShapeTime); // off from where it was: probably moving
        AddToSilhouette(cube, det, frame, ShapeWeight(cube, shapeTime, now), now);
    }

    // Live: one new measurement of a shown object.
    void Follow(ObjectCube cube, Measurement m, float now)
    {
        cube.UpdateShape(m.Size, m.Facing, SizeSmoothing);
        Vector3 measuredCenter = m.CenterAt(cube.size.z);

        // A measurement far from where the object should be is usually a ray that hit the wall behind it.
        // Ignore it once; if the next one agrees, the object really is there: jump.
        Vector3 expected = cube.PredictedCenter(now, MaxPrediction);
        float limit = Mathf.Max(JumpDistance, JumpRatio * Vector3.Distance(viewer, expected));
        if ((measuredCenter - expected).magnitude <= limit)
        {
            cube.jumpStreak = 0;
            cube.ApplyMeasurement(measuredCenter, now, settings.deadband);
        }
        else if (++cube.jumpStreak >= JumpConfirmCount)
        {
            cube.jumpStreak = 0;
            cube.ResetPosition(measuredCenter, now);
            Log($"~ {Describe(cube)} jumped");
        }
    }

    // Weight of a new mask so the shape follows changes within about `timeConstant` seconds whatever the result
    // rate: with results every dt seconds, weight = 1 - e^(-dt / timeConstant).
    static float ShapeWeight(ObjectCube cube, float timeConstant, float now)
    {
        if (timeConstant <= 0.001f)
            return 1f;
        float dt = Mathf.Clamp(now - cube.silhouette.LastUpdateTime, 0.001f, 1f);
        return 1f - Mathf.Exp(-dt / timeConstant);
    }

    // Adds the result's mask to the object's silhouette (the YOLO box when it has no mask).
    void AddToSilhouette(ObjectCube cube, DetectionMsg det, ScanFrame frame, float weight, float now)
    {
        var regions = frame.MaskRegions(det);
        if (regions.Count == 0)
        {
            if (!warnedNoMask)
            {
                warnedNoMask = true;
                Debug.LogWarning("[steelarm] YOLO results carry no masks; silhouettes use the boxes. Check that a -seg model is running.");
            }
            Rect box = frame.BoxRect(det);
            boxRegion.Clear();
            boxRegion.Add(new Vector2(box.xMin, box.yMin));
            boxRegion.Add(new Vector2(box.xMax, box.yMin));
            boxRegion.Add(new Vector2(box.xMax, box.yMax));
            boxRegion.Add(new Vector2(box.xMin, box.yMax));
            regions = boxRegions;
        }
        cube.silhouette.AddMask(regions, frame, cube.FrontCenter, weight, settings.outlineBlur, now);
    }

    // ===================== Creating and removing =====================

    void Create(DetectionMsg det, ScanFrame frame, bool scan, bool canMeasure, float now)
    {
        if (!canMeasure)
            return; // head turning fast: wait for a sharper image
        int rays = scan ? DepthMeasurer.ScanRays : DepthMeasurer.LiveRays;
        if (!depth.Measure(det, frame, rays, out var m) || !UsableDepth(m, scan))
            return;
        Vector3 center = m.CenterAt(m.boxDepth);
        if (IsInsideSameClass(det.class_name, center))
        {
            duplicatesSkipped++; // YOLO found a part of an object that is already there
            return;
        }
        if (!m.hit)
            Debug.LogWarning($"[steelarm] No depth for {det.class_name}; placed at {DepthMeasurer.FallbackDistance} m.");

        var cube = new ObjectCube(nextSerial++, det.class_name, lineMaterial)
        {
            detection = det,
            registered = RobotActionRegistry.GetActions(det.class_name) != null,
            predictWhenDrawn = !scan,
            rotation = m.Facing,
            createdTime = now,
            lastSeen = now,
            hits = 1
        };
        cube.AddSample(center, m.Size);
        cubes.Add(cube);
        if (scan)
            Confirm(cube, det, frame, now); // a scan has only this one result
    }

    // Live: an object placed without real depth (depth not ready yet, rays missed) would sit at the fallback distance
    // and, seen from the same spot, still overlap perfectly, so nothing would correct it. Better to wait for real depth.
    // A scan has only its one result, so the fallback is used there (with a warning).
    bool UsableDepth(Measurement m, bool scan) => m.hit || scan || !depth.HasDepth;

    // The object becomes visible; its silhouette starts as this result's mask.
    void Confirm(ObjectCube cube, DetectionMsg det, ScanFrame frame, float now)
    {
        cube.Confirm(now, settings.minCutoff);
        AddToSilhouette(cube, det, frame, 1f, now);
        Log($"+ {Describe(cube)}, size {cube.size.x:0.00} x {cube.size.y:0.00} x {cube.size.z:0.00} m");
    }

    bool ShouldRemove(ObjectCube cube, bool scan, float now)
    {
        if (cube.state == ObjectCube.State.Tentative)
            return now - cube.createdTime > TentativeTimeout || cube.misses >= 2;
        if (scan)
            return cube.misses >= 1;
        return cube.misses >= settings.lostMisses || now - cube.lastSeen > UnseenTimeout;
    }

    void RemoveAt(int index, string reason)
    {
        ObjectCube cube = cubes[index];
        cubes.RemoveAt(index);
        if (!cube.Visible)
        {
            cube.Destroy();
            return;
        }
        Log($"- {Describe(cube)} ({reason})");
        cube.StartDying(); // fades out; destroyed in Animate
        dying.Add(cube);
    }

    public void RemoveAll()
    {
        for (int j = cubes.Count - 1; j >= 0; j--)
            RemoveAt(j, "cleared");
    }

    // ===================== Geometry =====================

    // The object's box (at its predicted position) drawn into this camera image, as a viewport rectangle.
    Projection Project(ObjectCube cube, ScanFrame frame, float now)
    {
        var p = new Projection();
        Vector3 center = cube.PredictedCenter(now, MaxPrediction);
        cube.GetCorners(center, corners);
        float xMin = float.MaxValue, yMin = float.MaxValue, xMax = float.MinValue, yMax = float.MinValue;
        foreach (var corner in corners)
        {
            if (!frame.WorldToViewport(corner, out Vector2 v))
                return p; // partly behind the camera: not comparable, and not 'in view'
            xMin = Mathf.Min(xMin, v.x); xMax = Mathf.Max(xMax, v.x);
            yMin = Mathf.Min(yMin, v.y); yMax = Mathf.Max(yMax, v.y);
        }
        p.valid = true;
        // Only the part inside the image: that is all YOLO can see of it (zero size when it is completely outside).
        p.rect = Rect.MinMaxRect(Mathf.Clamp01(xMin), Mathf.Clamp01(yMin), Mathf.Clamp01(xMax), Mathf.Clamp01(yMax));
        // In view = its center is clearly inside the image (an object at the border may be cut off and missed by YOLO).
        p.inView = frame.WorldToViewport(center, out Vector2 c)
                   && c.x > ViewMargin && c.x < 1f - ViewMargin && c.y > ViewMargin && c.y < 1f - ViewMargin;
        return p;
    }

    // The box touches the image border, so the object continues outside the image (and is not simply too big to fit).
    static bool IsCutOff(Rect box)
    {
        bool touches = box.xMin < EdgeMargin || box.yMin < EdgeMargin || box.xMax > 1f - EdgeMargin || box.yMax > 1f - EdgeMargin;
        return touches && box.width * box.height < EdgeBigArea;
    }

    // One of the first `count` objects (any class), as projected for this result, already covers this box almost exactly.
    bool OverlapsExisting(Rect box, int count)
    {
        for (int j = 0; j < count; j++)
        {
            if (projections[j].valid && IoU(projections[j].rect, box) >= DuplicateIoU)
                return true;
        }
        return false;
    }

    // The point lies inside the (slightly enlarged) box of an existing object of this class.
    bool IsInsideSameClass(string className, Vector3 point)
    {
        foreach (var cube in cubes)
        {
            if (cube.className != className)
                continue;
            Vector3 local = Quaternion.Inverse(cube.rotation) * (point - cube.center);
            Vector3 half = cube.size * 0.6f + new Vector3(0.03f, 0.03f, 0.03f);
            if (Mathf.Abs(local.x) <= half.x && Mathf.Abs(local.y) <= half.y && Mathf.Abs(local.z) <= half.z)
                return true;
        }
        return false;
    }

    // Intersection over union of two rectangles: 0 = apart, 1 = identical.
    static float IoU(Rect a, Rect b)
    {
        float w = Mathf.Min(a.xMax, b.xMax) - Mathf.Max(a.xMin, b.xMin);
        float h = Mathf.Min(a.yMax, b.yMax) - Mathf.Max(a.yMin, b.yMin);
        if (w <= 0f || h <= 0f)
            return 0f;
        float intersection = w * h;
        float union = a.width * a.height + b.width * b.height - intersection;
        return union > 1e-9f ? intersection / union : 0f;
    }

    // ===================== Every frame =====================

    // Draws all objects. head: the viewer's position.
    public void Animate(float now, float deltaTime, Vector3 head, Color outlineColor, float lineWidth, float fadeDuration)
    {
        foreach (var cube in cubes)
            cube.Animate(now, deltaTime, head, outlineColor, lineWidth, fadeDuration);
        for (int i = dying.Count - 1; i >= 0; i--)
        {
            dying[i].Animate(now, deltaTime, head, outlineColor, lineWidth, fadeDuration);
            if (dying[i].FadedOut)
            {
                dying[i].Destroy();
                dying.RemoveAt(i);
            }
        }
    }

    // The nearest object whose silhouette the ray hits, or null.
    public ObjectCube FindPointed(Ray ray, out float nearest)
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

    // ===================== Log =====================

    string Describe(ObjectCube cube) => $"{cube.className} #{cube.serial} at {Vector3.Distance(viewer, cube.center):0.00} m";

    void Log(string message)
    {
        if (settings.debugLog)
            Debug.Log("[steelarm] " + message);
    }
}
