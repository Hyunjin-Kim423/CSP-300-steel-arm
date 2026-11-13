using System.Collections.Generic;
using RosMessageTypes.Yolo;
using UnityEngine;
using EnvironmentRaycastManager = Meta.XR.EnvironmentRaycastManager;

// Finds where a YOLO result is in the world, using the headset's depth (Meta environment raycasts).
//
// A few rays are sent through points inside the result's box (inside its mask when there is one, so a chair's legs
// don't send rays to the wall behind). Their median depth is the object's front face. Width and height come from the
// YOLO box at that depth. The front-to-back depth cannot be seen from one side and is guessed from the width.
public class DepthMeasurer
{
    public struct Measurement
    {
        public Vector3 front;      // center of the front face (world)
        public Vector3 forward;    // horizontal direction from the camera towards the object
        public float width;
        public float height;
        public float boxDepth;     // front to back (guessed)
        public bool hit;           // false: no ray hit anything, FallbackDistance was used

        public Vector3 Size => new Vector3(width, height, boxDepth);
        public Quaternion Facing => Quaternion.LookRotation(forward, Vector3.up);
        // Center of a box with this front face that is `depth` meters deep.
        public Vector3 CenterAt(float depth) => front + forward * (depth * 0.5f);
    }

    public const int LiveRays = 9;              // rays per measurement in live mode (every result)
    public const int ScanRays = 25;             // in scan mode (measured once, so more are affordable)
    public const float FallbackDistance = 1f;   // depth used when no ray hits (meters)
    const float DepthPerWidth = 1f;             // guessed front-to-back depth, as a fraction of the width
    const float MinBoxDepth = 0.05f;
    const float MaxBoxDepth = 1f;

    // Counters for the stats log (reset by the reader).
    public int rays;
    public int measurements;

    readonly EnvironmentRaycastManager raycaster;
    readonly List<Vector2> candidates = new List<Vector2>();
    readonly List<Vector2> samples = new List<Vector2>();
    readonly List<float> depths = new List<float>();

    public bool HasDepth => raycaster != null;

    // host: the GameObject the raycast manager is added to if the scene has none.
    public DepthMeasurer(GameObject host)
    {
        if (!EnvironmentRaycastManager.IsSupported)
        {
            Debug.LogWarning($"[steelarm] Environment depth is not supported; objects are placed at {FallbackDistance} m.");
            return;
        }
        OVRPermissionsRequester.Request(new[] { OVRPermissionsRequester.Permission.Scene });
        raycaster = UnityEngine.Object.FindAnyObjectByType<EnvironmentRaycastManager>();
        if (raycaster == null)
            raycaster = host.AddComponent<EnvironmentRaycastManager>();
    }

    // Measures one result: front face center, width and height (gravity-aligned), facing the camera horizontally.
    public bool Measure(DetectionMsg det, ScanFrame frame, int rayCount, out Measurement m)
    {
        m = new Measurement();
        Rect box = frame.BoxRect(det);
        if (box.width <= 0f || box.height <= 0f)
            return false;
        measurements++;

        // Depth: median of the rays that hit something.
        BuildSamplePoints(det, frame, box, Mathf.Max(1, rayCount));
        depths.Clear();
        if (raycaster != null)
        {
            foreach (var point in samples)
            {
                Ray ray = frame.ViewportRay(point);
                rays++;
                if (raycaster.Raycast(ray, out var hit))
                {
                    float d = Vector3.Dot(hit.point - ray.origin, frame.forward);
                    if (d > 0.05f)
                        depths.Add(d);
                }
            }
        }
        m.hit = depths.Count > 0;
        float front = m.hit ? Median(depths) : FallbackDistance;

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
            var corner = new Vector2(k == 0 || k == 1 ? box.xMin : box.xMax, k == 0 || k == 3 ? box.yMin : box.yMax);
            Vector3 p = frame.ViewportToWorld(corner, front);
            float pr = Vector3.Dot(p, right);
            rMin = Mathf.Min(rMin, pr); rMax = Mathf.Max(rMax, pr);
            yMin = Mathf.Min(yMin, p.y); yMax = Mathf.Max(yMax, p.y);
            fSum += Vector3.Dot(p, forward);
        }

        m.forward = forward;
        m.width = Mathf.Max(0.03f, rMax - rMin);
        m.height = Mathf.Max(0.03f, yMax - yMin);
        m.boxDepth = Mathf.Clamp(m.width * DepthPerWidth, MinBoxDepth, MaxBoxDepth);
        m.front = right * ((rMin + rMax) * 0.5f) + forward * (fSum * 0.25f) + Vector3.up * ((yMin + yMax) * 0.5f);
        return true;
    }

    // True when something (a hand, a person) is clearly in front of this object in this image.
    public bool IsOccluded(ObjectCube cube, ScanFrame frame)
    {
        if (raycaster == null || !frame.WorldToViewport(cube.center, out Vector2 viewport))
            return false;
        Ray ray = frame.ViewportRay(viewport);
        rays++;
        if (!raycaster.Raycast(ray, out var hit))
            return false;
        float hitDepth = Vector3.Dot(hit.point - ray.origin, frame.forward);
        float cubeDepth = Vector3.Dot(cube.center - ray.origin, frame.forward);
        return hitDepth < cubeDepth - Mathf.Max(0.15f, cube.size.z);
    }

    // Viewport points to aim the rays at: a grid over the middle of the YOLO box, kept inside the mask when there is one.
    void BuildSamplePoints(DetectionMsg det, ScanFrame frame, Rect box, int count)
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
                float v = grid == 1 ? 0.5f : Mathf.Lerp(inset, 1f - inset, gy / (float)(grid - 1));
                candidates.Add(new Vector2(Mathf.Lerp(box.xMin, box.xMax, u), Mathf.Lerp(box.yMin, box.yMax, v)));
            }
        }

        var regions = frame.MaskRegions(det);
        foreach (var candidate in candidates)
        {
            foreach (var region in regions)
            {
                if (Polygon2D.Contains(region, candidate))
                {
                    samples.Add(candidate);
                    break;
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
}
