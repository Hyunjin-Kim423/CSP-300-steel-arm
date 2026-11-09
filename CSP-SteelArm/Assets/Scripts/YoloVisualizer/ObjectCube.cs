using RosMessageTypes.Yolo;
using UnityEngine;

// One real object found by YOLO, kept as a gravity-aligned box in the world.
//
// The box is placed in world space and stays there: it does NOT follow the head. Turning the head or walking around
// only changes the view; the box is moved only when a new measurement says the real object moved.
// What is drawn eases towards the box, so a move looks smooth instead of jumping. It is either
//   - the silhouette (default): ObjectSilhouette, anchored at the box's front face, shape averaged from the masks
//   - the wireframe box itself (drawBox)
//
//   Tentative   seen, not drawn yet. Its measurements are averaged until YOLO has seen it often enough.
//   Confirmed   drawn, and matched against every new YOLO result (see YoloBoxVisualizer).
public class ObjectCube
{
    public enum State { Tentative, Confirmed }

    // Unit box centered at the origin: bottom face 0-3, top face 4-7. Local +z points away from the camera that
    // first measured the object (front face at -z).
    static readonly Vector3[] k_UnitCorners =
    {
        new Vector3(-0.5f, -0.5f, -0.5f), new Vector3(0.5f, -0.5f, -0.5f), new Vector3(0.5f, -0.5f, 0.5f), new Vector3(-0.5f, -0.5f, 0.5f),
        new Vector3(-0.5f,  0.5f, -0.5f), new Vector3(0.5f,  0.5f, -0.5f), new Vector3(0.5f,  0.5f, 0.5f), new Vector3(-0.5f,  0.5f, 0.5f),
    };

    public readonly int serial;
    public readonly string className;
    public State state = State.Tentative;
    public string trackId = "";      // last yolo_ros track id ("" when tracking is not used)
    public DetectionMsg detection;   // latest matched YOLO result (class, score)
    public bool registered;          // listed in yolo_dectection_list.json (has robot actions)
    public bool drawBox;             // draw the wireframe box instead of the silhouette
    public readonly ObjectSilhouette silhouette;

    // The box: where the object is believed to be (world space).
    public Vector3 center;
    public Vector3 size;             // x: width, y: height, z: depth (front to back)
    public Quaternion rotation;      // yaw only
    public Vector3 velocity;         // m/s, used to predict where it is when matching
    public float lastMeasureTime;

    // Matching bookkeeping
    public int hits;                 // results matched to this object
    public int misses;               // results in a row in which it was in view but not found
    public int moveStreak;           // results in a row that disagree with the box (it may have moved)
    public float createdTime;
    public float lastSeen;

    // Tentative: measurements are averaged until the box is confirmed.
    Vector3 sumCenter;
    Vector3 sumSize;
    int sampleCount;
    OneEuroFilter3 centerFilter;

    // Drawing
    readonly GameObject root;
    readonly LineRenderer[] lines = new LineRenderer[6]; // bottom loop, top loop, 4 vertical edges
    Vector3 drawnCenter;
    Vector3 drawnSize;
    Quaternion drawnRotation = Quaternion.identity;
    Vector3 builtSize = new Vector3(-1f, -1f, -1f);
    readonly Vector3[] localCorners = new Vector3[8];
    readonly Vector3[] loop = new Vector3[4];
    readonly Vector3[] edge = new Vector3[2];
    float flashStart = -1f;
    bool highlighted;
    Color highlightColor;
    Material highlightFill;
    Color appliedColor;
    float appliedWidth = -1f;

    public ObjectCube(int serial, string className, Material lineMaterial)
    {
        this.serial = serial;
        this.className = className;
        root = new GameObject($"YoloCube {className} #{serial}");
        silhouette = new ObjectSilhouette($"YoloSilhouette {className} #{serial}", lineMaterial);
        root.SetActive(false); // tentative boxes are not drawn
        for (int i = 0; i < lines.Length; i++)
        {
            var line = new GameObject(i < 2 ? "Loop" : "Edge").AddComponent<LineRenderer>();
            line.transform.SetParent(root.transform, false);
            line.useWorldSpace = false;  // corners are local: moving the box only moves the root transform
            line.loop = i < 2;
            line.positionCount = i < 2 ? 4 : 2;
            line.sharedMaterial = lineMaterial;
            line.numCornerVertices = 0;
            line.numCapVertices = 0;
            lines[i] = line;
        }
    }

    public int SampleCount => sampleCount;
    public bool Visible => state == State.Confirmed;
    public Vector3 DrawnCenter => drawnCenter;
    public Vector3 DrawnSize => drawnSize;

    // Center of the box's front face (the side the first camera saw): the silhouette's fixed point.
    public Vector3 FrontCenter => center - rotation * Vector3.forward * (size.z * 0.5f);
    public Vector3 DrawnFrontCenter => drawnCenter - drawnRotation * Vector3.forward * (drawnSize.z * 0.5f);

    public Vector3 PredictedCenter(float now, float maxPrediction)
        => center + velocity * Mathf.Clamp(now - lastMeasureTime, 0f, maxPrediction);

    // ---------- measurements ----------

    // Tentative only: one more measurement to average.
    public void AddSample(Vector3 measuredCenter, Vector3 measuredSize)
    {
        sumCenter += measuredCenter;
        sumSize += measuredSize;
        sampleCount++;
        center = sumCenter / sampleCount;
        size = sumSize / sampleCount;
    }

    // Becomes a drawn box at the averaged measurements.
    public void Confirm(float now, float minCutoff, float beta)
    {
        state = State.Confirmed;
        centerFilter = new OneEuroFilter3(minCutoff, beta);
        centerFilter.Reset(center, now);
        lastMeasureTime = now;
        velocity = Vector3.zero;
        drawnCenter = center;
        drawnSize = size;
        drawnRotation = rotation;
        RebuildLines();
        root.transform.SetPositionAndRotation(drawnCenter, drawnRotation);
        root.SetActive(drawBox);
        flashStart = now;
    }

    // Confirmed only: a new measurement of the center (the object may have moved).
    // Changes smaller than deadband are ignored so a resting box stays perfectly still. Returns true when it moved.
    public bool ApplyMeasurement(Vector3 measuredCenter, float now, float deadband)
    {
        float dt = now - lastMeasureTime;
        if ((measuredCenter - center).magnitude < deadband)
        {
            lastMeasureTime = now;
            velocity *= 0.5f;
            return false;
        }
        Vector3 previous = center;
        center = centerFilter.Filter(measuredCenter, now);
        velocity = dt > 1e-3f && dt < 1f ? (center - previous) / dt : Vector3.zero;
        lastMeasureTime = now;
        return true;
    }

    // World corners of a box (0-3 bottom, 4-7 top).
    public static void GetCorners(Vector3 boxCenter, Vector3 boxSize, Quaternion boxRotation, Vector3[] result)
    {
        for (int i = 0; i < 8; i++)
            result[i] = boxCenter + boxRotation * Vector3.Scale(k_UnitCorners[i], boxSize);
    }

    public void GetDrawnCorners(Vector3[] result) => GetCorners(drawnCenter, drawnSize, drawnRotation, result);

    // ---------- drawing ----------

    // fill: the silhouette's fill while highlighted (null = none).
    public void SetHighlight(bool on, Color color, Material fill = null)
    {
        highlighted = on;
        highlightColor = color;
        highlightFill = on ? fill : null;
    }

    // Eases the drawn box to the box and updates its look. t: how far to move this frame (0..1).
    // head: the viewer's position (the silhouette faces it).
    public void Animate(float t, float now, Vector3 head, Color normalColor, float normalWidth, Color flashColor, float flashDuration)
    {
        if (!Visible)
        {
            silhouette.Render(false, default, default, default, 0f, null);
            return;
        }

        drawnCenter = Vector3.Lerp(drawnCenter, center, t);
        drawnSize = Vector3.Lerp(drawnSize, size, t);
        drawnRotation = Quaternion.Slerp(drawnRotation, rotation, t);
        if (root.activeSelf != drawBox)
            root.SetActive(drawBox);
        if (drawBox)
        {
            root.transform.SetPositionAndRotation(drawnCenter, drawnRotation);
            if ((drawnSize - builtSize).sqrMagnitude > 1e-6f)
                RebuildLines();
        }

        Color color = highlighted ? highlightColor : normalColor;
        float width = highlighted ? normalWidth * 1.75f : normalWidth;
        if (flashStart >= 0f)
        {
            float k = (now - flashStart) / Mathf.Max(0.01f, flashDuration);
            if (k >= 1f)
            {
                flashStart = -1f;
            }
            else
            {
                float e = (1f - k) * (1f - k); // ease out
                color = Color.Lerp(color, flashColor, e);
                width *= 1f + 2.5f * e;
            }
        }
        silhouette.Render(!drawBox, DrawnFrontCenter, head, color, width, highlighted ? highlightFill : null);
        if (!drawBox)
            return;
        if (color != appliedColor || !Mathf.Approximately(width, appliedWidth))
        {
            appliedColor = color;
            appliedWidth = width;
            foreach (var line in lines)
            {
                line.startColor = color;
                line.endColor = color;
                line.widthMultiplier = width;
            }
        }
    }

    void RebuildLines()
    {
        builtSize = drawnSize;
        for (int i = 0; i < 8; i++)
            localCorners[i] = Vector3.Scale(k_UnitCorners[i], drawnSize);

        for (int i = 0; i < 4; i++)
            loop[i] = localCorners[i];
        lines[0].SetPositions(loop);
        for (int i = 0; i < 4; i++)
            loop[i] = localCorners[i + 4];
        lines[1].SetPositions(loop);
        for (int i = 0; i < 4; i++)
        {
            edge[0] = localCorners[i];
            edge[1] = localCorners[i + 4];
            lines[2 + i].SetPositions(edge);
        }
    }

    // ---------- pointing ----------

    // True when the ray hits what is drawn (the silhouette, or the box); distance along the ray to the hit.
    public bool TryHit(Ray ray, out float distance)
    {
        distance = 0f;
        if (!Visible)
            return false;
        if (!drawBox)
            return silhouette.TryHit(ray, out distance);
        Quaternion inverse = Quaternion.Inverse(drawnRotation);
        Vector3 origin = inverse * (ray.origin - drawnCenter);
        Vector3 direction = inverse * ray.direction;
        Vector3 half = drawnSize * 0.5f;

        float tMin = float.NegativeInfinity, tMax = float.PositiveInfinity;
        for (int axis = 0; axis < 3; axis++)
        {
            float o = origin[axis], d = direction[axis], h = half[axis];
            if (Mathf.Abs(d) < 1e-6f)
            {
                if (o < -h || o > h)
                    return false;
                continue;
            }
            float t1 = (-h - o) / d, t2 = (h - o) / d;
            if (t1 > t2) (t1, t2) = (t2, t1);
            tMin = Mathf.Max(tMin, t1);
            tMax = Mathf.Min(tMax, t2);
            if (tMin > tMax)
                return false;
        }
        if (tMax < 0f)
            return false;            // box is behind the ray
        distance = tMin >= 0f ? tMin : tMax; // ray starts inside the box
        return true;
    }

    public void Destroy()
    {
        Object.Destroy(root);
        silhouette.Destroy();
    }
}
