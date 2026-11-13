using RosMessageTypes.Yolo;
using UnityEngine;

// One real object found by YOLO, kept as a gravity-aligned box in the world.
//
// The box is in WORLD space: it does not follow the head. It is moved only by new measurements of the object
// (ObjectTracker), filtered so a resting object stays still and a moving one is followed.
// The box itself is never drawn. It is used to match new YOLO results to this object, and its front face is the
// anchor of what IS drawn: the object's silhouette (ObjectSilhouette), which eases after the box.
//
//   Tentative   seen, not drawn yet. Its measurements are averaged until YOLO has seen it often enough.
//   Confirmed   drawn, and matched against every new YOLO result.
public class ObjectCube
{
    public enum State { Tentative, Confirmed }

    const float FollowSpeed = 10f;           // how fast the drawn silhouette eases to the box (higher = snappier)
    const float DrawPrediction = 0.1f;       // live: between results, draw at most this far (seconds) ahead along the velocity
    const float FlashDuration = 0.5f;        // a new object's outline starts in the flash color
    const float HighlightWidth = 1.75f;      // the pointed object's outline is this much thicker
    const float OneEuroBeta = 2f;            // position filter: how much fast moves relax the smoothing
    static readonly Color k_FlashColor = new Color(0.55f, 0.9f, 1f, 1f);

    // Unit box centered at the origin (bottom face 0-3, top face 4-7). Local +z points away from the camera that
    // measured the object, so the front face is at -z.
    static readonly Vector3[] k_UnitCorners =
    {
        new Vector3(-0.5f, -0.5f, -0.5f), new Vector3(0.5f, -0.5f, -0.5f), new Vector3(0.5f, -0.5f, 0.5f), new Vector3(-0.5f, -0.5f, 0.5f),
        new Vector3(-0.5f,  0.5f, -0.5f), new Vector3(0.5f,  0.5f, -0.5f), new Vector3(0.5f,  0.5f, 0.5f), new Vector3(-0.5f,  0.5f, 0.5f),
    };

    public readonly int serial;
    public readonly string className;
    public readonly ObjectSilhouette silhouette;
    public State state = State.Tentative;
    public DetectionMsg detection;   // latest matched YOLO result (class, score)
    public bool registered;          // listed in yolo_dectection_list.json (has robot actions)
    public bool predictWhenDrawn;    // live mode: keep the drawn silhouette moving between results

    // The box: where the object is believed to be (world space).
    public Vector3 center;
    public Vector3 size;             // x: width, y: height, z: depth (front to back)
    public Quaternion rotation;      // yaw only; faces the camera that measured it
    public Vector3 velocity;         // m/s
    public float lastMeasureTime;

    // Matching bookkeeping (ObjectTracker)
    public int hits;                 // results matched to this object
    public int misses;               // results in a row in which it was in view but not found
    public int jumpStreak;           // measurements in a row far from where it should be (depth outliers?)
    public float createdTime;
    public float lastSeen;

    Vector3 sumCenter;               // tentative: measurements are averaged
    Vector3 sumSize;
    int sampleCount;
    OneEuroFilter3 centerFilter;

    // Drawing
    Vector3 drawnAnchor;             // where the silhouette is drawn; eases to FrontCenter
    float fade;                      // 0..1 opacity: fades in when shown, out when removed
    float flashStart = -1f;
    bool highlighted;
    Color highlightColor;
    Material highlightFill;

    public ObjectCube(int serial, string className, Material lineMaterial)
    {
        this.serial = serial;
        this.className = className;
        silhouette = new ObjectSilhouette($"YoloObject {className} #{serial}", lineMaterial);
    }

    public bool Visible => state == State.Confirmed;
    public bool Dying { get; private set; }
    public bool FadedOut => Dying && fade <= 0.001f;

    // Center of the box's front face: the fixed point the silhouette is anchored to.
    public Vector3 FrontCenter => center - rotation * Vector3.forward * (size.z * 0.5f);

    // Where the box should be now, following its velocity for at most maxPrediction seconds.
    public Vector3 PredictedCenter(float now, float maxPrediction)
        => center + velocity * Mathf.Clamp(now - lastMeasureTime, 0f, maxPrediction);

    // World corners of the box when its center is at boxCenter (0-3 bottom, 4-7 top).
    public void GetCorners(Vector3 boxCenter, Vector3[] result)
    {
        for (int i = 0; i < 8; i++)
            result[i] = boxCenter + rotation * Vector3.Scale(k_UnitCorners[i], size);
    }

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

    // Becomes a drawn object at the averaged measurements. minCutoff: position filter smoothing (Hz).
    public void Confirm(float now, float minCutoff)
    {
        state = State.Confirmed;
        centerFilter = new OneEuroFilter3(minCutoff, OneEuroBeta);
        centerFilter.Reset(center, now);
        lastMeasureTime = now;
        velocity = Vector3.zero;
        drawnAnchor = FrontCenter;
        flashStart = now;
        fade = 0f;
    }

    // Confirmed only: a new measurement of the center. Changes smaller than deadband are ignored, so a resting
    // object stays perfectly still. Returns true when it moved.
    public bool ApplyMeasurement(Vector3 measuredCenter, float now, float deadband)
    {
        float dt = now - lastMeasureTime;
        lastMeasureTime = now;
        if ((measuredCenter - center).magnitude < deadband)
        {
            velocity *= 0.5f;
            return false;
        }
        Vector3 previous = center;
        center = centerFilter.Filter(measuredCenter, now);
        Vector3 rawVelocity = dt > 1e-3f && dt < 1f ? (center - previous) / dt : Vector3.zero;
        velocity = Vector3.Lerp(velocity, rawVelocity, 0.5f); // a little smoothing: it is used for prediction
        return true;
    }

    // The box turns to face the camera that measured it and its size follows the measurements (k: 0..1 per call).
    public void UpdateShape(Vector3 measuredSize, Quaternion facing, float k)
    {
        size = Vector3.Lerp(size, measuredSize, k);
        rotation = Quaternion.Slerp(rotation, facing, k);
    }

    // Jumps to a new position (the object really is somewhere else; don't slide there slowly).
    public void ResetPosition(Vector3 newCenter, float now)
    {
        center = newCenter;
        velocity = Vector3.zero;
        centerFilter.Reset(newCenter, now);
        lastMeasureTime = now;
    }

    // ---------- drawing ----------

    // fill: the silhouette's fill while highlighted (null = none).
    public void SetHighlight(bool on, Color color, Material fill = null)
    {
        highlighted = on;
        highlightColor = color;
        highlightFill = on ? fill : null;
    }

    // Removed from tracking: fades out. Destroy it once FadedOut.
    public void StartDying()
    {
        Dying = true;
        SetHighlight(false, default);
    }

    // Call every frame. head: the viewer's position (the silhouette faces it). fadeDuration: seconds (0 = no fading).
    public void Animate(float now, float deltaTime, Vector3 head, Color normalColor, float normalWidth, float fadeDuration)
    {
        if (!Visible)
            return;
        fade = Mathf.MoveTowards(fade, Dying ? 0f : 1f, fadeDuration > 0f ? deltaTime / fadeDuration : 1f);

        Vector3 target = FrontCenter;
        if (predictWhenDrawn)
            target += velocity * Mathf.Clamp(now - lastMeasureTime, 0f, DrawPrediction);
        drawnAnchor = Vector3.Lerp(drawnAnchor, target, 1f - Mathf.Exp(-FollowSpeed * deltaTime));

        Color color = highlighted ? highlightColor : normalColor;
        if (flashStart >= 0f)
        {
            float k = (now - flashStart) / FlashDuration;
            if (k >= 1f)
                flashStart = -1f;
            else
                color = Color.Lerp(color, k_FlashColor, (1f - k) * (1f - k)); // color only: the outline keeps its size
        }
        color.a *= fade;
        float width = highlighted ? normalWidth * HighlightWidth : normalWidth;
        silhouette.Render(drawnAnchor, head, color, width, highlighted ? highlightFill : null, now);
    }

    // True when the ray hits inside the drawn silhouette; distance along the ray to the hit.
    public bool TryHit(Ray ray, out float distance)
    {
        distance = 0f;
        return Visible && !Dying && silhouette.TryHit(ray, out distance);
    }

    public void Destroy() => silhouette.Destroy();
}
