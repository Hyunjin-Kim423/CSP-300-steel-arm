using System.Collections.Generic;
using RosMessageTypes.Yolo;
using UnityEngine;

// One object found by YOLO, as drawn in the world.
//
//  - Outline: one or more closed parts (a person behind a chair can be split in two), or the bounding box.
//  - Fill: shown only while the object is pointed at.
//  - Info for the label and the menu (detection, text, color).
//
// Shapes are not drawn directly: each YOLO result sets a TARGET shape, and Animate() moves the drawn (CURRENT)
// shape towards it every frame, so outlines change smoothly instead of jumping.
public class DetectedObject
{
    // One closed outline (+ its fill).
    public class Part
    {
        public LineRenderer line;
        public MeshFilter fill;
        public Vector3[] target = new Vector3[0];   // latest shape from YOLO (world space)
        public Vector3[] current = new Vector3[0];  // drawn shape, follows target
        public int[] triangles;                     // fill triangulation of the target shape (null = rebuild)
    }

    public string key;            // track id (live, tracked), "#n" (live, untracked) or "s<scan>:<n>" (scan mode)
    public bool active;
    public float lastSeen;

    // What YOLO said
    public DetectionMsg detection;
    public string text;           // e.g. "cup 0.87"
    public Color color;           // fill / label color (registered in the JSON or not)

    // Where it is
    public bool hasDepth;
    public float depth;                                          // distance from the camera that saw it
    public Vector3 cameraPosition;                               // where that camera was
    public readonly Vector3[] targetCorners = new Vector3[4];   // bounding box: 0 left-bottom, 1 left-top, 2 right-top, 3 right-bottom
    public readonly Vector3[] corners = new Vector3[4];         // drawn bounding box (follows targetCorners)

    // How it is drawn
    public bool hasMask;                                         // outline from the mask (false = bounding box)
    public readonly List<Part> parts = new List<Part>();         // pooled; only the first partCount are in use
    public int partCount;

    public Vector3 Center => (corners[0] + corners[2]) * 0.5f;

    // ---------- shape ----------

    // Stores a new target shape for a part. With align, the new points are rotated so point k lands next to the
    // currently drawn point k, which keeps the morph from sliding around the outline.
    public static void SetTarget(Part part, Vector3[] target, bool align)
    {
        int n = target.Length;
        if (align && part.current.Length == n)
        {
            int bestShift = 0;
            float bestCost = float.MaxValue;
            for (int shift = 0; shift < n; shift++)
            {
                float cost = 0f;
                for (int k = 0; k < n && cost < bestCost; k++)
                    cost += (target[(k + shift) % n] - part.current[k]).sqrMagnitude;
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
        else if (part.current.Length != n)
        {
            part.current = (Vector3[])target.Clone();
        }
        part.target = target;
        part.triangles = null; // the fill must be re-triangulated for the new shape
    }

    // Jumps straight to the target shape (new object, or the outline changed too much to morph).
    public void SnapToTarget()
    {
        System.Array.Copy(targetCorners, corners, 4);
        for (int i = 0; i < partCount; i++)
            System.Array.Copy(parts[i].target, parts[i].current, parts[i].target.Length);
    }

    // Moves the drawn shape a step towards the target. t: how far to move this frame (0..1).
    public void Animate(float t)
    {
        for (int c = 0; c < 4; c++)
            corners[c] = Vector3.Lerp(corners[c], targetCorners[c], t);
        for (int i = 0; i < partCount; i++)
        {
            Part part = parts[i];
            for (int k = 0; k < part.current.Length; k++)
                part.current[k] = Vector3.Lerp(part.current[k], part.target[k], t);
            part.line.SetPositions(part.current);
        }
    }

    // ---------- outline ----------

    // Shows the outlines of the parts in use and hides the unused ones.
    public void ShowOutline()
    {
        for (int i = 0; i < partCount; i++)
        {
            Part part = parts[i];
            part.line.gameObject.SetActive(true);
            part.line.positionCount = part.current.Length;
            part.line.SetPositions(part.current);
        }
        HidePartsFrom(partCount);
    }

    public void HideAll() => HidePartsFrom(0);

    void HidePartsFrom(int index)
    {
        for (int i = index; i < parts.Count; i++)
        {
            parts[i].line.gameObject.SetActive(false);
            parts[i].fill.gameObject.SetActive(false);
        }
    }

    // ---------- fill (only while pointed at) ----------

    public void ShowFill(Material material)
    {
        for (int i = 0; i < partCount; i++)
        {
            Part part = parts[i];
            Mesh mesh = part.fill.sharedMesh;
            if (part.triangles == null)
            {
                // Triangulate the target shape once; the morphing vertices reuse the same triangles.
                s_Polygon.Clear();
                foreach (var p in part.target)
                    s_Polygon.Add(ToBoxUV(targetCorners, p));
                part.triangles = Polygon2D.Triangulate(s_Polygon);
                mesh.Clear();
                mesh.vertices = part.current;
                mesh.triangles = part.triangles;
            }
            var renderer = part.fill.GetComponent<MeshRenderer>();
            if (renderer.sharedMaterial != material)
                renderer.sharedMaterial = material;
            part.fill.gameObject.SetActive(true);
        }
        for (int i = partCount; i < parts.Count; i++)
            parts[i].fill.gameObject.SetActive(false);
    }

    public void HideFill()
    {
        foreach (var part in parts)
            part.fill.gameObject.SetActive(false);
    }

    // Keeps the fill glued to the morphing outline. Call every frame while the fill is shown.
    public void UpdateFill()
    {
        for (int i = 0; i < partCount; i++)
        {
            Part part = parts[i];
            if (part.triangles == null)
                continue;
            Mesh mesh = part.fill.sharedMesh;
            mesh.vertices = part.current;
            mesh.RecalculateBounds();
        }
    }

    // ---------- pointing ----------

    // True when the ray hits this object: inside its outline, or inside its box when it has no mask.
    // Testing the outline instead of the box keeps touching / overlapping objects apart.
    public bool TryHit(Ray ray, out float distance)
    {
        distance = 0f;
        if (!new Plane(corners[0], corners[1], corners[2]).Raycast(ray, out distance))
            return false;
        Vector2 uv = ToBoxUV(corners, ray.GetPoint(distance));
        if (uv.x < 0f || uv.x > 1f || uv.y < 0f || uv.y > 1f)
            return false;
        return !hasMask || OutlineContains(uv);
    }

    bool OutlineContains(Vector2 boxUV)
    {
        for (int i = 0; i < partCount; i++)
        {
            s_Polygon.Clear();
            foreach (var p in parts[i].current)
                s_Polygon.Add(ToBoxUV(corners, p));
            if (Polygon2D.Contains(s_Polygon, boxUV))
                return true;
        }
        return false;
    }

    // Position of a world point on the box plane, as 0..1 across the box (x: left->right, y: bottom->top).
    static Vector2 ToBoxUV(Vector3[] box, Vector3 p)
    {
        Vector3 d = p - box[0];
        Vector3 right = box[3] - box[0];
        Vector3 up = box[1] - box[0];
        return new Vector2(Vector3.Dot(d, right) / right.sqrMagnitude, Vector3.Dot(d, up) / up.sqrMagnitude);
    }

    static readonly List<Vector2> s_Polygon = new List<Vector2>();
}
