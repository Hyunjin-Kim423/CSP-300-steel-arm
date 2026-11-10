using System.Collections.Generic;
using UnityEngine;

// The silhouette of one object: its POSITION is fixed in the world (the object's anchor point), only its SHAPE is
// refined by every new YOLO mask.
//
// How a mask becomes a stable shape:
//   1. Each mask point is projected, with the camera pose of the frame it came from, onto the plane through the
//      anchor that faces that camera. In that plane the point is stored in meters relative to the anchor.
//      This removes head movement: the same object seen after turning or stepping lands at the same place.
//   2. The mask is drawn into a small grid (GridSize x GridSize cells around the anchor), and the grid keeps a running
//      average of the masks (alpha = weight of the new one). Averaging cells instead of outline points needs no point
//      matching, and handles shapes that split or merge.
//   3. The averaged grid is blurred a little (a copy; the average itself is not blurred). The blur rounds off the
//      stair steps of the mask and the flicker of single cells along the edge.
//   4. Between two results the shown grid blends from the previous shape to the new one, and the outline is traced
//      out of it every frame (marching squares, 50 % level), so the shape changes continuously instead of in steps.
//
// Drawing: the outline lies in a plane through the anchor that always faces the viewer, so it looks the same size and
// shape from any direction. The shape itself is the one seen from the last update; when the user walks around, the new
// masks slowly take over. If the view changes a lot at once (object seen again from the other side), the average is
// started over.
public class ObjectSilhouette
{
    public const int GridSize = 48;
    const int MaxParts = 4;
    const int PointsPerPart = 48;

    readonly GameObject root;
    readonly Material lineMaterial;
    readonly List<LineRenderer> lines = new List<LineRenderer>();
    readonly List<MeshFilter> fills = new List<MeshFilter>();

    // Averaged mask
    readonly float[] grid = new float[GridSize * GridSize];
    readonly float[] target = new float[GridSize * GridSize];     // blurred average: the shape to blend to
    readonly float[] previous = new float[GridSize * GridSize];   // what was shown when the newest result arrived
    readonly float[] display = new float[GridSize * GridSize];    // shown now; the outline is traced from it
    readonly float[] blurTemp = new float[GridSize * GridSize];
    bool blending;
    float blendStart;
    float blendDuration;
    readonly byte[] coverage = new byte[GridSize * GridSize];
    bool hasGrid;
    float half;            // half the grid width, meters
    float cell;            // cell size, meters
    Vector3 lastView;      // direction camera -> anchor of the last update
    float lastUpdateTime = float.NegativeInfinity;

    // Current outline: parts in the anchor plane (meters, x right, y up)
    readonly List<List<Vector2>> parts = new List<List<Vector2>>();
    int partCount;
    readonly Vector3[] lineBuffer = new Vector3[PointsPerPart];
    bool fillDirty = true;
    Color appliedColor;
    float appliedWidth = -1f;

    public ObjectSilhouette(string name, Material lineMaterial)
    {
        this.lineMaterial = lineMaterial;
        root = new GameObject(name);
        root.SetActive(false);
    }

    public bool HasShape => partCount > 0;
    public float LastUpdateTime => lastUpdateTime;
    public Vector3 Position => root.transform.position;
    public Quaternion Rotation => root.transform.rotation;

    // Half the width of the drawn outline in meters (for the menu placement).
    public float HalfWidth
    {
        get
        {
            float xMin = 0f, xMax = 0f;
            for (int p = 0; p < partCount; p++)
            {
                foreach (var v in parts[p])
                {
                    xMin = Mathf.Min(xMin, v.x);
                    xMax = Mathf.Max(xMax, v.x);
                }
            }
            return Mathf.Max(0.02f, (xMax - xMin) * 0.5f);
        }
    }

    // ---------- adding masks ----------

    // regions: the mask outline(s) in viewport coordinates of `frame` (see MaskOutline.SplitIntoRegions).
    // anchor: the object's fixed point in the world. alpha: weight of this mask in the average (1 = replace).
    // resetAngle: if the view direction changed more than this since the last update, the average starts over.
    // blurPasses: how much the outline is rounded (0 = none). now: Time.time.
    public void AddMask(IReadOnlyList<List<Vector2>> regions, ScanFrame frame, Vector3 anchor, float alpha, float resetAngle,
        int blurPasses, float now)
    {
        Vector3 toAnchor = anchor - frame.CameraPosition;
        if (toAnchor.sqrMagnitude < 1e-4f || regions.Count == 0)
            return;
        Vector3 view = toAnchor.normalized;
        Quaternion inverse = Quaternion.Inverse(PlaneRotation(view));

        // 1. Mask -> anchor plane (meters around the anchor).
        s_Local.Clear();
        float extent = 0f;
        foreach (var region in regions)
        {
            var local = RentList();
            foreach (var v in region)
            {
                Ray ray = frame.ViewportRay(v);
                float denom = Vector3.Dot(ray.direction, view);
                if (denom < 1e-3f)
                    continue;
                Vector3 p = ray.origin + ray.direction * (Vector3.Dot(anchor - ray.origin, view) / denom);
                Vector3 q = inverse * (p - anchor);
                local.Add(new Vector2(q.x, q.y));
                extent = Mathf.Max(extent, Mathf.Abs(q.x), Mathf.Abs(q.y));
            }
            if (local.Count >= 3)
                s_Local.Add(local);
            else
                ReturnList(local);
        }
        if (s_Local.Count == 0)
        {
            ReturnAll();
            return;
        }

        // Start over: first mask, a big jump in view direction, or the shape no longer fits the grid.
        bool restart = !hasGrid
                       || Vector3.Angle(view, lastView) > resetAngle
                       || extent > half * 0.95f
                       || extent < half * 0.25f;
        if (restart)
        {
            half = Mathf.Max(0.03f, extent * 1.4f);
            cell = 2f * half / GridSize;
            System.Array.Clear(grid, 0, grid.Length);
            alpha = 1f;
            hasGrid = true;
        }
        lastView = view;
        float sinceLast = now - lastUpdateTime;
        lastUpdateTime = now;

        // 2. Rasterize and average.
        Rasterize(s_Local);
        ReturnAll();
        alpha = Mathf.Clamp01(alpha);
        for (int i = 0; i < grid.Length; i++)
            grid[i] += (coverage[i] - grid[i]) * alpha;

        // 3. Blurred copy = the new target shape.
        Blur(blurPasses);

        // 4. Blend to it until about when the next result is due (Render traces the outline every frame).
        if (restart || partCount == 0)
        {
            System.Array.Copy(target, display, target.Length);
            System.Array.Copy(target, previous, target.Length);
            blending = false;
            TraceOutline();
            fillDirty = true;
        }
        else
        {
            System.Array.Copy(display, previous, display.Length);
            blendStart = now;
            blendDuration = Mathf.Clamp(sinceLast, 0.03f, 0.25f);
            blending = true;
        }
    }

    // Moves the shown shape along the blend. Returns true when the outline changed.
    bool UpdateBlend(float now)
    {
        if (!blending)
            return false;
        float u = (now - blendStart) / blendDuration;
        if (u >= 1f)
        {
            u = 1f;
            blending = false;
        }
        for (int i = 0; i < display.Length; i++)
            display[i] = previous[i] + (target[i] - previous[i]) * u;
        TraceOutline();
        fillDirty = true;
        return true;
    }

    // Even-odd scanline fill of all regions into `coverage` (cell centers).
    void Rasterize(List<List<Vector2>> polygons)
    {
        System.Array.Clear(coverage, 0, coverage.Length);
        for (int y = 0; y < GridSize; y++)
        {
            float py = -half + (y + 0.5f) * cell;
            s_Crossings.Clear();
            foreach (var poly in polygons)
            {
                for (int i = 0, j = poly.Count - 1; i < poly.Count; j = i++)
                {
                    Vector2 a = poly[i], b = poly[j];
                    if ((a.y > py) != (b.y > py))
                        s_Crossings.Add(a.x + (py - a.y) * (b.x - a.x) / (b.y - a.y));
                }
            }
            if (s_Crossings.Count < 2)
                continue;
            s_Crossings.Sort();
            for (int k = 0; k + 1 < s_Crossings.Count; k += 2)
            {
                int x0 = Mathf.Max(0, Mathf.CeilToInt((s_Crossings[k] + half) / cell - 0.5f));
                int x1 = Mathf.Min(GridSize - 1, Mathf.FloorToInt((s_Crossings[k + 1] + half) / cell - 0.5f));
                for (int x = x0; x <= x1; x++)
                    coverage[y * GridSize + x] = 1;
            }
        }
    }

    // target = grid blurred with a [1 2 1] / 4 kernel (each pass along x, then y). Outside the grid counts as empty.
    void Blur(int passes)
    {
        float[] smoothed = target;
        System.Array.Copy(grid, smoothed, grid.Length);
        for (int pass = 0; pass < passes; pass++)
        {
            for (int y = 0; y < GridSize; y++)
            {
                int row = y * GridSize;
                for (int x = 0; x < GridSize; x++)
                {
                    float left = x > 0 ? smoothed[row + x - 1] : 0f;
                    float right = x < GridSize - 1 ? smoothed[row + x + 1] : 0f;
                    blurTemp[row + x] = (left + 2f * smoothed[row + x] + right) * 0.25f;
                }
            }
            for (int y = 0; y < GridSize; y++)
            {
                for (int x = 0; x < GridSize; x++)
                {
                    float down = y > 0 ? blurTemp[(y - 1) * GridSize + x] : 0f;
                    float up = y < GridSize - 1 ? blurTemp[(y + 1) * GridSize + x] : 0f;
                    smoothed[y * GridSize + x] = (down + 2f * blurTemp[y * GridSize + x] + up) * 0.25f;
                }
            }
        }
    }

    // ---------- marching squares ----------

    float Value(int x, int y) => x < 0 || y < 0 || x >= GridSize || y >= GridSize ? 0f : display[y * GridSize + x];

    // Edge keys: horizontal edge from corner (x, y) to (x + 1, y), vertical edge from (x, y) to (x, y + 1),
    // with corners -1..GridSize (the grid is surrounded by empty cells, so every outline closes).
    static int HKey(int x, int y) => ((y + 1) * (GridSize + 2) + (x + 1)) * 2;
    static int VKey(int x, int y) => ((y + 1) * (GridSize + 2) + (x + 1)) * 2 + 1;

    Vector2 Corner(int x, int y) => new Vector2(-half + (x + 0.5f) * cell, -half + (y + 0.5f) * cell);

    Vector2 EdgePoint(int key)
    {
        int index = key >> 1;
        int x = index % (GridSize + 2) - 1;
        int y = index / (GridSize + 2) - 1;
        bool vertical = (key & 1) == 1;
        int x2 = vertical ? x : x + 1;
        int y2 = vertical ? y + 1 : y;
        float a = Value(x, y), b = Value(x2, y2);
        float t = Mathf.Abs(b - a) > 1e-6f ? Mathf.Clamp01((0.5f - a) / (b - a)) : 0.5f;
        return Vector2.Lerp(Corner(x, y), Corner(x2, y2), t);
    }

    void Link(int a, int b)
    {
        AddNeighbor(a, b);
        AddNeighbor(b, a);
    }

    static void AddNeighbor(int key, int other)
    {
        if (s_Links.TryGetValue(key, out var pair))
            s_Links[key] = new Vector2Int(pair.x, other);
        else
            s_Links[key] = new Vector2Int(other, -1);
    }

    void TraceOutline()
    {
        s_Links.Clear();
        for (int y = -1; y < GridSize; y++)
        {
            for (int x = -1; x < GridSize; x++)
            {
                float bl = Value(x, y), br = Value(x + 1, y), tr = Value(x + 1, y + 1), tl = Value(x, y + 1);
                int c = (bl >= 0.5f ? 1 : 0) | (br >= 0.5f ? 2 : 0) | (tr >= 0.5f ? 4 : 0) | (tl >= 0.5f ? 8 : 0);
                if (c == 0 || c == 15)
                    continue;
                int B = HKey(x, y), T = HKey(x, y + 1), L = VKey(x, y), R = VKey(x + 1, y);
                bool centerIn = (bl + br + tr + tl) * 0.25f >= 0.5f;
                switch (c)
                {
                    case 1: case 14: Link(L, B); break;
                    case 2: case 13: Link(B, R); break;
                    case 3: case 12: Link(L, R); break;
                    case 4: case 11: Link(R, T); break;
                    case 6: case 9: Link(B, T); break;
                    case 7: case 8: Link(L, T); break;
                    case 5:
                        if (centerIn) { Link(L, T); Link(B, R); } else { Link(L, B); Link(R, T); }
                        break;
                    case 10:
                        if (centerIn) { Link(L, B); Link(R, T); } else { Link(B, R); Link(L, T); }
                        break;
                }
            }
        }

        // Chain the links into closed loops.
        RecycleLoops();
        s_Visited.Clear();
        foreach (var start in s_Links.Keys)
        {
            if (s_Visited.Contains(start))
                continue;
            var loop = RentList();
            int prev = -1, cur = start;
            for (int guard = 0; guard < 100000; guard++)
            {
                s_Visited.Add(cur);
                loop.Add(EdgePoint(cur));
                Vector2Int n = s_Links[cur];
                int next = n.x != prev ? n.x : n.y;
                if (next < 0 || next == start || s_Visited.Contains(next))
                    break;
                prev = cur;
                cur = next;
            }
            if (loop.Count >= 4)
                s_Loops.Add(loop);
            else
                ReturnList(loop);
        }

        // Keep the outer outlines (holes are dropped), largest first, small specks removed.
        s_Areas.Clear();
        float maxArea = 0f;
        foreach (var loop in s_Loops)
        {
            float area = Mathf.Abs(SignedArea(loop));
            s_Areas.Add(area);
            maxArea = Mathf.Max(maxArea, area);
        }
        s_Order.Clear();
        for (int i = 0; i < s_Loops.Count; i++)
        {
            if (s_Areas[i] < Mathf.Max(maxArea * 0.05f, cell * cell * 4f))
                continue;
            if (IsHole(i))
                continue;
            s_Order.Add(i);
        }
        s_Order.Sort(s_ByAreaDescending);

        partCount = Mathf.Min(MaxParts, s_Order.Count);
        while (parts.Count < partCount)
            parts.Add(new List<Vector2>(PointsPerPart));
        for (int p = 0; p < partCount; p++)
            MaskOutline.Resample(s_Loops[s_Order[p]], PointsPerPart, parts[p]);
    }

    // A loop that lies inside a bigger one is a hole (e.g. the opening in a chair's back).
    bool IsHole(int i)
    {
        Vector2 probe = s_Loops[i][0];
        for (int j = 0; j < s_Loops.Count; j++)
        {
            if (j != i && s_Areas[j] > s_Areas[i] && Polygon2D.Contains(s_Loops[j], probe))
                return true;
        }
        return false;
    }

    static float SignedArea(List<Vector2> poly)
    {
        float area = 0f;
        for (int i = 0, j = poly.Count - 1; i < poly.Count; j = i++)
            area += poly[j].x * poly[i].y - poly[i].x * poly[j].y;
        return area * 0.5f;
    }

    // ---------- drawing ----------

    // Places the silhouette at `anchor`, facing the viewer at `head`. fillMaterial: null = no fill. now: Time.time.
    public void Render(bool visible, Vector3 anchor, Vector3 head, Color color, float width, Material fillMaterial, float now)
    {
        if (visible)
            UpdateBlend(now);
        if (!visible || partCount == 0)
        {
            if (root.activeSelf)
                root.SetActive(false);
            return;
        }
        if (!root.activeSelf)
            root.SetActive(true);

        Vector3 view = anchor - head;
        root.transform.SetPositionAndRotation(anchor, PlaneRotation(view.sqrMagnitude > 1e-6f ? view.normalized : Vector3.forward));

        while (lines.Count < partCount)
            AddPart();
        for (int p = 0; p < lines.Count; p++)
        {
            bool used = p < partCount;
            if (lines[p].gameObject.activeSelf != used)
                lines[p].gameObject.SetActive(used);
            if (!used)
            {
                fills[p].gameObject.SetActive(false);
                continue;
            }
            var points = parts[p];
            for (int k = 0; k < PointsPerPart; k++)
                lineBuffer[k] = points[k];
            lines[p].SetPositions(lineBuffer);
        }

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

        bool fill = fillMaterial != null;
        for (int p = 0; p < partCount; p++)
        {
            MeshFilter f = fills[p];
            if (f.gameObject.activeSelf != fill)
                f.gameObject.SetActive(fill);
            if (!fill)
                continue;
            var renderer = f.GetComponent<MeshRenderer>();
            if (renderer.sharedMaterial != fillMaterial)
                renderer.sharedMaterial = fillMaterial;
            if (fillDirty)
            {
                Mesh mesh = f.sharedMesh;
                mesh.Clear();
                for (int k = 0; k < PointsPerPart; k++)
                    s_Vertices[k] = parts[p][k];
                mesh.vertices = s_Vertices;
                mesh.triangles = Polygon2D.Triangulate(parts[p]);
                mesh.RecalculateBounds();
            }
        }
        if (fill)
            fillDirty = false;
    }

    void AddPart()
    {
        var line = new GameObject("Outline").AddComponent<LineRenderer>();
        line.transform.SetParent(root.transform, false);
        line.useWorldSpace = false;
        line.loop = true;
        line.positionCount = PointsPerPart;
        line.sharedMaterial = lineMaterial;
        line.alignment = LineAlignment.View;
        lines.Add(line);

        var fillObject = new GameObject("Fill");
        fillObject.transform.SetParent(root.transform, false);
        var filter = fillObject.AddComponent<MeshFilter>();
        filter.sharedMesh = new Mesh { name = "SilhouetteFill" };
        filter.sharedMesh.MarkDynamic();
        fillObject.AddComponent<MeshRenderer>();
        fillObject.SetActive(false);
        fills.Add(filter);
        appliedWidth = -1f; // apply the look to the new line too
    }

    // ---------- pointing / menu ----------

    // True when the ray hits inside the drawn outline.
    public bool TryHit(Ray ray, out float distance)
    {
        distance = 0f;
        if (!root.activeSelf || partCount == 0)
            return false;
        Transform t = root.transform;
        var plane = new Plane(t.forward, t.position);
        if (!plane.Raycast(ray, out distance) || distance < 0f)
            return false;
        Vector3 local = t.InverseTransformPoint(ray.GetPoint(distance));
        var p = new Vector2(local.x, local.y);
        for (int i = 0; i < partCount; i++)
        {
            if (Polygon2D.Contains(parts[i], p))
                return true;
        }
        return false;
    }

    // The drawn outline points in world space.
    public void GetWorldOutline(List<Vector3> result)
    {
        result.Clear();
        Transform t = root.transform;
        for (int i = 0; i < partCount; i++)
        {
            foreach (var v in parts[i])
                result.Add(t.TransformPoint(v));
        }
    }

    public void Destroy() => Object.Destroy(root);

    // Plane facing along `view`, kept upright (local x = right, y = up as seen by the viewer).
    static Quaternion PlaneRotation(Vector3 view)
    {
        Vector3 up = Mathf.Abs(Vector3.Dot(view, Vector3.up)) > 0.98f ? Vector3.forward : Vector3.up;
        return Quaternion.LookRotation(view, up);
    }

    // ---------- shared scratch lists ----------

    static readonly List<List<Vector2>> s_Local = new List<List<Vector2>>();
    static readonly List<List<Vector2>> s_Loops = new List<List<Vector2>>();
    static readonly Stack<List<Vector2>> s_Pool = new Stack<List<Vector2>>();
    static readonly List<float> s_Crossings = new List<float>();
    static readonly List<float> s_Areas = new List<float>();
    static readonly List<int> s_Order = new List<int>();
    static readonly Dictionary<int, Vector2Int> s_Links = new Dictionary<int, Vector2Int>();
    static readonly HashSet<int> s_Visited = new HashSet<int>();
    static readonly Vector3[] s_Vertices = new Vector3[PointsPerPart];
    static readonly System.Comparison<int> s_ByAreaDescending = (a, b) => s_Areas[b].CompareTo(s_Areas[a]);

    static List<Vector2> RentList() => s_Pool.Count > 0 ? s_Pool.Pop() : new List<Vector2>();

    static void ReturnList(List<Vector2> list)
    {
        list.Clear();
        s_Pool.Push(list);
    }

    static void ReturnAll()
    {
        foreach (var list in s_Local)
            ReturnList(list);
        s_Local.Clear();
    }

    static void RecycleLoops()
    {
        foreach (var list in s_Loops)
            ReturnList(list);
        s_Loops.Clear();
    }
}
