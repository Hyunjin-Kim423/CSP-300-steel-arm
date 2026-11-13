using System.Collections.Generic;
using UnityEngine;

// The averaged mask of one object, and the outline traced from it. Used by ObjectSilhouette.
//
// Masks arrive as polygons in meters around the object's anchor. Each one is drawn into a small grid
// (Size x Size cells), and the grid keeps a running average of them. Averaging cells instead of outline points needs
// no point matching and copes with shapes that split or merge.
//
//   Add          draws a mask, mixes it into the average, and blurs a copy of the average a little: the new target
//                shape. The blur rounds off the stair steps of the mask and the flicker of single cells on the edge.
//   BlendTo      the shown shape then moves from what is shown now to the target over a short time
//   UpdateBlend  (every frame) and the outline is traced from it again (marching squares at the 50 % level),
//                so the shape changes continuously instead of in steps.
public class MaskGrid
{
    public const int PointsPerPart = 48;   // every outline part is resampled to this many points
    const int Size = 48;
    const int MaxParts = 4;

    // The outline: closed parts in meters around the anchor (x right, y up). Only the first PartCount are in use.
    public readonly List<List<Vector2>> parts = new List<List<Vector2>>();
    public int PartCount { get; private set; }

    readonly float[] average = new float[Size * Size];
    readonly float[] target = new float[Size * Size];     // blurred average: the shape to blend to
    readonly float[] previous = new float[Size * Size];   // what was shown when the blend started
    readonly float[] shown = new float[Size * Size];      // shown now; the outline is traced from it
    readonly float[] blurTemp = new float[Size * Size];
    readonly byte[] coverage = new byte[Size * Size];     // the newest mask
    float half;            // half the grid width in meters (0 = nothing added yet)
    float cell;            // cell size in meters
    bool fresh;            // just restarted: the next mask replaces the (empty) average
    bool blending;
    float blendStart;
    float blendDuration;

    // True when a mask reaching `extent` meters from the anchor fits the grid without being tiny in it.
    public bool Fits(float extent) => half > 0f && extent <= half * 0.95f && extent >= half * 0.25f;

    // Starts over with a grid sized for masks reaching `extent` meters from the anchor.
    public void Restart(float extent)
    {
        half = Mathf.Max(0.03f, extent * 1.4f);
        cell = 2f * half / Size;
        System.Array.Clear(average, 0, average.Length);
        fresh = true;
    }

    // Mixes a mask (the first `count` polygons) into the average. weight: 0..1 share of the new mask.
    public void Add(List<List<Vector2>> polygons, int count, float weight, int blurPasses)
    {
        Rasterize(polygons, count);
        weight = fresh ? 1f : Mathf.Clamp01(weight);
        fresh = false;
        for (int i = 0; i < average.Length; i++)
            average[i] += (coverage[i] - average[i]) * weight;
        Blur(blurPasses);
    }

    // Shows the target shape at once.
    public void ShowNow()
    {
        System.Array.Copy(target, shown, target.Length);
        blending = false;
        TraceOutline();
    }

    // Moves the shown shape to the target over `duration` seconds (UpdateBlend does the moving).
    public void BlendTo(float now, float duration)
    {
        System.Array.Copy(shown, previous, shown.Length);
        blendStart = now;
        blendDuration = duration;
        blending = true;
    }

    // Call every frame. Returns true when the outline changed.
    public bool UpdateBlend(float now)
    {
        if (!blending)
            return false;
        float u = (now - blendStart) / blendDuration;
        if (u >= 1f)
        {
            u = 1f;
            blending = false;
        }
        for (int i = 0; i < shown.Length; i++)
            shown[i] = previous[i] + (target[i] - previous[i]) * u;
        TraceOutline();
        return true;
    }

    // ---------- mask -> grid ----------

    // Even-odd scanline fill of the polygons into `coverage` (tested at the cell centers).
    void Rasterize(List<List<Vector2>> polygons, int count)
    {
        System.Array.Clear(coverage, 0, coverage.Length);
        for (int y = 0; y < Size; y++)
        {
            float py = -half + (y + 0.5f) * cell;
            s_Crossings.Clear();
            for (int p = 0; p < count; p++)
            {
                var poly = polygons[p];
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
                int x1 = Mathf.Min(Size - 1, Mathf.FloorToInt((s_Crossings[k + 1] + half) / cell - 0.5f));
                for (int x = x0; x <= x1; x++)
                    coverage[y * Size + x] = 1;
            }
        }
    }

    // target = average blurred with a [1 2 1] / 4 kernel (each pass along x, then y). Outside the grid counts as empty.
    void Blur(int passes)
    {
        System.Array.Copy(average, target, average.Length);
        for (int pass = 0; pass < passes; pass++)
        {
            for (int y = 0; y < Size; y++)
            {
                int row = y * Size;
                for (int x = 0; x < Size; x++)
                {
                    float left = x > 0 ? target[row + x - 1] : 0f;
                    float right = x < Size - 1 ? target[row + x + 1] : 0f;
                    blurTemp[row + x] = (left + 2f * target[row + x] + right) * 0.25f;
                }
            }
            for (int y = 0; y < Size; y++)
            {
                for (int x = 0; x < Size; x++)
                {
                    float down = y > 0 ? blurTemp[(y - 1) * Size + x] : 0f;
                    float up = y < Size - 1 ? blurTemp[(y + 1) * Size + x] : 0f;
                    target[y * Size + x] = (down + 2f * blurTemp[y * Size + x] + up) * 0.25f;
                }
            }
        }
    }

    // ---------- grid -> outline (marching squares) ----------

    float Value(int x, int y) => x < 0 || y < 0 || x >= Size || y >= Size ? 0f : shown[y * Size + x];

    // Edge keys: the horizontal edge from corner (x, y) to (x + 1, y) and the vertical edge from (x, y) to (x, y + 1),
    // for corners -1..Size (the grid is surrounded by empty cells, so every outline closes).
    static int HKey(int x, int y) => ((y + 1) * (Size + 2) + (x + 1)) * 2;
    static int VKey(int x, int y) => ((y + 1) * (Size + 2) + (x + 1)) * 2 + 1;

    Vector2 Corner(int x, int y) => new Vector2(-half + (x + 0.5f) * cell, -half + (y + 0.5f) * cell);

    // Where the 50 % level crosses this edge.
    Vector2 EdgePoint(int key)
    {
        int index = key >> 1;
        int x = index % (Size + 2) - 1;
        int y = index / (Size + 2) - 1;
        bool vertical = (key & 1) == 1;
        int x2 = vertical ? x : x + 1;
        int y2 = vertical ? y + 1 : y;
        float a = Value(x, y), b = Value(x2, y2);
        float t = Mathf.Abs(b - a) > 1e-6f ? Mathf.Clamp01((0.5f - a) / (b - a)) : 0.5f;
        return Vector2.Lerp(Corner(x, y), Corner(x2, y2), t);
    }

    // The outline passes from edge a to edge b.
    static void Link(int a, int b)
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
        // 1. In every square of four neighbouring cells, link the edges the outline crosses.
        s_Links.Clear();
        for (int y = -1; y < Size; y++)
        {
            for (int x = -1; x < Size; x++)
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

        // 2. Chain the links into closed loops.
        s_LoopCount = 0;
        s_Visited.Clear();
        foreach (var start in s_Links.Keys)
        {
            if (s_Visited.Contains(start))
                continue;
            if (s_LoopCount == s_Loops.Count)
                s_Loops.Add(new List<Vector2>());
            var loop = s_Loops[s_LoopCount];
            loop.Clear();
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
                s_LoopCount++; // otherwise its list is reused for the next loop
        }

        // 3. Keep the outer outlines (holes are dropped), largest first, without small specks.
        s_Areas.Clear();
        float maxArea = 0f;
        for (int i = 0; i < s_LoopCount; i++)
        {
            s_Areas.Add(Mathf.Abs(SignedArea(s_Loops[i])));
            maxArea = Mathf.Max(maxArea, s_Areas[i]);
        }
        s_Order.Clear();
        for (int i = 0; i < s_LoopCount; i++)
        {
            if (s_Areas[i] >= Mathf.Max(maxArea * 0.05f, cell * cell * 4f) && !IsHole(i))
                s_Order.Add(i);
        }
        s_Order.Sort(s_LargestFirst);

        PartCount = Mathf.Min(MaxParts, s_Order.Count);
        while (parts.Count < PartCount)
            parts.Add(new List<Vector2>(PointsPerPart));
        for (int p = 0; p < PartCount; p++)
            MaskOutline.Resample(s_Loops[s_Order[p]], PointsPerPart, parts[p]);
    }

    // A loop that lies inside a bigger one is a hole (e.g. the opening in a chair's back).
    static bool IsHole(int i)
    {
        Vector2 probe = s_Loops[i][0];
        for (int j = 0; j < s_LoopCount; j++)
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

    // Scratch data shared by all grids (everything runs on the main thread).
    static readonly List<float> s_Crossings = new List<float>();
    static readonly Dictionary<int, Vector2Int> s_Links = new Dictionary<int, Vector2Int>();
    static readonly HashSet<int> s_Visited = new HashSet<int>();
    static readonly List<List<Vector2>> s_Loops = new List<List<Vector2>>();
    static int s_LoopCount;
    static readonly List<float> s_Areas = new List<float>();
    static readonly List<int> s_Order = new List<int>();
    static readonly System.Comparison<int> s_LargestFirst = (a, b) => s_Areas[b].CompareTo(s_Areas[a]);
}
