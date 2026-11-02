using System.Collections.Generic;
using UnityEngine;

// Small 2D polygon helpers used for the outline fill and for "is the controller pointing inside this outline?".
public static class Polygon2D
{
    // True when p lies inside the polygon (even-odd rule).
    public static bool Contains(List<Vector2> poly, Vector2 p)
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

    // Ear-clipping triangulation of a simple (possibly concave) polygon.
    // If the outline self-intersects and no ear can be found, the remaining part is left unfilled.
    public static int[] Triangulate(List<Vector2> pts)
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
}
