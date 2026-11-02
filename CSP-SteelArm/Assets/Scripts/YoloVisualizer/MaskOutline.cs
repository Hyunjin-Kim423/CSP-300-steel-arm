using System.Collections.Generic;
using RosMessageTypes.Yolo;
using UnityEngine;

// Turns the point list of a YOLO segmentation mask into clean closed outlines.
//
// A YOLO mask can have several separate regions (e.g. a person split by a chair in front of them), but yolo_ros sends
// them as ONE point list joined by thin "bridges". SplitIntoRegions undoes that, and Resample gives each outline a fixed
// point count so consecutive outlines can be morphed point by point.
public static class MaskOutline
{
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
    // Each chain that returns to its start is one region. Returns the regions in viewport coordinates (bottom-left origin); the list is reused by the next call.
    // Limitation: a 0/45/90-degree bridge that leaves a region before it is closed is not detected and stays as a thin slit.
    public static IReadOnlyList<List<Vector2>> SplitIntoRegions(Point2DMsg[] points, float w, float h)
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
        return s_Regions;
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

    // Resamples a closed polygon to exactly n points evenly spaced along its perimeter, counter-clockwise.
    public static void Resample(List<Vector2> poly, int n, List<Vector2> result)
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
}
