using System.Collections.Generic;
using UnityEngine;

// The drawn silhouette of one object: an outline (and, while pointed at, a fill) around the object's anchor point.
//
// Position: the anchor is a fixed point in the world (the front of the object's box, see ObjectCube).
// Shape: every YOLO mask of the object is projected, with the camera pose of the image it came from, onto the plane
// through the anchor that faces that camera. There it is stored in meters around the anchor, which removes head
// movement: the same object seen after turning or stepping lands in the same place. The masks are averaged and the
// outline is traced from the average (MaskGrid).
// Drawing: the outline lies in a plane through the anchor that always faces the viewer. The shape is the one seen
// from the latest images; when the user walks around, new masks gradually take over. If the view direction jumps
// (object seen again from another side), the average starts over.
public class ObjectSilhouette
{
    const float ViewResetAngle = 40f;   // degrees between two masks' view directions that restart the average

    readonly GameObject root;
    readonly Material lineMaterial;
    readonly List<LineRenderer> lines = new List<LineRenderer>();   // one line + fill per outline part
    readonly List<MeshFilter> fills = new List<MeshFilter>();
    readonly MaskGrid grid = new MaskGrid();
    Vector3 lastView;                   // direction camera -> anchor of the latest mask
    bool fillDirty = true;
    Color appliedColor;
    float appliedWidth = -1f;

    public ObjectSilhouette(string name, Material lineMaterial)
    {
        this.lineMaterial = lineMaterial;
        root = new GameObject(name);
        root.SetActive(false);
    }

    public bool HasShape => grid.PartCount > 0;
    public float LastUpdateTime { get; private set; } = float.NegativeInfinity;
    public Vector3 Position => root.transform.position;

    // Half the width of the outline in meters (keeps the menu from covering the object).
    public float HalfWidth
    {
        get
        {
            float xMin = 0f, xMax = 0f;
            for (int p = 0; p < grid.PartCount; p++)
            {
                foreach (var point in grid.parts[p])
                {
                    xMin = Mathf.Min(xMin, point.x);
                    xMax = Mathf.Max(xMax, point.x);
                }
            }
            return Mathf.Max(0.02f, (xMax - xMin) * 0.5f);
        }
    }

    // ---------- shape ----------

    // regions: the mask outline(s) in viewport coordinates of `frame`. anchor: the object's fixed point in the world.
    // weight: share of this mask in the average (1 = replace). blurPasses: how much the outline is rounded.
    public void AddMask(IReadOnlyList<List<Vector2>> regions, ScanFrame frame, Vector3 anchor, float weight, int blurPasses, float now)
    {
        Vector3 toAnchor = anchor - frame.CameraPosition;
        if (toAnchor.sqrMagnitude < 1e-4f)
            return;
        Vector3 view = toAnchor.normalized;
        int count = ProjectToAnchorPlane(regions, frame, anchor, view, out float extent);
        if (count == 0)
            return;

        bool restart = !grid.Fits(extent) || Vector3.Angle(view, lastView) > ViewResetAngle;
        if (restart)
            grid.Restart(extent);
        grid.Add(s_Polygons, count, weight, blurPasses);
        if (restart || grid.PartCount == 0)
        {
            grid.ShowNow();
            fillDirty = true;
        }
        else
        {
            // Blend to the new shape until about when the next result is due.
            grid.BlendTo(now, Mathf.Clamp(now - LastUpdateTime, 0.03f, 0.25f));
        }
        lastView = view;
        LastUpdateTime = now;
    }

    // Mask points -> meters around the anchor, in the plane through the anchor facing along `view`.
    // Fills s_Polygons and returns how many it filled. extent: the farthest point's distance from the anchor.
    static int ProjectToAnchorPlane(IReadOnlyList<List<Vector2>> regions, ScanFrame frame, Vector3 anchor, Vector3 view, out float extent)
    {
        Quaternion toPlane = Quaternion.Inverse(PlaneRotation(view));
        int count = 0;
        extent = 0f;
        foreach (var region in regions)
        {
            if (count == s_Polygons.Count)
                s_Polygons.Add(new List<Vector2>());
            var polygon = s_Polygons[count];
            polygon.Clear();
            foreach (var viewportPoint in region)
            {
                Ray ray = frame.ViewportRay(viewportPoint);
                float along = Vector3.Dot(ray.direction, view);
                if (along < 1e-3f)
                    continue;
                Vector3 onPlane = ray.origin + ray.direction * (Vector3.Dot(anchor - ray.origin, view) / along);
                Vector3 local = toPlane * (onPlane - anchor);
                polygon.Add(new Vector2(local.x, local.y));
                extent = Mathf.Max(extent, Mathf.Abs(local.x), Mathf.Abs(local.y));
            }
            if (polygon.Count >= 3)
                count++; // otherwise its list is reused for the next region
        }
        return count;
    }

    // ---------- drawing ----------

    // Call every frame while the object is shown. Draws at `anchor`, facing the viewer at `head`.
    // fillMaterial: null = outline only.
    public void Render(Vector3 anchor, Vector3 head, Color color, float width, Material fillMaterial, float now)
    {
        if (grid.UpdateBlend(now))
            fillDirty = true;
        int partCount = grid.PartCount;
        if (root.activeSelf != partCount > 0)
            root.SetActive(partCount > 0);
        if (partCount == 0)
            return;

        Vector3 view = anchor - head;
        root.transform.SetPositionAndRotation(anchor, PlaneRotation(view.sqrMagnitude > 1e-6f ? view.normalized : Vector3.forward));

        while (lines.Count < partCount)
            AddPart();
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
        for (int p = 0; p < lines.Count; p++)
        {
            bool used = p < partCount;
            if (lines[p].gameObject.activeSelf != used)
                lines[p].gameObject.SetActive(used);
            if (fills[p].gameObject.activeSelf != (used && fill))
                fills[p].gameObject.SetActive(used && fill);
            if (!used)
                continue;

            var points = grid.parts[p];
            for (int k = 0; k < MaskGrid.PointsPerPart; k++)
                s_Points[k] = points[k];
            lines[p].SetPositions(s_Points);
            if (!fill)
                continue;

            var renderer = fills[p].GetComponent<MeshRenderer>();
            if (renderer.sharedMaterial != fillMaterial)
                renderer.sharedMaterial = fillMaterial;
            if (fillDirty)
            {
                Mesh mesh = fills[p].sharedMesh;
                mesh.Clear();
                mesh.vertices = s_Points;
                mesh.triangles = Polygon2D.Triangulate(points);
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
        line.useWorldSpace = false; // points are in the anchor plane; moving the object only moves the root
        line.loop = true;
        line.positionCount = MaskGrid.PointsPerPart;
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
        appliedWidth = -1f; // makes Render apply the color and width to the new line too
    }

    // ---------- pointing / menu ----------

    // True when the ray hits inside the drawn outline; distance along the ray to the hit.
    public bool TryHit(Ray ray, out float distance)
    {
        distance = 0f;
        if (!root.activeSelf)
            return false;
        Transform t = root.transform;
        if (!new Plane(t.forward, t.position).Raycast(ray, out distance) || distance < 0f)
            return false;
        Vector2 local = t.InverseTransformPoint(ray.GetPoint(distance));
        for (int p = 0; p < grid.PartCount; p++)
        {
            if (Polygon2D.Contains(grid.parts[p], local))
                return true;
        }
        return false;
    }

    // The drawn outline points in world space.
    public void GetWorldOutline(List<Vector3> result)
    {
        result.Clear();
        Transform t = root.transform;
        for (int p = 0; p < grid.PartCount; p++)
        {
            foreach (var point in grid.parts[p])
                result.Add(t.TransformPoint(point));
        }
    }

    public void Destroy() => Object.Destroy(root);

    // Plane facing along `view`, kept upright (local x = right, y = up as seen by the viewer).
    static Quaternion PlaneRotation(Vector3 view)
    {
        Vector3 up = Mathf.Abs(Vector3.Dot(view, Vector3.up)) > 0.98f ? Vector3.forward : Vector3.up;
        return Quaternion.LookRotation(view, up);
    }

    // Scratch data shared by all silhouettes (everything runs on the main thread).
    static readonly List<List<Vector2>> s_Polygons = new List<List<Vector2>>();
    static readonly Vector3[] s_Points = new Vector3[MaskGrid.PointsPerPart];
}
