using System.Collections.Generic;
using RosMessageTypes.Yolo;
using UnityEngine;
using PassthroughCameraAccess = Meta.XR.PassthroughCameraAccess;

// One camera frame that was sent to YOLO, with the camera pose at the moment it was taken.
// Converts between positions in that image (YOLO pixels / viewport) and world positions.
public readonly struct ScanFrame
{
    static readonly List<List<Vector2>> s_NoRegions = new List<List<Vector2>>();

    readonly PassthroughCameraAccess camera;
    readonly Pose pose;
    public readonly Vector3 forward;     // camera's optical axis
    public readonly Vector3 up;          // camera's up
    public readonly float imageWidth;    // size of the image YOLO saw, in pixels
    public readonly float imageHeight;

    public ScanFrame(PassthroughCameraAccess camera, Pose pose, Vector2Int imageSize)
    {
        this.camera = camera;
        this.pose = pose;
        forward = pose.rotation * Vector3.forward;
        up = pose.rotation * Vector3.up;
        imageWidth = imageSize.x;
        imageHeight = imageSize.y;
    }

    public Vector3 CameraPosition => pose.position;

    // YOLO pixels have a top-left origin; the camera viewport has a bottom-left origin.
    public Vector2 PixelToViewport(double x, double y) => new Vector2((float)(x / imageWidth), 1f - (float)(y / imageHeight));

    public Ray ViewportRay(Vector2 viewport) => camera.ViewportPointToRay(viewport, pose);

    // World point seen at this viewport position, on the plane `depth` meters in front of the camera.
    public Vector3 ViewportToWorld(Vector2 viewport, float depth)
    {
        Ray ray = ViewportRay(viewport);
        return ray.origin + ray.direction * (depth / Vector3.Dot(ray.direction, forward));
    }

    // Where a world point appears in this image (viewport, may be outside 0..1). False when it is behind the camera.
    public bool WorldToViewport(Vector3 world, out Vector2 viewport)
    {
        if (Vector3.Dot(world - pose.position, forward) < 0.05f)
        {
            viewport = default;
            return false;
        }
        viewport = camera.WorldToViewportPoint(world, pose);
        return true;
    }

    // The YOLO box of a result, as a viewport rectangle.
    public Rect BoxRect(DetectionMsg det)
    {
        var center = det.bbox.center.position;
        var size = det.bbox.size;
        Vector2 leftBottom = PixelToViewport(center.x - size.x / 2, center.y + size.y / 2);
        Vector2 rightTop = PixelToViewport(center.x + size.x / 2, center.y - size.y / 2);
        return Rect.MinMaxRect(leftBottom.x, leftBottom.y, rightTop.x, rightTop.y);
    }

    // The mask outline(s) of a result in viewport coordinates; empty when it has no mask (no -seg model).
    // The returned list is reused by the next call.
    public IReadOnlyList<List<Vector2>> MaskRegions(DetectionMsg det)
    {
        var points = det.mask?.data;
        if (points == null || points.Length < 3)
            return s_NoRegions;
        return MaskOutline.SplitIntoRegions(points, imageWidth, imageHeight);
    }
}
