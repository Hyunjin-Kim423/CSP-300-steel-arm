using UnityEngine;
using PassthroughCameraAccess = Meta.XR.PassthroughCameraAccess;

// One camera frame that was sent to YOLO, with the camera pose at the moment it was taken.
// Converts between positions in that image (YOLO pixels / viewport) and world positions.
public readonly struct ScanFrame
{
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
}
