using System;
using UnityEngine;

public struct CameraFrameMetadata
{
    public bool IsValid, IsMock; public string Status, CameraIdentity; public Pose WorldPose; public DateTime TimestampUtc;
    public Vector2 FocalLength, PrincipalPoint; public Vector2Int SensorResolution;
}

public interface ICameraFrameProvider { bool TryGetFrame(out Texture texture, out CameraFrameMetadata metadata); }

/// Uses documented MRUK PCA properties: Intrinsics, GetCameraPose, Timestamp and CameraPosition.
public sealed class QuestCameraFrameProvider : ICameraFrameProvider
{
    readonly Meta.XR.PassthroughCameraAccess access;
    public QuestCameraFrameProvider(Meta.XR.PassthroughCameraAccess access) { this.access = access; }
    public bool TryGetFrame(out Texture texture, out CameraFrameMetadata metadata)
    {
        texture = null; metadata = new CameraFrameMetadata { Status = "camera access component is missing" };
        if (access == null) return false;
        if (!access.IsPlaying) { metadata.Status = "camera permission not granted or camera is not playing"; return false; }
        texture = access.GetTexture(); if (texture == null) { metadata.Status = "camera texture unavailable"; return false; }
        var i = access.Intrinsics;
        metadata = new CameraFrameMetadata {
            IsValid = i.FocalLength.x > 0 && i.FocalLength.y > 0, IsMock = false, Status = "live",
            CameraIdentity = access.CameraPosition.ToString(), WorldPose = access.GetCameraPose(), TimestampUtc = access.Timestamp.ToUniversalTime(),
            FocalLength = i.FocalLength, PrincipalPoint = i.PrincipalPoint, SensorResolution = i.SensorResolution
        };
        if (!metadata.IsValid) metadata.Status = "camera intrinsics unavailable";
        return metadata.IsValid;
    }
}

/// Editor-only deterministic source. Its status and frame ID prohibit silent physical-calibration use.
public sealed class MockCameraFrameProvider : ICameraFrameProvider
{
    readonly Texture2D texture;
    public MockCameraFrameProvider(int width, int height)
    {
        texture = new Texture2D(width, height, TextureFormat.RGB24, false); var pixels = new Color32[width * height];
        for (var y = 0; y < height; ++y) for (var x = 0; x < width; ++x) pixels[y * width + x] = ((x / 32 + y / 32) & 1) == 0 ? new Color32(255, 0, 255, 255) : new Color32(0, 0, 0, 255);
        texture.SetPixels32(pixels); texture.Apply(false, true);
    }
    public bool TryGetFrame(out Texture result, out CameraFrameMetadata metadata)
    {
        result = texture; metadata = new CameraFrameMetadata {
            IsValid = true, IsMock = true, Status = "MOCK: synthetic checkerboard; never physical calibration", CameraIdentity = "mock_left",
            WorldPose = new Pose(new Vector3(.1f, 1.5f, -.2f), Quaternion.identity), TimestampUtc = DateTime.UtcNow,
            FocalLength = new Vector2(600, 600), PrincipalPoint = new Vector2(texture.width / 2f, texture.height / 2f), SensorResolution = new Vector2Int(texture.width, texture.height)
        }; return true;
    }
}
