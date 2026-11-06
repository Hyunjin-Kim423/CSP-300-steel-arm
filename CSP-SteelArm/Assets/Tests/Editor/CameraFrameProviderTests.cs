using NUnit.Framework;
using UnityEngine;

public class CameraFrameProviderTests
{
    [Test]
    public void MockFrameIsExplicitlyMarkedAndInternallyConsistent()
    {
        var provider = new MockCameraFrameProvider(640, 480);
        Assert.That(provider.TryGetFrame(out var texture, out var metadata), Is.True);
        Assert.That(metadata.IsMock, Is.True);
        Assert.That(metadata.Status, Does.StartWith("MOCK:"));
        Assert.That(texture.width, Is.EqualTo(metadata.SensorResolution.x));
        Assert.That(texture.height, Is.EqualTo(metadata.SensorResolution.y));
        Assert.That(metadata.FocalLength.x, Is.GreaterThan(0));
    }
}
