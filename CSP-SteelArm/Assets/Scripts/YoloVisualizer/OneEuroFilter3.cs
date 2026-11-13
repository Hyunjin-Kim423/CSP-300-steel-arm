using UnityEngine;

// One Euro filter for a Vector3 (Casiez et al. 2012), for samples that arrive at an irregular rate.
// Smooths strongly while the value is (nearly) still and relaxes the smoothing as it moves faster,
// so a resting object stays steady and a moving one does not lag much.
//   minCutoff  smoothing while still (Hz). Lower = steadier, more lag.
//   beta       how much the speed relaxes the smoothing. Higher = less lag on fast moves.
public class OneEuroFilter3
{
    public float minCutoff;
    public float beta;
    public float derivativeCutoff;

    Vector3 value;
    Vector3 derivative;
    float lastTime;
    bool hasValue;

    public OneEuroFilter3(float minCutoff, float beta, float derivativeCutoff = 1f)
    {
        this.minCutoff = minCutoff;
        this.beta = beta;
        this.derivativeCutoff = derivativeCutoff;
    }

    public Vector3 Value => value;

    public void Reset(Vector3 start, float time)
    {
        value = start;
        derivative = Vector3.zero;
        lastTime = time;
        hasValue = true;
    }

    public Vector3 Filter(Vector3 sample, float time)
    {
        if (!hasValue)
        {
            Reset(sample, time);
            return value;
        }
        float dt = Mathf.Max(1e-3f, time - lastTime);
        lastTime = time;

        Vector3 rawDerivative = (sample - value) / dt;
        derivative = Vector3.Lerp(derivative, rawDerivative, Alpha(derivativeCutoff, dt));
        float cutoff = minCutoff + beta * derivative.magnitude;
        value = Vector3.Lerp(value, sample, Alpha(cutoff, dt));
        return value;
    }

    static float Alpha(float cutoff, float dt)
    {
        float tau = 1f / (2f * Mathf.PI * Mathf.Max(1e-3f, cutoff));
        return 1f / (1f + tau / dt);
    }
}
