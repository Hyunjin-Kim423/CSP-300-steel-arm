using UnityEngine;
using UnityEngine.XR.ARFoundation;

// The "scanning" wave: rings of light that spread from the headset over the real surfaces around the player.
//
// It needs the headset's depth (AR Foundation occlusion). Depth is switched on with the first wave and stays on while
// waves are playing; it is switched off only after depthIdleTimeout seconds without waves, so pressing the scan button
// repeatedly never keeps restarting it. At most MaxWaves waves play at once; a new one replaces the oldest.
public class ScanWaveEffect : MonoBehaviour
{
    [Tooltip("Optional. Leave empty to use Resources/Shaders/ScanWave.")]
    public Shader shader;
    public Color color = new Color(0.45f, 0.85f, 1f, 1f);
    [Tooltip("How fast the wave spreads (m/s).")]
    public float speed = 2.5f;
    [Tooltip("The wave fades out and stops at this distance (m).")]
    public float maxRadius = 4f;
    [Tooltip("Width of the bright wave front (m).")]
    public float bandWidth = 0.06f;
    [Tooltip("Length of the soft glow behind the front (m).")]
    public float trailLength = 0.35f;
    [Range(0f, 1f)] public float intensity = 0.8f;
    [Tooltip("Depth stays on this long after the last wave, so repeated scans don't restart it (s).")]
    public float depthIdleTimeout = 3f;
    [Tooltip("After switching depth on, a wave waits at most this long for the first depth frame before it starts (s).")]
    public float depthWarmupTimeout = 0.5f;

    const int MaxWaves = 4;

    struct Wave
    {
        public bool active;
        public bool waitingForDepth;
        public Vector3 origin;
        public float startTime;
    }

    readonly Wave[] waves = new Wave[MaxWaves];
    readonly Vector4[] waveOrigins = new Vector4[MaxWaves];
    readonly float[] waveFade = new float[MaxWaves];
    static readonly int s_WaveOriginsId = Shader.PropertyToID("_WaveOrigins");
    static readonly int s_WaveFadeId = Shader.PropertyToID("_WaveFade");

    AROcclusionManager occlusion;
    ARShaderOcclusion shaderOcclusion;
    bool depthOn;
    bool depthReady;
    float depthOnSince;
    float lastActiveTime;
    MeshRenderer quad;
    Material material;

    // Starts a new wave from origin (usually the headset position).
    public void Play(Vector3 origin)
    {
        if (!EnsureSetUp())
            return;
        SwitchDepth(true);

        int slot = 0;
        for (int i = 0; i < MaxWaves; i++)
        {
            if (!waves[i].active)
            {
                slot = i;
                break;
            }
            if (waves[i].startTime < waves[slot].startTime)
                slot = i; // all busy: replace the oldest
        }
        waves[slot] = new Wave { active = true, waitingForDepth = !depthReady, origin = origin, startTime = Time.time };
        lastActiveTime = Time.time;
    }

    void Update()
    {
        if (material == null)
            return;

        bool anyActive = false;
        for (int i = 0; i < MaxWaves; i++)
        {
            ref Wave wave = ref waves[i];
            waveOrigins[i] = Vector4.zero; // w <= 0: unused
            waveFade[i] = 0f;
            if (!wave.active)
                continue;
            anyActive = true;

            if (wave.waitingForDepth)
            {
                if (!depthReady && Time.time - depthOnSince < depthWarmupTimeout)
                    continue;
                wave.waitingForDepth = false;
                wave.startTime = Time.time;
            }

            float radius = (Time.time - wave.startTime) * speed;
            if (radius >= maxRadius)
            {
                wave.active = false;
                continue;
            }
            waveOrigins[i] = new Vector4(wave.origin.x, wave.origin.y, wave.origin.z, Mathf.Max(radius, 0.001f));
            waveFade[i] = 1f - radius / maxRadius;
        }

        material.SetVectorArray(s_WaveOriginsId, waveOrigins);
        material.SetFloatArray(s_WaveFadeId, waveFade);
        quad.enabled = anyActive;

        if (anyActive)
            lastActiveTime = Time.time;
        else if (depthOn && Time.time - lastActiveTime > depthIdleTimeout)
            SwitchDepth(false);
    }

    // ---------- set-up ----------

    bool EnsureSetUp()
    {
        if (material != null)
            return true;

        Shader waveShader = shader != null ? shader : Shader.Find("SteelArm/ScanWave");
        Camera cam = Camera.main;
        if (waveShader == null || cam == null)
        {
            Debug.LogWarning("[steelarm] Scan wave disabled: " + (waveShader == null ? "shader SteelArm/ScanWave not found." : "no main camera."));
            enabled = false;
            return false;
        }

        material = new Material(waveShader);
        material.SetColor("_Color", color);
        material.SetFloat("_BandWidth", bandWidth);
        material.SetFloat("_TrailLength", trailLength);
        material.SetFloat("_Intensity", intensity);

        // A quad in front of the eyes, large enough to cover the whole view; the shader draws on the real surfaces behind it.
        var go = GameObject.CreatePrimitive(PrimitiveType.Quad);
        go.name = "ScanWave";
        Destroy(go.GetComponent<Collider>());
        go.transform.SetParent(cam.transform, false);
        go.transform.localPosition = new Vector3(0f, 0f, 0.5f);
        go.transform.localScale = new Vector3(4f, 4f, 1f);
        quad = go.GetComponent<MeshRenderer>();
        quad.sharedMaterial = material;
        quad.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        quad.receiveShadows = false;
        quad.enabled = false;

        // The MR template camera already carries AROcclusionManager + ARShaderOcclusion (disabled).
        occlusion = FindAnyObjectByType<AROcclusionManager>(FindObjectsInactive.Include);
        if (occlusion == null)
            occlusion = cam.gameObject.AddComponent<AROcclusionManager>();
        if (!occlusion.TryGetComponent(out shaderOcclusion))
            shaderOcclusion = occlusion.gameObject.AddComponent<ARShaderOcclusion>();
        occlusion.frameReceived += _ => depthReady = true;
        return true;
    }

    void SwitchDepth(bool on)
    {
        if (on == depthOn)
            return;
        depthOn = on;
        depthReady = false;
        if (on)
        {
            depthOnSince = Time.time;
            // The wave shader reads the raw depth, so the soft-occlusion preprocessing pass is not needed.
            shaderOcclusion.occlusionShaderMode = AROcclusionShaderMode.HardOcclusion;
        }
        occlusion.enabled = on;
        shaderOcclusion.enabled = on;
    }
}
