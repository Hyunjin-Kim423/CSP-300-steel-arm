// Scan wave: rings of light that spread from a point (the headset) over the REAL surfaces around the player.
// Drawn on a quad in front of the camera. For each pixel, the real surface position is rebuilt from the headset's
// environment depth (AROcclusionManager + ARShaderOcclusion: _EnvironmentDepthTexture and its reprojection
// matrices, the same inputs as the MR template occlusion shaders), and the pixel lights up where that surface is
// at the wave's current radius.
Shader "SteelArm/ScanWave"
{
    Properties
    {
        _Color ("Color", Color) = (0.45, 0.85, 1, 1)
        _BandWidth ("Wave Front Width (m)", Float) = 0.06
        _TrailLength ("Trail Length (m)", Float) = 0.35
        _Intensity ("Intensity", Range(0, 1)) = 0.8
    }

    SubShader
    {
        Tags
        {
            "RenderType" = "Transparent"
            "Queue" = "Transparent+100"
            "RenderPipeline" = "UniversalPipeline"
            "IgnoreProjector" = "True"
        }

        Pass
        {
            Name "ScanWave"
            Tags { "LightMode" = "UniversalForward" }

            // Alpha is written too, so the glow shows over passthrough.
            Blend SrcAlpha OneMinusSrcAlpha, One OneMinusSrcAlpha
            ZWrite Off
            ZTest Always
            Cull Off

            HLSLPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #pragma multi_compile_instancing
            #pragma multi_compile _ XR_LINEAR_DEPTH

            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.xr.arfoundation/Assets/Shaders/Utils.hlsl"

            TEXTURE2D_ARRAY_FLOAT(_EnvironmentDepthTexture);
            SAMPLER(sampler_EnvironmentDepthTexture);
            float4x4 _EnvironmentDepthProjectionMatrices[2];
            int _IsOcclusionOn;

            CBUFFER_START(UnityPerMaterial)
                float4 _Color;
                float _BandWidth;
                float _TrailLength;
                float _Intensity;
            CBUFFER_END

            // Up to 4 waves at once: xyz = origin (world), w = current radius (<= 0: unused). Fade 0..1 per wave.
            float4 _WaveOrigins[4];
            float _WaveFade[4];

            struct Attributes
            {
                float4 positionOS : POSITION;
                UNITY_VERTEX_INPUT_INSTANCE_ID
            };

            struct Varyings
            {
                float4 positionCS : SV_POSITION;
                float3 positionWS : TEXCOORD0;
                UNITY_VERTEX_INPUT_INSTANCE_ID
                UNITY_VERTEX_OUTPUT_STEREO
            };

            Varyings vert(Attributes input)
            {
                Varyings output = (Varyings)0;
                UNITY_SETUP_INSTANCE_ID(input);
                UNITY_TRANSFER_INSTANCE_ID(input, output);
                UNITY_INITIALIZE_VERTEX_OUTPUT_STEREO(output);
                output.positionWS = TransformObjectToWorld(input.positionOS.xyz);
                output.positionCS = TransformWorldToHClip(output.positionWS);
                return output;
            }

            // World position -> environment depth texture uv. Returns 1 when inside the depth camera frustum.
            float ProjectToDepthUV(float3 positionWS, out float2 uv)
            {
                float4 cs = mul(_EnvironmentDepthProjectionMatrices[unity_StereoEyeIndex], float4(positionWS, 1.0));
                uv = (cs.xy / cs.w + 1.0) * 0.5;
                return (all(uv >= 0.0) && all(uv <= 1.0) && cs.w > 0.0) ? 1.0 : 0.0;
            }

            float SampleEnvironmentDepthLinear(float2 uv)
            {
                float d = SAMPLE_TEXTURE2D_ARRAY_LOD(_EnvironmentDepthTexture, sampler_EnvironmentDepthTexture, uv, unity_StereoEyeIndex, 0).r;
            #ifdef XR_LINEAR_DEPTH
                return d;
            #else
                return LinearizeDepth(ConvertDepthToSymmetricRange(d));
            #endif
            }

            half4 frag(Varyings input) : SV_Target
            {
                UNITY_SETUP_INSTANCE_ID(input);
                UNITY_SETUP_STEREO_EYE_INDEX_POST_VERTEX(input);

                if (_IsOcclusionOn == 0)
                    return half4(0, 0, 0, 0);

                // The real surface seen through this pixel.
                float3 cameraPos = GetCameraPositionWS();
                float3 rayDir = normalize(input.positionWS - cameraPos);
                float3 cameraForward = -UNITY_MATRIX_V[2].xyz;
                float2 uv;
                float valid = ProjectToDepthUV(input.positionWS, uv);
                float viewDepth = SampleEnvironmentDepthLinear(saturate(uv));
                float3 surface = cameraPos + rayDir * (viewDepth / max(dot(rayDir, cameraForward), 0.1));
                valid *= (viewDepth > 0.05 && viewDepth < 20.0) ? 1.0 : 0.0;

                // Bright thin front at the wave radius, softer trail behind it.
                float glow = 0.0;
                [unroll] for (int k = 0; k < 4; k++)
                {
                    float4 wave = _WaveOrigins[k];
                    float used = wave.w > 0.0 ? 1.0 : 0.0;
                    float behind = wave.w - distance(surface, wave.xyz); // > 0: the front has already passed
                    float front = 1.0 - smoothstep(0.0, _BandWidth, abs(behind));
                    float trail = behind > 0.0 ? exp(-behind / _TrailLength) * 0.35 : 0.0;
                    glow += (front + trail) * _WaveFade[k] * used;
                }

                float alpha = saturate(glow * _Intensity) * valid;
                clip(alpha - 0.002);
                return half4(_Color.rgb, alpha);
            }
            ENDHLSL
        }
    }
}
