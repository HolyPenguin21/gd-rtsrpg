Shader "GameDev/Terrain Surface"
{
    Properties
    {
        [NoScaleOffset] _SurfaceMap("Surface albedo", 2D) = "white" {}
        [NoScaleOffset] _EarthMap("Exposed earth albedo", 2D) = "white" {}
        _Tint("Surface tint", Color) = (1, 1, 1, 1)
        _TilesPerUnit("Texture repeats per world unit", Float) = 0.25
        [Enum(Plateau,0,Cliff,1,Ramp,2)] _SurfaceType("Surface type", Float) = 0
        _EdgeWidth("Grass setback (fraction of a terrain cell)", Range(0.2, 0.72)) = 0.64
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" "RenderPipeline"="UniversalPipeline" }
        Pass
        {
            Name "TerrainSurface"
            Tags { "LightMode"="UniversalForward" }
            HLSLPROGRAM
            #pragma vertex Vert
            #pragma fragment Frag
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Lighting.hlsl"
            TEXTURE2D(_SurfaceMap);
            SAMPLER(sampler_SurfaceMap);
            TEXTURE2D(_EarthMap);
            SAMPLER(sampler_EarthMap);
            CBUFFER_START(UnityPerMaterial)
                half4 _Tint;
                float _TilesPerUnit;
                float _SurfaceType;
                float _EdgeWidth;
            CBUFFER_END
            struct Attributes
            {
                float4 positionOS : POSITION;
                float3 normalOS : NORMAL;
                float2 edgeMask : TEXCOORD0;
            };
            struct Varyings
            {
                float4 positionCS : SV_POSITION;
                float3 positionWS : TEXCOORD0;
                float3 normalWS : TEXCOORD1;
                float2 edgeMask : TEXCOORD2;
            };
            Varyings Vert(Attributes input)
            {
                Varyings output;
                output.positionWS = TransformObjectToWorld(input.positionOS.xyz);
                output.positionCS = TransformWorldToHClip(output.positionWS);
                output.normalWS = TransformObjectToWorldNormal(input.normalOS);
                output.edgeMask = input.edgeMask;
                return output;
            }
            half3 SampleRock(float3 p, half3 normal)
            {
                half3 weights = abs(normal);
                weights *= weights;
                weights /= weights.x + weights.y + weights.z;
                return SAMPLE_TEXTURE2D(_SurfaceMap, sampler_SurfaceMap, p.zy).rgb * weights.x
                     + SAMPLE_TEXTURE2D(_SurfaceMap, sampler_SurfaceMap, p.xz).rgb * weights.y
                     + SAMPLE_TEXTURE2D(_SurfaceMap, sampler_SurfaceMap, p.xy).rgb * weights.z;
            }

            float3 GroundRelief(float3 position, float3 normal, float height)
            {
                // Surface gradient in world space, so cliff projections need no mesh tangents.
                float3 dpdx = ddx(position);
                float3 dpdy = ddy(position);
                float3 tangentX = cross(dpdy, normal);
                float3 tangentY = cross(normal, dpdx);
                float determinant = dot(dpdx, tangentX);
                float3 gradient = sign(determinant) * (ddx(height) * tangentX + ddy(height) * tangentY);
                gradient /= max(abs(determinant), 1e-8);
                return normalize(normal - gradient / max(1.0, length(gradient) * 2.0));
            }

            half4 Frag(Varyings input) : SV_Target
            {
                half3 normal = normalize(input.normalWS);
                float3 p = input.positionWS * _TilesPerUnit;
                half3 albedo;
                if (_SurfaceType == 1)
                {
                    half3 surface = SampleRock(p, normal);
                    // A second projection scale breaks the repeated stone silhouettes.
                    half3 fine = SampleRock(p * 2.73 + float3(0.43, 0.19, 0.71), normal);
                    half3 coarse = SampleRock(p * 0.37 + float3(0.13, 0.47, 0.29), normal);
                    float fineHeight = dot(fine, float3(0.30, 0.59, 0.11));
                    float relief = 0.19 * dot(surface, float3(0.30, 0.59, 0.11)) + 0.08 * fineHeight;
                    normal = GroundRelief(input.positionWS, normal, relief);
                    half3 rock = lerp(surface, coarse, 0.22) * (0.93 + 0.65 * fineHeight);
                    albedo = rock * (1.0 - 0.24 * input.edgeMask.y);
                }
                else
                {
                    albedo = SAMPLE_TEXTURE2D(_SurfaceMap, sampler_SurfaceMap, p.xz).rgb;
                    if (_SurfaceType == 0)
                    {
                        half3 earth = SAMPLE_TEXTURE2D(_EarthMap, sampler_EarthMap, p.xz).rgb;
                        float distance = 1.0 - input.edgeMask.x;
                        float2 world = input.positionWS.xz;
                        float irregularity = 0.085 * sin(dot(world, float2(1.62, 1.17)))
                                           + 0.045 * sin(dot(world, float2(3.9, -3.1)))
                                           + (earth.r - 0.3) * 0.10;
                        float grass = smoothstep(_EdgeWidth - 0.065, _EdgeWidth + 0.065, distance + irregularity);
                        half3 cliffTop = lerp(earth, albedo, grass);
                        float landing = smoothstep(0.30, 1.0, input.edgeMask.y);
                        albedo = lerp(cliffTop, earth * half3(0.78, 0.73, 0.65), landing);
                    }
                }

                albedo *= _Tint.rgb;
                Light light = GetMainLight();
                half diffuse = saturate(dot(normal, light.direction));
                half3 illumination = max(SampleSH(normal), half3(0.18, 0.18, 0.18)) + light.color * diffuse * 0.7;
                // Stable lower fill keeps unlit cliff faces legible under the demo sun.
                illumination *= 0.72 + 0.28 * saturate(normal.y);
                return half4(albedo * illumination, 1);
            }
            ENDHLSL
        }
    }
}
