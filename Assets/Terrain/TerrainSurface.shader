Shader "GameDev/Terrain Surface"
{
    Properties
    {
        _BaseColor("Base color", Color) = (0.29, 0.36, 0.20, 1)
        _AccentColor("Mineral / earth color", Color) = (0.39, 0.38, 0.26, 1)
        _GrainScale("World grain scale", Float) = 5
        _Variation("Variation", Range(0, 1)) = 0.25
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
            CBUFFER_START(UnityPerMaterial)
                half4 _BaseColor;
                half4 _AccentColor;
                float _GrainScale;
                float _Variation;
            CBUFFER_END
            struct Attributes { float4 positionOS : POSITION; float3 normalOS : NORMAL; };
            struct Varyings { float4 positionCS : SV_POSITION; float3 positionWS : TEXCOORD0; float3 normalWS : TEXCOORD1; };
            Varyings Vert(Attributes input)
            {
                Varyings output;
                output.positionWS = TransformObjectToWorld(input.positionOS.xyz);
                output.positionCS = TransformWorldToHClip(output.positionWS);
                output.normalWS = TransformObjectToWorldNormal(input.normalOS);
                return output;
            }
            float TerrainHash(float3 p)
            {
                p = frac(p * 0.1031);
                p += dot(p, p.yzx + 33.33);
                return frac((p.x + p.y) * p.z);
            }
            float TerrainNoise(float3 p)
            {
                float3 i = floor(p), f = frac(p);
                f = f * f * (3.0 - 2.0 * f);
                return lerp(lerp(lerp(TerrainHash(i), TerrainHash(i + float3(1,0,0)), f.x),
                    lerp(TerrainHash(i + float3(0,1,0)), TerrainHash(i + float3(1,1,0)), f.x), f.y),
                    lerp(lerp(TerrainHash(i + float3(0,0,1)), TerrainHash(i + float3(1,0,1)), f.x),
                    lerp(TerrainHash(i + float3(0,1,1)), TerrainHash(i + float3(1,1,1)), f.x), f.y), f.z);
            }
            half4 Frag(Varyings input) : SV_Target
            {
                // Full 3D world noise: the same scale on tops, slopes AND tall rock walls.
                // Surface type comes exclusively from Builder's material slot.
                float broad = TerrainNoise(input.positionWS * 0.65);
                float scale = max(_GrainScale, 0.01);
                float grain = TerrainNoise(input.positionWS * scale);
                float footprint = max(length(ddx(input.positionWS)), length(ddy(input.positionWS))) * scale;
                grain = lerp(grain, 0.5, saturate(footprint));
                half3 albedo = lerp(_BaseColor.rgb, _AccentColor.rgb, broad * 0.65);
                albedo *= 1.0 + (grain - 0.5) * _Variation;
                half3 normal = normalize(input.normalWS);
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
