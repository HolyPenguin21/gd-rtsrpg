Shader "GameDev/Camera Preview"
{
    Properties
    {
        _BaseColor("Color", Color) = (0.18, 0.25, 0.2, 1)
        _GridColor("Grid color", Color) = (0.45, 0.55, 0.45, 1)
        _GridSize("Grid spacing (preview only)", Float) = 5
        _ShowGrid("Show grid", Float) = 1
    }
    SubShader
    {
        Tags { "RenderType"="Opaque" "RenderPipeline"="UniversalPipeline" }
        Pass
        {
            Name "CameraPreview"
            Tags { "LightMode"="SRPDefaultUnlit" }
            HLSLPROGRAM
            #pragma vertex Vert
            #pragma fragment Frag
            #include "Packages/com.unity.render-pipelines.universal/ShaderLibrary/Core.hlsl"

            CBUFFER_START(UnityPerMaterial)
                half4 _BaseColor;
                half4 _GridColor;
                float _GridSize;
                float _ShowGrid;
            CBUFFER_END

            struct Attributes { float4 positionOS : POSITION; };
            struct Varyings { float4 positionCS : SV_POSITION; float3 positionWS : TEXCOORD0; };
            Varyings Vert(Attributes input)
            {
                Varyings output;
                output.positionWS = TransformObjectToWorld(input.positionOS.xyz);
                output.positionCS = TransformWorldToHClip(output.positionWS);
                return output;
            }
            half4 Frag(Varyings input) : SV_Target
            {
                float2 cell = input.positionWS.xz / max(_GridSize, 0.01);
                float2 width = max(fwidth(cell), 0.0001);
                float2 lineDistance = abs(frac(cell + 0.5) - 0.5) / width;
                float grid = 1.0 - saturate(min(lineDistance.x, lineDistance.y));
                float checker = fmod(abs(floor(cell.x) + floor(cell.y)), 2.0);
                half4 background = _BaseColor * lerp(1.0, 1.12, checker * saturate(_ShowGrid));
                return lerp(background, _GridColor, grid * saturate(_ShowGrid));
            }
            ENDHLSL
        }
    }
}
