#version 330 core
out vec4 FragColor;

struct Material {
    sampler2D diffuseSampler;
    sampler2D specularSampler;
    sampler2D normalSampler;
    float shininess;
}; 

struct Light {
    vec3 dir;
    vec3 color; // this is I_d (I_s = I_d, I_a = 0.3 * I_d)
};

uniform vec3 viewPos;
uniform Material material;
uniform Light light;

in vec2 TexCoord;
in vec3 FragPos;
in vec3 Normal;
in mat3 TBN;
in vec4 FragPosLightSpace;
in vec3 VertexColor;

uniform sampler2D shadowMap;

uniform float useNormalMap;
uniform float useSpecularMap;
uniform float useShadow;
uniform float useLighting;

uniform float debugThickness;



void main()
{
    vec3 color = texture(material.diffuseSampler, TexCoord).rgb;

    // on-off by key 3 (useLighting). 
    // if useLighting is 0, return diffuse value without considering any lighting.(DO NOT CHANGE)
    if (useLighting < 0.5f){
        FragColor = vec4(color, 1.0); 
        return; 
    }

    vec3 normal = normalize(Normal);
    vec3 lightDir = normalize(-light.dir);

    if(useNormalMap > 0.5f)
    {
        normal = texture(material.normalSampler, TexCoord).rgb;
        normal = normalize(normal * 2.0 - 1.0);
        normal = normalize(TBN * normal);
    }

    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = diff * light.color * color; 
    vec3 ambient = 0.3 * light.color * color;

    vec3 specular = vec3(0.0);

    if(useSpecularMap > 0.5f)
    {
        float k_s = texture(material.specularSampler, TexCoord).r;
        
        vec3 viewDir = normalize(viewPos - FragPos);
        vec3 halfwayDir = normalize(lightDir + viewDir);  
        float spec = pow(max(dot(normal, halfwayDir), 0.0), material.shininess);
        
        specular = light.color * spec * k_s;
    }
    
    vec3 result;
    
    if (debugThickness > 0.5f) {
        // 1. 비눗방울 렌더링 (PBD 간섭색 적용)
        float lightInfluence = 0.5;
        
        // 텍스처(color) 대신 VertexColor에 조명 밝기(diff * light.color)만 곱합니다.
        vec3 bubbleDiffuse = VertexColor * diff * light.color; 
        
        // 본래 색상(VertexColor)과 조명 받은 색상을 섞고 하이라이트를 더합니다.
        result = mix(VertexColor, bubbleDiffuse, lightInfluence) + specular;
    } else {
        // 2. 일반 물체 렌더링 (기존 방식)
        result = ambient + diffuse + specular;
    }

    // 최종 출력
    FragColor = vec4(result, 1.0);

}