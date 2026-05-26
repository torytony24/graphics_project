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
in float FilmThickness;

uniform sampler2D shadowMap;
uniform samplerCube skyboxTexture;
uniform sampler2D thinFilmLUT;

uniform float useNormalMap;
uniform float useSpecularMap;
uniform float useShadow;
uniform float useLighting;

uniform float debugThickness;
uniform float filmTime;
uniform float filmThicknessScale;
uniform float filmRefractiveIndex;
uniform float filmAlpha;
uniform float filmR0;
uniform float filmDeltaMax;
uniform float filmIridescenceStrength;
uniform float filmRefractionStrength;
uniform float filmFresnelStrength;
uniform float filmReflectionIntensity;
uniform float filmRoughness;



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
    
    vec3 viewDir = normalize(viewPos - FragPos);
    float cosTheta = clamp(dot(normal, viewDir), 0.0, 1.0);
    float fresnel = schlickFresnel(cosTheta, filmR0);
    vec3 result;
    
    if (debugThickness > 0.5f) {
        float opticalThickness = continuousFilmThickness(FilmThickness, normal, FragPos);
        float hNm = opticalThickness * filmThicknessScale;
        float deltaNm = opticalPathDifference(hNm, cosTheta, filmRefractiveIndex);
        vec3 interferenceRGB = sampleThinFilmLUT(deltaNm, cosTheta);
        vec3 reflectedDir = reflect(-viewDir, normal);
        vec3 envReflection = texture(skyboxTexture, reflectedDir).rgb;
        vec3 refractedDir = refract(-viewDir, normal, 1.0 / filmRefractiveIndex);
        if (length(refractedDir) < 1e-4) {
            refractedDir = reflectedDir;
        }
        vec3 envRefraction = texture(skyboxTexture, refractedDir).rgb;

        vec3 halfDir = normalize(lightDir + viewDir);
        float NoL = clamp(dot(normal, lightDir), 0.0, 1.0);
        float NoV = max(cosTheta, 1e-4);
        float NoH = clamp(dot(normal, halfDir), 0.0, 1.0);
        float LoH = clamp(dot(lightDir, halfDir), 0.0, 1.0);

        float directionalFresnel = schlickFresnel(LoH, filmR0) * filmFresnelStrength;
        float D = distributionGGX(NoH, filmRoughness);
        float V = visibilitySmithGGXCorrelated(NoV, NoL, filmRoughness);
        float cookTorranceSpecular = D * V * NoL;

        float viewFresnel = fresnel * filmFresnelStrength;
        float localIridescence = clamp(cookTorranceSpecular * directionalFresnel * filmIridescenceStrength, 0.0, 3.0);
        float envIridescence = viewFresnel * filmReflectionIntensity;

        vec3 transparentFilm = envRefraction * filmRefractionStrength * (1.0 - clamp(viewFresnel, 0.0, 0.85));
        vec3 neutralReflection = envReflection * viewFresnel * 0.18;
        vec3 iridescentEnv = envReflection * interferenceRGB * envIridescence;
        vec3 iridescentLight = light.color * interferenceRGB * localIridescence;
        result = transparentFilm + neutralReflection + iridescentEnv + iridescentLight;
    } else {
        result = ambient + diffuse + specular;
    }

    float alpha = debugThickness > 0.5f ? clamp(filmAlpha + fresnel * 0.36 * filmFresnelStrength, filmAlpha, 0.70) : 1.0;
    FragColor = vec4(result, alpha);

}
