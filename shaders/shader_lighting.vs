#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aTexCoord;
layout (location = 2) in vec3 aNormal;
layout (location = 3) in vec3 aTangent;
layout (location = 4) in vec3 aColor;
layout (location = 5) in float aThickness;

out vec2 TexCoord;
out vec3 FragPos;
out vec3 Normal;
out mat3 TBN;
out vec4 FragPosLightSpace;
out vec3 VertexColor;
out float FilmThickness;

uniform mat4 world;
uniform mat4 view;
uniform mat4 projection;

uniform float useNormalMap;
uniform mat4 lightSpaceMatrix;

void main()
{
    TexCoord = aTexCoord;
    
    FragPos = vec3(world * vec4(aPos, 1.0));
    
    FragPosLightSpace = lightSpaceMatrix * vec4(FragPos, 1.0);

    mat3 normalMatrix = transpose(inverse(mat3(world)));
    Normal = normalize(normalMatrix * aNormal);

    // on-off by key 1 (useNormalMap).
    // if model does not have a normal map, this should be always 0.
    // if useNormalMap is 0, we use a geometric normal as a surface normal.
    // if useNormalMap is 1, we use a geometric normal altered by normal map as a surface normal.
    if (useNormalMap > 0.5) {

        vec3 T = normalize(normalMatrix * aTangent);
        vec3 N = Normal;
        
        T = normalize(T - dot(T, N) * N);
        
        vec3 B = cross(N, T);
        
        TBN = mat3(T, B, N);
    }

    gl_Position = projection * view * world * vec4(aPos, 1.0f);

    VertexColor = aColor;
    FilmThickness = aThickness;

}
