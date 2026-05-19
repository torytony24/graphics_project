#ifndef MODEL_H
#define MODEL_H

#include <glad/glad.h> 

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include "mesh.h"
#include "shader.h"

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include <vector>
using namespace std;



class Model 
{
public:
    /*  Model Data */
    Mesh mesh;
    Texture* diffuse;
    Texture* normal;
    Texture* specular;
    unsigned int VAO;
    bool ignoreShadow = false;

    Model() {}
    /*  Functions   */
    // constructor, expects a filepath to a 3D model.
    Model(string const &path):diffuse(NULL), normal(NULL), specular(NULL){
        loadModel(path);
        this->VAO = mesh.VAO;
    }

    void bind() {
        glBindVertexArray(mesh.VAO);

        glActiveTexture(GL_TEXTURE0);
        if (this->diffuse) glBindTexture(GL_TEXTURE_2D, diffuse->ID);
        else glBindTexture(GL_TEXTURE_2D, 0);

        glActiveTexture(GL_TEXTURE1);
        if (this->specular) glBindTexture(GL_TEXTURE_2D, specular->ID);
        else glBindTexture(GL_TEXTURE_2D, 0);

        glActiveTexture(GL_TEXTURE2);
        if (this->normal) glBindTexture(GL_TEXTURE_2D, normal->ID);
        else glBindTexture(GL_TEXTURE_2D, 0);
    }
    
private:
    /*  Functions   */
    // loads a model with supported ASSIMP extensions from file and stores the resulting meshes in the meshes vector.
    void loadModel(string const &path)
    {
        // read file via ASSIMP
        Assimp::Importer importer;

        // TODO : to get additional 3 points, DO NOT use aiProcess_CalcTangentSpace!
        const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | aiProcess_GenNormals);
        
        // check for errors
        if(!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) // if is Not Zero
        {
            cout << "ERROR::ASSIMP:: " << importer.GetErrorString() << endl;
            return;
        }

        // 안전하게 첫 번째 메쉬가 존재하는 노드를 찾음
        aiNode* startNode = scene->mRootNode;
        if (startNode->mNumMeshes == 0) {
            // 루트에 메쉬가 없을 수 있으므로 자식들 중 메쉬를 가진 첫 노드를 찾음
            aiNode* found = nullptr;
            for (unsigned int i = 0; i < startNode->mNumChildren; ++i) {
                aiNode* child = startNode->mChildren[i];
                if (child && child->mNumMeshes > 0) {
                    found = child;
                    break;
                }
            }
            // 못 찾으면 루트 그대로 사용 (그래도 mNumMeshes == 0이면 이후 체크에서 안전하게 처리됨)
            if (found) startNode = found;
        }

        // process the chosen node (if it has meshes)
        processNode(startNode, scene);
    }

    // Original code : processes a node in a recursive fashion. Processes each individual mesh located at the node and repeats this process on its children nodes (if any).
    // modified version : only process FIRST item of obj file. Do not consider texture or material.
    void processNode(aiNode *node, const aiScene *scene)
    {
        if (!node) return;
        if (node->mNumMeshes == 0) {
            cout << "WARNING: chosen node contains no meshes." << endl;
            return;
        }
        // ensure the mesh index exists
        unsigned int meshIndex = node->mMeshes[0];
        if (meshIndex >= scene->mNumMeshes) {
            cout << "ERROR: mesh index out of range." << endl;
            return;
        }

        aiMesh* mesh = scene->mMeshes[meshIndex];
        this->mesh = processMesh(mesh, scene);
    }

    Mesh processMesh(aiMesh *mesh, const aiScene *scene)
    {
        // data to fill
        vector<Vertex> vertices;
        vector<unsigned int> indices;
        vector<Texture> textures;

        // Walk through each of the mesh's vertices
        for(unsigned int i = 0; i < mesh->mNumVertices; i++)
        {
            Vertex vertex;
            glm::vec3 vector; // we declare a placeholder vector since assimp uses its own vector class that doesn't directly convert to glm's vec3 class so we transfer the data to this placeholder glm::vec3 first.
            // positions
            vector.x = mesh->mVertices[i].x;
            vector.y = mesh->mVertices[i].y;
            vector.z = mesh->mVertices[i].z;
            vertex.Position = vector;

            // normals
            vector.x = mesh->mNormals[i].x;
            vector.y = mesh->mNormals[i].y;
            vector.z = mesh->mNormals[i].z;
            vertex.Normal = vector;

            // texture coordinates
            if(mesh->mTextureCoords[0]) // does the mesh contain texture coordinates?
            {
                glm::vec2 vec;
                // a vertex can contain up to 8 different texture coordinates. We thus make the assumption that we won't 
                // use models where a vertex can have multiple texture coordinates so we always take the first set (0).
                vec.x = mesh->mTextureCoords[0][i].x; 
                vec.y = mesh->mTextureCoords[0][i].y;
                vertex.TexCoords = vec;
            }
            else
                vertex.TexCoords = glm::vec2(0.0f, 0.0f);
            // tangent
            // TODO : to get additional 3 points, DO NOT use mTangents directly!
            glm::vec3 n = glm::normalize(vertex.Normal);

            glm::vec3 c1 = glm::cross(n, glm::vec3(0.0f, 0.0f, 1.0f));
            glm::vec3 c2 = glm::cross(n, glm::vec3(0.0f, 1.0f, 0.0f));

            if (glm::length(c1) > glm::length(c2)) {
                vector = glm::normalize(c1);
            }
            else {
                vector = glm::normalize(c2);
            }

            vertex.Tangent = vector;
            vertices.push_back(vertex);


        }
        // now walk through each of the mesh's faces (a face is a mesh its triangle) and retrieve the corresponding vertex indices.
        for(unsigned int i = 0; i < mesh->mNumFaces; i++)
        {
            aiFace face = mesh->mFaces[i];
            // retrieve all indices of the face and store them in the indices vector
            for(unsigned int j = 0; j < face.mNumIndices; j++)
                indices.push_back(face.mIndices[j]);
        }
        
        // return a mesh object created from the extracted mesh data
        return Mesh(vertices, indices);
    }
};
class Entity {
public:
    Model* model;
    glm::mat4 modelMatrix;
    Entity(Model* model, glm::mat4 modelMatrix) {
        this->model = model;
        this->modelMatrix = modelMatrix;
    }

    Entity(Model* model, glm::vec3 position, float rotX, float rotY, float rotZ, float scale) {
        this->model = model;
        glm::mat4 transform = glm::mat4(1.0f);
        transform = glm::translate(transform, position);
        transform = glm::rotate(transform, glm::radians(rotX), glm::vec3(1.0f, 0.0f, 0.0f));
        transform = glm::rotate(transform, glm::radians(rotY), glm::vec3(0.0f, 1.0f, 0.0f));
        transform = glm::rotate(transform, glm::radians(rotZ), glm::vec3(0.0f, 0.0f, 1.0f));
        transform = glm::scale(transform, glm::vec3(scale));

        this->modelMatrix = transform;
    }

    glm::mat4 getModelMatrix() {
        return this->modelMatrix;
    }
};

#endif
