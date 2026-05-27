#define GLM_ENABLE_EXPERIMENTAL
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "shader.h"
#include "opengl_utils.h"
#include "geometry_primitives.h"
#include <iostream>
#include <vector>
#include "camera.h"
#include "texture.h"
#include "texture_cube.h"
#include "model.h"
#include "mesh.h"
#include "scene.h"
#include "math_utils.h"
#include "light.h"
#include "pbd.h"

Mesh createHighResolutionSphereMesh(unsigned int rings, unsigned int segments);
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow* window, DirectionalLight* sun);

bool isWindowed = true;
bool isKeyboardDone[1024] = { 0 };

// setting
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;
const unsigned int SHADOW_WIDTH = 1024;
const unsigned int SHADOW_HEIGHT = 1024;
const float planeSize = 15.f;

// camera
Camera camera(glm::vec3(-1.0f, 1.0f, 3.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

bool useNormalMap = true;
bool useSpecular = false;
bool useShadow = true;
bool useLighting = true;

bool usePCF = true;
bool showWireframe = false;

//float pbdStretchStiffness = 0.85f;
//float pbdBendStiffness = 0.15f;
//float pbdDamping = 0.015f;
//int pbdSolverIterations = 12;
float pbdStretchStiffness = 0.70f;
float pbdBendStiffness = 0.08f;
float pbdDamping = 0.005f;
int pbdSolverIterations = 4;
unsigned int thicknessDisturbanceVertex = 1000;


int main()
{
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // glfw window creation
    // --------------------
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Soap Bubble Simulator", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    // tell GLFW to capture our mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    // configure global opengl state
    // -----------------------------
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // build and compile our shader program
    // ------------------------------------
    Shader lightingShader("../shaders/shader_lighting.vs", "../shaders/shader_lighting.fs"); // you can name your shader files however you like
    Shader shadowShader("../shaders/shadow.vs", "../shaders/shadow.fs");
    Shader skyboxShader("../shaders/shader_skybox.vs", "../shaders/shader_skybox.fs");
    Shader wireframeShader("../shaders/wireframe.vs", "../shaders/wireframe.fs");


    Model yourOwnModel = Model("../resources/myobj/sphere.obj");
    // Use a denser sphere here so the soap-film deformation stays smooth.
    yourOwnModel.mesh = createHighResolutionSphereMesh(64, 128);
    yourOwnModel.VAO = yourOwnModel.mesh.VAO; 
    PBDSolver* spherePBD = nullptr;
    spherePBD = new PBDSolver(&yourOwnModel.mesh);
    spherePBD->initialize();




    // Add entities to scene.
    // you can change the position/orientation.
    Scene scene;

    // add your model's entity here!
    Entity* sphereEntity = new Entity(&yourOwnModel, glm::vec3(-1, 1, -1), 0.0f, 0.0f, 0.0f, 1);
    scene.addEntity(sphereEntity);

    // define depth texture
    DepthMapTexture depth = DepthMapTexture(SHADOW_WIDTH, SHADOW_HEIGHT);


    // skybox
    std::vector<std::string> faces
    {
        "../resources/skybox/right.jpg",
        "../resources/skybox/left.jpg",
        "../resources/skybox/top.jpg",
        "../resources/skybox/bottom.jpg",
        "../resources/skybox/front.jpg",
        "../resources/skybox/back.jpg"
    };
    CubemapTexture skyboxTexture = CubemapTexture(faces);
    ThinFilmLUTTexture thinFilmLUT(256, 128);
    unsigned int VAOskybox, VBOskybox;
    getPositionVAO(skybox_positions, sizeof(skybox_positions), VAOskybox, VBOskybox);


    lightingShader.use();
    lightingShader.setInt("material.diffuseSampler", 0);
    lightingShader.setInt("material.specularSampler", 1);
    lightingShader.setInt("material.normalSampler", 2);
    lightingShader.setInt("depthMapSampler", 3);
    lightingShader.setInt("skyboxTexture", 4);
    lightingShader.setInt("thinFilmLUT", 5);
    lightingShader.setFloat("material.shininess", 64.f);    // set shininess to constant value.


    skyboxShader.use();
    skyboxShader.setInt("skyboxTexture1", 0);


    DirectionalLight sun(30.0f, 30.0f, glm::vec3(0.8f));
    sun.updateLightDir();

    // timing setup for fixed timestep
    float oldTime = glfwGetTime();
    float accumulator = 0.0f;
    const float FIXED_DT = 1.0f / 60.0f;
    const float MAX_DT = 0.1f;

    while (!glfwWindowShouldClose(window))
    {
        // SPACE: trigger one-shot wave on key edge
        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS && !isKeyboardDone[GLFW_KEY_SPACE]) {
            isKeyboardDone[GLFW_KEY_SPACE] = true;
            if (spherePBD) {
                // a single press starts a slow zero-gravity droplet wobble
                spherePBD->startWave(thicknessDisturbanceVertex, 0.16f, 1.2f, 7.0f);
            }
        }
        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_RELEASE) {
            isKeyboardDone[GLFW_KEY_SPACE] = false;
        }
        if (glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS && !isKeyboardDone[GLFW_KEY_T]) {
            spherePBD->injectThicknessDisturbance(thicknessDisturbanceVertex, 0.12f, 0.95f);
            thicknessDisturbanceVertex = (thicknessDisturbanceVertex + 137u) % yourOwnModel.mesh.vertices.size();
            isKeyboardDone[GLFW_KEY_T] = true;
        }
        if (glfwGetKey(window, GLFW_KEY_T) == GLFW_RELEASE) {
            isKeyboardDone[GLFW_KEY_T] = false;
        }

        float currentTime = glfwGetTime();
        float dt = currentTime - oldTime;
        oldTime = currentTime;

        if (dt > MAX_DT) {
            dt = MAX_DT;
        }

        deltaTime = dt;
        accumulator += dt;

        processInput(window, &sun);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		//glm::vec3 gravity = glm::vec3(0.0f, -9.81f, 0.0f);
        glm::vec3 gravity = glm::vec3(0.0f, 0.0f, 0.0f);
        const int SUB_STEPS = 2;

        while (accumulator >= FIXED_DT) {
            float subDt = FIXED_DT / SUB_STEPS;
            if (spherePBD) {
                spherePBD->setStretchStiffness(pbdStretchStiffness);
                spherePBD->setBendStiffness(pbdBendStiffness);
                for (int i = 0; i < SUB_STEPS; i++) {
                    spherePBD->step(subDt, pbdSolverIterations, gravity, pbdDamping);
                }
            }
            accumulator -= FIXED_DT;
        }

        if (spherePBD) {
            spherePBD->applyToMesh();
        }


        glm::mat4 lightProjection = sun.getProjectionMatrix();
        glm::mat4 lightView = sun.getViewMatrix(camera.Position);
        glm::mat4 lightSpaceMatrix = lightProjection * lightView;

        shadowShader.use();
        shadowShader.setMat4("lightSpaceMatrix", lightSpaceMatrix);

        glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
        glBindFramebuffer(GL_FRAMEBUFFER, depth.depthMapFBO);
        glClear(GL_DEPTH_BUFFER_BIT);

        for (map<Model*, vector<Entity*>>::iterator it = scene.entities.begin(); it != scene.entities.end(); ++it) {
            Model* model = it->first;
            for (Entity* entity : it->second) {
                shadowShader.setMat4("model", entity->getModelMatrix());
                model->bind();
                glDrawElements(GL_TRIANGLES, model->mesh.indices.size(), GL_UNSIGNED_INT, 0);
            }
        }

        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        glViewport(0, 0, SCR_WIDTH, SCR_HEIGHT);

        lightingShader.use();

        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        glm::mat4 view = camera.GetViewMatrix();
        lightingShader.setMat4("projection", projection);
        lightingShader.setMat4("view", view);

        lightingShader.setVec3("viewPos", camera.Position);

        if (glm::length(sun.lightDir) < 0.1f) {
            sun.lightDir = glm::normalize(glm::vec3(0.5f, -1.0f, 0.3f));
        }
        lightingShader.setVec3("light.dir", sun.lightDir);
        lightingShader.setVec3("light.color", sun.lightColor);

        lightingShader.setFloat("useLighting", useLighting ? 1.0f : 0.0f);
        lightingShader.setFloat("useShadow", useShadow ? 1.0f : 0.0f);
        lightingShader.setFloat("filmTime", currentTime);

        lightingShader.setMat4("lightSpaceMatrix", lightSpaceMatrix);
        lightingShader.setInt("shadowMap", 3);

        glActiveTexture(GL_TEXTURE3);
        glBindTexture(GL_TEXTURE_2D, depth.ID);

        for (map<Model*, vector<Entity*>>::iterator it = scene.entities.begin(); it != scene.entities.end(); ++it) {
            Model* model = it->first;

            if (model == &yourOwnModel) continue;

            bool hasNormal = (model->normal != NULL);
            lightingShader.setFloat("useNormalMap", (useNormalMap && hasNormal) ? 1.0f : 0.0f);

            bool hasSpecular = (model->specular != NULL);
            lightingShader.setFloat("useSpecularMap", (useSpecular && hasSpecular) ? 1.0f : 0.0f);

            lightingShader.setFloat("usePCF", usePCF ? 1.0f : 0.0f);

            for (Entity* entity : it->second) {
                lightingShader.setMat4("world", entity->getModelMatrix());
                model->bind();
                glDrawElements(GL_TRIANGLES, model->mesh.indices.size(), GL_UNSIGNED_INT, 0);
            }
        }

        // use skybox Shader
        skyboxShader.use();
        glDepthFunc(GL_LEQUAL);
        view = glm::mat4(glm::mat3(camera.GetViewMatrix()));
        skyboxShader.setMat4("view", view);
        skyboxShader.setMat4("projection", projection);

        // render a skybox
        glBindVertexArray(VAOskybox);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_CUBE_MAP, skyboxTexture.textureID);
        glDrawArrays(GL_TRIANGLES, 0, 36);

        glBindVertexArray(0);
        glDepthFunc(GL_LESS);

        lightingShader.use();
        lightingShader.setFloat("filmTime", currentTime);
        lightingShader.setFloat("debugThickness", 1.0f);
        lightingShader.setFloat("useNormalMap", 0.0f);
        lightingShader.setFloat("useSpecularMap", 0.0f);
        lightingShader.setFloat("useShadow", 0.0f);
        lightingShader.setFloat("filmThicknessScale", 9000.0f);
        lightingShader.setFloat("filmRefractiveIndex", 1.34f);
        lightingShader.setFloat("filmAlpha", 0.18f);
        lightingShader.setFloat("filmR0", 0.025f);
        lightingShader.setFloat("filmDeltaMax", 1200.0f);
        lightingShader.setFloat("filmIridescenceStrength", 2.25f);
        lightingShader.setFloat("filmRefractionStrength", 0.82f);
        lightingShader.setFloat("filmFresnelStrength", 1.0f);
        lightingShader.setFloat("filmReflectionIntensity", 0.85f);
        lightingShader.setFloat("filmRoughness", 0.16f);

        glActiveTexture(GL_TEXTURE4);
        glBindTexture(GL_TEXTURE_CUBE_MAP, skyboxTexture.textureID);
        glActiveTexture(GL_TEXTURE5);
        glBindTexture(GL_TEXTURE_2D, thinFilmLUT.ID);

        glDisable(GL_CULL_FACE);
        // Draw the translucent film without writing depth, so the bubble stays visually layered.
        glDepthMask(GL_FALSE);

        auto itSpherePBD = scene.entities.find(&yourOwnModel);
        if (itSpherePBD != scene.entities.end()) {
            for (Entity* entity : itSpherePBD->second) {
                lightingShader.setMat4("world", entity->getModelMatrix());
                yourOwnModel.bind();
                glDrawElements(GL_TRIANGLES, yourOwnModel.mesh.indices.size(), GL_UNSIGNED_INT, 0);
            }
        }

        glDepthMask(GL_TRUE);
        lightingShader.setFloat("debugThickness", 0.0f);
        lightingShader.setFloat("useShadow", useShadow ? 1.0f : 0.0f);

        if (showWireframe) {
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glDisable(GL_CULL_FACE);
            glLineWidth(1.0f);

            wireframeShader.use();
            wireframeShader.setMat4("projection", projection);
            wireframeShader.setMat4("view", view);

            auto itSphere = scene.entities.find(&yourOwnModel);
            if (itSphere != scene.entities.end()) {
                for (Entity* entity : itSphere->second) {
                    wireframeShader.setMat4("model", entity->getModelMatrix());
                    yourOwnModel.bind();
                    glDrawElements(GL_TRIANGLES, yourOwnModel.mesh.indices.size(), GL_UNSIGNED_INT, 0);
                }
            }

            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // optional: de-allocate all resources once they've outlived their purpose:
    // ------------------------------------------------------------------------

    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwTerminate();
    return 0;
}

Mesh createHighResolutionSphereMesh(unsigned int rings, unsigned int segments)
{
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
    vertices.reserve((rings + 1) * (segments + 1));
    indices.reserve(rings * segments * 6);

    for (unsigned int y = 0; y <= rings; ++y) {
        float v = static_cast<float>(y) / static_cast<float>(rings);
        float theta = v * glm::pi<float>();
        float sinTheta = std::sin(theta);
        float cosTheta = std::cos(theta);

        for (unsigned int x = 0; x <= segments; ++x) {
            float u = static_cast<float>(x) / static_cast<float>(segments);
            float phi = u * glm::two_pi<float>();

            glm::vec3 position(
                sinTheta * std::cos(phi),
                cosTheta,
                sinTheta * std::sin(phi)
            );

            Vertex vertex;
            vertex.Position = position;
            vertex.Normal = glm::normalize(position);
            vertex.TexCoords = glm::vec2(u, v);
            vertex.Tangent = glm::normalize(glm::vec3(-std::sin(phi), 0.0f, std::cos(phi)));
            vertex.Color = glm::vec3(1.0f);
            vertex.Thickness = 0.05f;
            vertices.push_back(vertex);
        }
    }

    for (unsigned int y = 0; y < rings; ++y) {
        for (unsigned int x = 0; x < segments; ++x) {
            unsigned int i0 = y * (segments + 1) + x;
            unsigned int i1 = i0 + 1;
            unsigned int i2 = (y + 1) * (segments + 1) + x;
            unsigned int i3 = i2 + 1;

            indices.push_back(i0);
            indices.push_back(i2);
            indices.push_back(i1);

            indices.push_back(i1);
            indices.push_back(i2);
            indices.push_back(i3);
        }
    }

    return Mesh(vertices, indices);
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow* window, DirectionalLight* sun)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);

    if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS && !isKeyboardDone[GLFW_KEY_F]) {
        showWireframe = !showWireframe;
        isKeyboardDone[GLFW_KEY_F] = true;
    }
    if (glfwGetKey(window, GLFW_KEY_F) == GLFW_RELEASE) {
        isKeyboardDone[GLFW_KEY_F] = false;
    }

    if (glfwGetKey(window, GLFW_KEY_LEFT_BRACKET) == GLFW_PRESS) {
        pbdStretchStiffness = MAX(0.0f, pbdStretchStiffness - deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_RIGHT_BRACKET) == GLFW_PRESS) {
        pbdStretchStiffness = MIN(1.0f, pbdStretchStiffness + deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_SEMICOLON) == GLFW_PRESS) {
        pbdDamping = MAX(0.0f, pbdDamping - 0.25f * deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_APOSTROPHE) == GLFW_PRESS) {
        pbdDamping = MIN(0.2f, pbdDamping + 0.25f * deltaTime);
    }
    if (glfwGetKey(window, GLFW_KEY_COMMA) == GLFW_PRESS && !isKeyboardDone[GLFW_KEY_COMMA]) {
        pbdSolverIterations = MAX(1, pbdSolverIterations - 1);
        isKeyboardDone[GLFW_KEY_COMMA] = true;
    }
    if (glfwGetKey(window, GLFW_KEY_COMMA) == GLFW_RELEASE) {
        isKeyboardDone[GLFW_KEY_COMMA] = false;
    }
    if (glfwGetKey(window, GLFW_KEY_PERIOD) == GLFW_PRESS && !isKeyboardDone[GLFW_KEY_PERIOD]) {
        pbdSolverIterations = MIN(64, pbdSolverIterations + 1);
        isKeyboardDone[GLFW_KEY_PERIOD] = true;
    }
    if (glfwGetKey(window, GLFW_KEY_PERIOD) == GLFW_RELEASE) {
        isKeyboardDone[GLFW_KEY_PERIOD] = false;
    }

    float t = 20.0f * deltaTime;
    

    bool lightChanged = false;
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
        sun->elevation += t;
        if (sun->elevation > 80.0f) sun->elevation = 80.0f;
        lightChanged = true;
    }
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
        sun->elevation -= t;
        if (sun->elevation < 15.0f) sun->elevation = 15.0f;
        lightChanged = true;
    }
    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
        sun->azimuth += t;
        lightChanged = true;
    }
    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
        sun->azimuth -= t;
        lightChanged = true;
    }

    if (lightChanged) {
        sun->updateLightDir();
    }

}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos;

    lastX = xpos;
    lastY = ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(yoffset);
}
