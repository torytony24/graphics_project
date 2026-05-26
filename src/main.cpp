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

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow* window, DirectionalLight* sun);

bool isWindowed = true;
bool isKeyboardDone[1024] = { 0 };

// setting
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;
const unsigned int SHADOW_WIDTH = 2048;
const unsigned int SHADOW_HEIGHT = 2048;
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

    // build and compile our shader program
    // ------------------------------------
    Shader lightingShader("../shaders/shader_lighting.vs", "../shaders/shader_lighting.fs"); // you can name your shader files however you like
    Shader shadowShader("../shaders/shadow.vs", "../shaders/shadow.fs");
    Shader skyboxShader("../shaders/shader_skybox.vs", "../shaders/shader_skybox.fs");
    Shader wireframeShader("../shaders/wireframe.vs", "../shaders/wireframe.fs");


    Model yourOwnModel = Model("../resources/myobj/sphere2.obj");
    PBDSolver* spherePBD = nullptr;
    spherePBD = new PBDSolver(&yourOwnModel.mesh);
    spherePBD->initialize();

    
    /*
    float maxY = -1e9f;
    for (const auto& v : yourOwnModel.mesh.vertices) {
        if (v.Position.y > maxY) {
            maxY = v.Position.y;
        }
    }

    float epsilon = 0.001f;
    auto& parts = spherePBD->particles(); 

    for (size_t i = 0; i < parts.size(); ++i) {
        if (parts[i].position.y >= maxY - epsilon) {
            parts[i].invMass = 0.0f; 
        }
    }
    */



    // Add entities to scene.
    // you can change the position/orientation.
    Scene scene;

    // add your model's entity here!
    Entity* sphereEntity = new Entity(&yourOwnModel, glm::vec3(-1, 1, -1), 0.0f, 0.0f, 0.0f, 0.2);
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
    unsigned int VAOskybox, VBOskybox;
    getPositionVAO(skybox_positions, sizeof(skybox_positions), VAOskybox, VBOskybox);


    lightingShader.use();
    lightingShader.setInt("material.diffuseSampler", 0);
    lightingShader.setInt("material.specularSampler", 1);
    lightingShader.setInt("material.normalSampler", 2);
    lightingShader.setInt("depthMapSampler", 3);
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
        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
            spherePBD->PBDSolver::addImpulse(1000, glm::vec3(5.0f, 2.0f, 0.0f));
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
        float damping = 0.00f;

        const int SUB_STEPS = 2;
        const int PBD_ITERATIONS = 2; // 비눗방울처럼 팽팽하려면 최소 20 이상 권장

        while (accumulator >= FIXED_DT) {
            float subDt = FIXED_DT / SUB_STEPS;
            if (spherePBD) {
                for (int i = 0; i < SUB_STEPS; i++) {
                    // 솔버 이터레이션을 2에서 20으로 증가시켜 파라미터 전달
                    spherePBD->step(subDt, PBD_ITERATIONS, gravity, damping);
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

            for (Entity* entity : it->second) {
                lightingShader.setMat4("world", entity->getModelMatrix());
                model->bind();
                glDrawElements(GL_TRIANGLES, model->mesh.indices.size(), GL_UNSIGNED_INT, 0);
            }
        }

        
        lightingShader.use(); // 조명 쉐이더 다시 바인딩
        lightingShader.setFloat("debugThickness", 1.0f); // 디버그 모드 ON!

        auto itSpherePBD = scene.entities.find(&yourOwnModel);
        if (itSpherePBD != scene.entities.end()) {
            for (Entity* entity : itSpherePBD->second) {
                lightingShader.setMat4("world", entity->getModelMatrix());
                yourOwnModel.bind();
                glDrawElements(GL_TRIANGLES, yourOwnModel.mesh.indices.size(), GL_UNSIGNED_INT, 0);
            }
        }

        lightingShader.setFloat("debugThickness", 0.0f); // 끄기




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
        glEnable(GL_CULL_FACE);


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
