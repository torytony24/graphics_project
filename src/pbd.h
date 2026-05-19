#pragma once

#include "mesh.h"
#include <glm/glm.hpp>
#include <vector>

struct PBDParticle {
    glm::vec3 position;
    glm::vec3 prevPosition;
    glm::vec3 acceleration;
    float invMass;

    float thickness;     
    float prevThickness; 
    float thicknessVelocity; 
    float baseArea;
};

struct PBDConstraint {
    unsigned int a, b; 
    float restLength;
};

class PBDSolver {
public:
    PBDSolver(Mesh* mesh);

    void initialize();

    void step(float dt, int solverIterations, const glm::vec3& gravity, float damping);

    void applyToMesh();

    void addImpulse(unsigned int particleIdx, const glm::vec3& velocity);

    std::vector<PBDParticle>& particles() { return m_particles; }
    std::vector<PBDConstraint>& constraints() { return m_constraints; }

private:
    Mesh* m_mesh;
    std::vector<PBDParticle> m_particles;
    std::vector<PBDConstraint> m_constraints;
    
    std::vector<std::vector<unsigned int>> m_neighbors;
    float m_initialTotalVolume;

    void buildConstraintsFromTriangles();
    void integrate(float dt, float damping);
    void solveConstraints(int iterations);
    void recomputeNormals();
    
    void integrateThickness(float dt);
    void solveThicknessConstraints(const std::vector<float>& currentAreas);
};