#pragma once

#include "mesh.h"
#include "thin_film.h"

#include <glm/glm.hpp>
#include <vector>

struct PBDParticle {
    glm::vec3 position;
    glm::vec3 previousPosition;
    glm::vec3 restPosition;
    glm::vec3 velocity;
    glm::vec3 force;
    float inverseMass;

    float thickness;
    float previousThickness;
    float thicknessVelocity;
    float baseArea;
};

class StretchConstraint {
public:
    StretchConstraint(unsigned int a, unsigned int b, float restLength, float stiffness);

    void project(std::vector<PBDParticle>& particles) const;

    unsigned int first() const { return m_a; }
    unsigned int second() const { return m_b; }
    float restLength() const { return m_restLength; }
    void setStiffness(float stiffness) { m_stiffness = stiffness; }

private:
    unsigned int m_a;
    unsigned int m_b;
    float m_restLength;
    float m_stiffness;
};

class BendConstraint {
public:
    BendConstraint(unsigned int a, unsigned int b, float restLength, float stiffness);

    void project(std::vector<PBDParticle>& particles) const;
    void setStiffness(float stiffness) { m_stiffness = stiffness; }

private:
    unsigned int m_a;
    unsigned int m_b;
    float m_restLength;
    float m_stiffness;
};

class PBDSolver {
public:
    explicit PBDSolver(Mesh* mesh);

    void initialize();
    void step(float dt, int solverIterations, const glm::vec3& externalForce, float damping);
    void applyToMesh();
    void addImpulse(unsigned int particleIdx, const glm::vec3& velocity);
    void injectThicknessDisturbance(unsigned int particleIdx, float deltaThickness, float radius);

    void setStretchStiffness(float stiffness);
    void setBendStiffness(float stiffness);
    void setDamping(float damping) { m_damping = damping; }
    void setPressure(float pressure) { m_pressure = pressure; }

    std::vector<PBDParticle>& particles() { return m_particles; }
    const std::vector<StretchConstraint>& stretchConstraints() const { return m_stretchConstraints; }
    const std::vector<BendConstraint>& bendConstraints() const { return m_bendConstraints; }

private:
    Mesh* m_mesh;
    std::vector<PBDParticle> m_particles;
    std::vector<StretchConstraint> m_stretchConstraints;
    std::vector<BendConstraint> m_bendConstraints;
    std::vector<std::vector<unsigned int> > m_neighbors;
    ThinFilmSimulator m_thinFilm;

    float m_stretchStiffness;
    float m_bendStiffness;
    float m_damping;
    float m_pressure;
    float m_radiusStiffness;
    glm::vec3 m_restCenter;
    float m_restAverageRadius;

    void buildConstraintsFromTriangles();
    void applyExternalForces(const glm::vec3& externalForce);
    void predictPositions(float dt);
    void projectConstraints(int iterations);
    void projectRadiusConstraint();
    void updateVelocities(float dt, float damping);
    void recomputeNormals();
};
