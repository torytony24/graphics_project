#pragma once

#include "mesh.h"

#include <glm/glm.hpp>
#include <vector>

// Evolves per-vertex film thickness over time.
class ThinFilmSimulator {
public:
    ThinFilmSimulator();

    void initialize(Mesh* mesh, float initialThickness);
    void step(float dt);
    void injectDisturbance(unsigned int centerVertex, float deltaThickness, float radius);
    void addThicknessDelta(unsigned int vertexIdx, float deltaThickness);
    void applyToMesh();

    void setWaveSpeedSquared(float c2) { m_c2 = c2; }
    void setDamping(float damping) { m_damping = damping; }
    void setThicknessRange(float minThickness, float maxThickness);

    const std::vector<float>& thickness() const { return m_h; }
    const std::vector<float>& thicknessVelocity() const { return m_vh; }

private:
    Mesh* m_mesh;
    std::vector<float> m_h;
    std::vector<float> m_vh;
    std::vector<float> m_vertexArea;
    std::vector<std::vector<unsigned int> > m_neighbors;

    float m_c2;
    float m_damping;
    float m_minThickness;
    float m_maxThickness;
    float m_initialTotalThickness;

    void buildAdjacency();
    void computeVertexAreas();
    void conserveGlobalThickness();
    glm::vec3 falseColor(float h) const;
};
