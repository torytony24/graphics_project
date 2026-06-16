#include "pbd.h"
#include <unordered_set>
#include <algorithm>

struct EdgeKey {
    unsigned int a, b;
    EdgeKey(unsigned int x, unsigned int y) {
        if (x < y) { a = x; b = y; }
        else { a = y; b = x; }
    }
    bool operator==(EdgeKey const& o) const { return a == o.a && b == o.b; }
};
namespace std {
    template<> struct hash<EdgeKey> {
        size_t operator()(EdgeKey const& k) const noexcept {
            return (size_t)k.a * 1000003u + k.b;
        }
    };
}

PBDSolver::PBDSolver(Mesh* mesh) : m_mesh(mesh), m_initialVolume(0.0f) {}

void PBDSolver::initialize() {
    m_particles.clear();
    m_edges.clear();
    if (!m_mesh) return;

    m_particles.resize(m_mesh->vertices.size());
    for (size_t i = 0; i < m_mesh->vertices.size(); ++i) {
        auto& v = m_mesh->vertices[i];
        PBDParticle p;
        p.position = v.Position;
        p.prevPosition = v.Position;
        p.acceleration = glm::vec3(0.0f);
        p.invMass = 1.0f;

        float seed = std::sin(9.0f * p.position.x + 3.0f * p.position.y) +
            0.6f * std::sin(11.0f * p.position.z - 4.0f * p.position.x);
        p.thickness = 0.05f + 0.01f * seed;
        p.prevThickness = p.thickness;
        p.thicknessVelocity = 0.0f;
        m_particles[i] = p;
    }

    buildConstraints();
    m_initialVolume = computeMeshVolume();
}

void PBDSolver::buildConstraints() {
    std::unordered_set<EdgeKey> uniqueEdges;
    for (size_t i = 0; i + 2 < m_mesh->indices.size(); i += 3) {
        uniqueEdges.insert(EdgeKey(m_mesh->indices[i], m_mesh->indices[i + 1]));
        uniqueEdges.insert(EdgeKey(m_mesh->indices[i + 1], m_mesh->indices[i + 2]));
        uniqueEdges.insert(EdgeKey(m_mesh->indices[i + 2], m_mesh->indices[i]));
    }

    for (const auto& edge : uniqueEdges) {
        PBDEdge e;
        e.a = edge.a;
        e.b = edge.b;
        e.restLength = glm::length(m_particles[e.a].position - m_particles[e.b].position);
        m_edges.push_back(e);
    }
}

float PBDSolver::computeMeshVolume() {
    float volume = 0.0f;
    for (size_t i = 0; i + 2 < m_mesh->indices.size(); i += 3) {
        glm::vec3 p1 = m_particles[m_mesh->indices[i]].position;
        glm::vec3 p2 = m_particles[m_mesh->indices[i + 1]].position;
        glm::vec3 p3 = m_particles[m_mesh->indices[i + 2]].position;
        volume += glm::dot(p1, glm::cross(p2, p3));
    }
    return volume / 6.0f;
}

void PBDSolver::integrate(float dt, float damping) {
    if (dt <= 0.0f) return;
    float dt2 = dt * dt;
    for (auto& p : m_particles) {
        if (p.invMass == 0.0f) continue;
        glm::vec3 velocity = (p.position - p.prevPosition) * (1.0f - damping);
        glm::vec3 newPos = p.position + velocity + p.acceleration * dt2;
        p.prevPosition = p.position;
        p.position = newPos;
        p.acceleration = glm::vec3(0.0f); 
    }
}



void PBDSolver::solveConstraints(int iterations) {
    const float volumeCompliance = 1e-10f;

    const float edgeStiffness = 0.8f;

    for (int it = 0; it < iterations; ++it) {

        std::vector<glm::vec3> edgeCorr(m_particles.size(), glm::vec3(0.0f));
        std::vector<int> edgeCounts(m_particles.size(), 0);

        for (const auto& e : m_edges) {
            glm::vec3 pA = m_particles[e.a].position;
            glm::vec3 pB = m_particles[e.b].position;
            glm::vec3 delta = pB - pA;
            float len = glm::length(delta);

            if (len > 1e-6f) {
                float diff = (len - e.restLength) / len;
                glm::vec3 corr = delta * diff * edgeStiffness * 0.5f;

                edgeCorr[e.a] += corr;
                edgeCorr[e.b] -= corr;
                edgeCounts[e.a]++;
                edgeCounts[e.b]++;
            }
        }

        for (size_t i = 0; i < m_particles.size(); ++i) {
            if (edgeCounts[i] > 0) {
                m_particles[i].position += edgeCorr[i] / (float)edgeCounts[i];
            }
        }

        float currentVolume = computeMeshVolume();
        float C_vol = currentVolume - m_initialVolume;

        std::vector<glm::vec3> gradVol(m_particles.size(), glm::vec3(0.0f));
        for (size_t i = 0; i + 2 < m_mesh->indices.size(); i += 3) {
            unsigned int idx1 = m_mesh->indices[i];
            unsigned int idx2 = m_mesh->indices[i + 1];
            unsigned int idx3 = m_mesh->indices[i + 2];

            glm::vec3 p1 = m_particles[idx1].position;
            glm::vec3 p2 = m_particles[idx2].position;
            glm::vec3 p3 = m_particles[idx3].position;

            gradVol[idx1] += glm::cross(p2, p3) / 6.0f;
            gradVol[idx2] += glm::cross(p3, p1) / 6.0f;
            gradVol[idx3] += glm::cross(p1, p2) / 6.0f;
        }

        float sumGradVolSq = 0.0f;
        for (size_t i = 0; i < m_particles.size(); ++i) {
            sumGradVolSq += glm::dot(gradVol[i], gradVol[i]);
        }

        if (sumGradVolSq > 1e-6f) {
            float lambdaVol = -C_vol / (sumGradVolSq + volumeCompliance);
            for (size_t i = 0; i < m_particles.size(); ++i) {
                m_particles[i].position += lambdaVol * gradVol[i];
            }
        }
    }
}



void PBDSolver::recomputeNormals() {
    for (auto& v : m_mesh->vertices) v.Normal = glm::vec3(0.0f);
    for (size_t i = 0; i + 2 < m_mesh->indices.size(); i += 3) {
        unsigned int i0 = m_mesh->indices[i];
        unsigned int i1 = m_mesh->indices[i + 1];
        unsigned int i2 = m_mesh->indices[i + 2];
        glm::vec3 p0 = m_mesh->vertices[i0].Position;
        glm::vec3 p1 = m_mesh->vertices[i1].Position;
        glm::vec3 p2 = m_mesh->vertices[i2].Position;
        glm::vec3 normal = glm::cross(p1 - p0, p2 - p0);
        if (glm::length(normal) > 1e-6f) {
            normal = glm::normalize(normal);
            m_mesh->vertices[i0].Normal += normal;
            m_mesh->vertices[i1].Normal += normal;
            m_mesh->vertices[i2].Normal += normal;
        }
    }
    for (auto& v : m_mesh->vertices) {
        if (glm::length(v.Normal) > 1e-6f) v.Normal = glm::normalize(v.Normal);
    }
}



void PBDSolver::integrateThickness(float dt) {
    if (dt <= 0.0f) return;

    std::vector<float> laplacians(m_particles.size(), 0.0f);

    for (const auto& e : m_edges) {
        float hA = m_particles[e.a].thickness;
        float hB = m_particles[e.b].thickness;

        float diff = hB - hA;
        laplacians[e.a] += diff;
        laplacians[e.b] -= diff;
    }

    float c2 = 100.0f; 
    float k_damp = 0.0f; 

    for (size_t i = 0; i < m_particles.size(); ++i) {
        auto& p = m_particles[i];

        float accel = c2 * laplacians[i];

        p.thicknessVelocity += accel * dt;
        p.thicknessVelocity *= (1.0f - k_damp);

        p.prevThickness = p.thickness;
        p.thickness += p.thicknessVelocity * dt;

        p.thickness = std::max(0.012f, std::min(p.thickness, 0.095f));
    }
}

void PBDSolver::step(float dt, int solverIterations, const glm::vec3& gravity, float damping) {
    m_time += dt;

    glm::vec3 centerOfMass(0.0f);
    for (const auto& p : m_particles) {
        centerOfMass += p.position;
    }
    centerOfMass /= (float)m_particles.size();

    float hoverY = std::sin(m_time * 0.8f) * 0.01f;
    glm::vec3 targetCenter(0.0f, hoverY, 0.0f);


    float stiffness = 5.0f;
    glm::vec3 restoreForce = (targetCenter - centerOfMass) * stiffness;

    float windStrength = 0.02f;
    for (auto& p : m_particles) {
        float windX = std::sin(p.position.y * 2.0f + m_time * 0.5f);
        float windY = std::cos(p.position.z * 2.0f + m_time * 0.4f);
        float windZ = std::cos(p.position.x * 2.0f + m_time * 0.6f);

        glm::vec3 windForce = glm::vec3(windX, windY, windZ) * windStrength;

        p.acceleration += gravity + windForce + restoreForce;
    }


    std::vector<glm::vec3> laplaceBeltrami(m_particles.size(), glm::vec3(0.0f));

    auto safeCot = [](const glm::vec3& a, const glm::vec3& b) -> float {
        float dotP = glm::dot(a, b);
        float crossLen = glm::length(glm::cross(a, b));
        if (crossLen < 1e-6f) return 0.0f;
        return glm::clamp(dotP / crossLen, -10.0f, 10.0f);
        };

    for (size_t i = 0; i + 2 < m_mesh->indices.size(); i += 3) {
        unsigned int i0 = m_mesh->indices[i];
        unsigned int i1 = m_mesh->indices[i + 1];
        unsigned int i2 = m_mesh->indices[i + 2];

        glm::vec3 p0 = m_particles[i0].position;
        glm::vec3 p1 = m_particles[i1].position;
        glm::vec3 p2 = m_particles[i2].position;

        glm::vec3 v01 = p1 - p0;
        glm::vec3 v12 = p2 - p1;
        glm::vec3 v20 = p0 - p2;

        float cot0 = safeCot(v01, -v20); 
        float cot1 = safeCot(v12, -v01); 
        float cot2 = safeCot(v20, -v12); 

        laplaceBeltrami[i0] += 0.5f * (cot2 * v01 + cot1 * (-v20));
        laplaceBeltrami[i1] += 0.5f * (cot0 * v12 + cot2 * (-v01));
        laplaceBeltrami[i2] += 0.5f * (cot1 * v20 + cot0 * (-v12));
    }

    float surfaceTension = 15.0f;   // surface tension 15-25
    for (size_t i = 0; i < m_particles.size(); ++i) {
        m_particles[i].acceleration += laplaceBeltrami[i] * surfaceTension;
    }


    integrate(dt, damping);
    solveConstraints(solverIterations);
    integrateThickness(dt);
    recomputeNormals();
}

void PBDSolver::applyToMesh() {
    if (!m_mesh) return;
    for (size_t i = 0; i < m_particles.size(); ++i) {
        m_mesh->vertices[i].Position = m_particles[i].position;
        m_mesh->vertices[i].Thickness = m_particles[i].thickness; 
    }
    m_mesh->updateVertexBuffer();
}

void PBDSolver::addImpulse(unsigned int particleIdx, const glm::vec3& velocity) {
    if (particleIdx >= m_particles.size()) return;

    m_particles[particleIdx].prevPosition -= velocity * 0.001f;
    
}