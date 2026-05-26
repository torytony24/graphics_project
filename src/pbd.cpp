#include "pbd.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

#include <glm/gtc/constants.hpp>

namespace {
struct EdgeKey {
    unsigned int a;
    unsigned int b;

    EdgeKey(unsigned int x, unsigned int y)
    {
        if (x < y) {
            a = x;
            b = y;
        } else {
            a = y;
            b = x;
        }
    }

    bool operator==(const EdgeKey& other) const
    {
        return a == other.a && b == other.b;
    }
};

struct EdgeKeyHash {
    size_t operator()(const EdgeKey& edge) const
    {
        return static_cast<size_t>(edge.a) * 73856093u ^ static_cast<size_t>(edge.b) * 19349663u;
    }
};

struct AdjacentFaces {
    unsigned int opposite[2];
    int count;

    AdjacentFaces() : opposite{0, 0}, count(0) {}
};

float clamp01(float value)
{
    return std::max(0.0f, std::min(1.0f, value));
}
}

StretchConstraint::StretchConstraint(unsigned int a, unsigned int b, float restLength, float stiffness)
    : m_a(a), m_b(b), m_restLength(restLength), m_stiffness(clamp01(stiffness))
{
}

void StretchConstraint::project(std::vector<PBDParticle>& particles) const
{
    PBDParticle& a = particles[m_a];
    PBDParticle& b = particles[m_b];
    const float w1 = a.inverseMass;
    const float w2 = b.inverseMass;
    const float wsum = w1 + w2;
    if (wsum <= 0.0f) return;

    const glm::vec3 delta = b.position - a.position;
    const float length = glm::length(delta);
    if (length <= 1e-6f) return;

    const glm::vec3 correction = (length - m_restLength) * (delta / length) * m_stiffness;
    if (w1 > 0.0f) a.position += correction * (w1 / wsum);
    if (w2 > 0.0f) b.position -= correction * (w2 / wsum);
}

BendConstraint::BendConstraint(unsigned int a, unsigned int b, float restLength, float stiffness)
    : m_a(a), m_b(b), m_restLength(restLength), m_stiffness(clamp01(stiffness))
{
}

void BendConstraint::project(std::vector<PBDParticle>& particles) const
{
    PBDParticle& a = particles[m_a];
    PBDParticle& b = particles[m_b];
    const float w1 = a.inverseMass;
    const float w2 = b.inverseMass;
    const float wsum = w1 + w2;
    if (wsum <= 0.0f) return;

    const glm::vec3 delta = b.position - a.position;
    const float length = glm::length(delta);
    if (length <= 1e-6f) return;

    const glm::vec3 correction = (length - m_restLength) * (delta / length) * m_stiffness;
    if (w1 > 0.0f) a.position += correction * (w1 / wsum);
    if (w2 > 0.0f) b.position -= correction * (w2 / wsum);
}

PBDSolver::PBDSolver(Mesh* mesh)
    : m_mesh(mesh),
      m_stretchStiffness(0.85f),
      m_bendStiffness(0.15f),
      m_damping(0.015f),
      m_pressure(0.0f),
      m_radiusStiffness(0.35f),
      m_restCenter(0.0f),
      m_restAverageRadius(1.0f)
{
}

void PBDSolver::initialize()
{
    m_particles.clear();
    m_stretchConstraints.clear();
    m_bendConstraints.clear();
    m_neighbors.clear();

    if (!m_mesh) return;

    m_particles.resize(m_mesh->vertices.size());
    for (size_t i = 0; i < m_mesh->vertices.size(); ++i) {
        const Vertex& vertex = m_mesh->vertices[i];
        PBDParticle particle;
        particle.position = vertex.Position;
        particle.previousPosition = vertex.Position;
        particle.restPosition = vertex.Position;
        particle.velocity = glm::vec3(0.0f);
        particle.force = glm::vec3(0.0f);
        particle.inverseMass = 1.0f;
        particle.thickness = 0.05f;
        particle.previousThickness = 0.05f;
        particle.thicknessVelocity = 0.0f;
        particle.baseArea = 0.0f;
        m_particles[i] = particle;
    }

    m_restCenter = glm::vec3(0.0f);
    for (size_t i = 0; i < m_particles.size(); ++i) {
        m_restCenter += m_particles[i].restPosition;
    }
    if (!m_particles.empty()) {
        m_restCenter /= static_cast<float>(m_particles.size());
    }

    m_restAverageRadius = 0.0f;
    for (size_t i = 0; i < m_particles.size(); ++i) {
        m_restAverageRadius += glm::length(m_particles[i].restPosition - m_restCenter);
    }
    if (!m_particles.empty()) {
        m_restAverageRadius /= static_cast<float>(m_particles.size());
    }

    for (size_t i = 0; i + 2 < m_mesh->indices.size(); i += 3) {
        const unsigned int i0 = m_mesh->indices[i];
        const unsigned int i1 = m_mesh->indices[i + 1];
        const unsigned int i2 = m_mesh->indices[i + 2];

        const glm::vec3 p0 = m_mesh->vertices[i0].Position;
        const glm::vec3 p1 = m_mesh->vertices[i1].Position;
        const glm::vec3 p2 = m_mesh->vertices[i2].Position;
        const float area = 0.5f * glm::length(glm::cross(p1 - p0, p2 - p0));
        m_particles[i0].baseArea += area / 3.0f;
        m_particles[i1].baseArea += area / 3.0f;
        m_particles[i2].baseArea += area / 3.0f;
    }

    buildConstraintsFromTriangles();
    m_thinFilm.initialize(m_mesh, 0.05f);
    recomputeNormals();
}

void PBDSolver::setStretchStiffness(float stiffness)
{
    m_stretchStiffness = clamp01(stiffness);
    for (size_t i = 0; i < m_stretchConstraints.size(); ++i) {
        m_stretchConstraints[i].setStiffness(m_stretchStiffness);
    }
}

void PBDSolver::setBendStiffness(float stiffness)
{
    m_bendStiffness = clamp01(stiffness);
    for (size_t i = 0; i < m_bendConstraints.size(); ++i) {
        m_bendConstraints[i].setStiffness(m_bendStiffness);
    }
}

void PBDSolver::buildConstraintsFromTriangles()
{
    if (!m_mesh) return;

    m_stretchConstraints.clear();
    m_bendConstraints.clear();
    m_neighbors.assign(m_particles.size(), std::vector<unsigned int>());

    std::unordered_set<EdgeKey, EdgeKeyHash> uniqueEdges;
    std::unordered_map<EdgeKey, AdjacentFaces, EdgeKeyHash> edgeFaces;

    for (size_t i = 0; i + 2 < m_mesh->indices.size(); i += 3) {
        const unsigned int i0 = m_mesh->indices[i];
        const unsigned int i1 = m_mesh->indices[i + 1];
        const unsigned int i2 = m_mesh->indices[i + 2];

        const unsigned int edgeVerts[3][2] = {{i0, i1}, {i1, i2}, {i2, i0}};
        const unsigned int oppositeVerts[3] = {i2, i0, i1};

        for (int edgeIdx = 0; edgeIdx < 3; ++edgeIdx) {
            EdgeKey edge(edgeVerts[edgeIdx][0], edgeVerts[edgeIdx][1]);
            uniqueEdges.insert(edge);

            AdjacentFaces& faces = edgeFaces[edge];
            if (faces.count < 2) {
                faces.opposite[faces.count] = oppositeVerts[edgeIdx];
                faces.count++;
            }
        }
    }

    m_stretchConstraints.reserve(uniqueEdges.size());
    for (std::unordered_set<EdgeKey, EdgeKeyHash>::const_iterator it = uniqueEdges.begin(); it != uniqueEdges.end(); ++it) {
        const EdgeKey& edge = *it;
        const float restLength = glm::length(m_particles[edge.b].position - m_particles[edge.a].position);
        m_stretchConstraints.push_back(StretchConstraint(edge.a, edge.b, restLength, m_stretchStiffness));
        m_neighbors[edge.a].push_back(edge.b);
        m_neighbors[edge.b].push_back(edge.a);
    }

    for (std::unordered_map<EdgeKey, AdjacentFaces, EdgeKeyHash>::const_iterator it = edgeFaces.begin(); it != edgeFaces.end(); ++it) {
        const AdjacentFaces& faces = it->second;
        if (faces.count != 2 || faces.opposite[0] == faces.opposite[1]) continue;

        const float restLength = glm::length(m_particles[faces.opposite[1]].position - m_particles[faces.opposite[0]].position);
        if (restLength > 1e-6f) {
            m_bendConstraints.push_back(BendConstraint(faces.opposite[0], faces.opposite[1], restLength, m_bendStiffness));
        }
    }
}

void PBDSolver::applyExternalForces(const glm::vec3& externalForce)
{
    if (!m_mesh) return;

    recomputeNormals();
    for (size_t i = 0; i < m_particles.size(); ++i) {
        PBDParticle& particle = m_particles[i];
        particle.force = glm::vec3(0.0f);
        if (particle.inverseMass <= 0.0f) continue;

        particle.force += externalForce;
        if (m_pressure != 0.0f) {
            particle.force += m_mesh->vertices[i].Normal * m_pressure;
        }
    }
}

void PBDSolver::predictPositions(float dt)
{
    const float dt2 = dt * dt;
    for (size_t i = 0; i < m_particles.size(); ++i) {
        PBDParticle& particle = m_particles[i];
        particle.previousPosition = particle.position;
        if (particle.inverseMass <= 0.0f) {
            particle.velocity = glm::vec3(0.0f);
            continue;
        }

        particle.position += particle.velocity * dt + particle.force * particle.inverseMass * dt2;
    }
}

void PBDSolver::projectConstraints(int iterations)
{
    const int count = std::max(1, iterations);
    for (int iteration = 0; iteration < count; ++iteration) {
        for (size_t i = 0; i < m_stretchConstraints.size(); ++i) {
            m_stretchConstraints[i].project(m_particles);
        }

        for (size_t i = 0; i < m_bendConstraints.size(); ++i) {
            m_bendConstraints[i].project(m_particles);
        }

        projectRadiusConstraint();
    }
}

void PBDSolver::projectRadiusConstraint()
{
    if (m_particles.empty() || m_restAverageRadius <= 1e-6f) return;

    glm::vec3 center(0.0f);
    float massSum = 0.0f;
    for (size_t i = 0; i < m_particles.size(); ++i) {
        const PBDParticle& particle = m_particles[i];
        if (particle.inverseMass <= 0.0f) continue;
        center += particle.position;
        massSum += 1.0f;
    }
    if (massSum <= 0.0f) return;
    center /= massSum;

    float averageRadius = 0.0f;
    int movableCount = 0;
    for (size_t i = 0; i < m_particles.size(); ++i) {
        const PBDParticle& particle = m_particles[i];
        if (particle.inverseMass <= 0.0f) continue;
        averageRadius += glm::length(particle.position - center);
        movableCount++;
    }
    if (movableCount == 0) return;
    averageRadius /= static_cast<float>(movableCount);
    if (averageRadius <= 1e-6f) return;

    const float scale = 1.0f + (m_restAverageRadius / averageRadius - 1.0f) * m_radiusStiffness;
    for (size_t i = 0; i < m_particles.size(); ++i) {
        PBDParticle& particle = m_particles[i];
        if (particle.inverseMass <= 0.0f) continue;
        particle.position = center + (particle.position - center) * scale;
    }
}

void PBDSolver::updateVelocities(float dt, float damping)
{
    if (dt <= 0.0f) return;

    const float damp = 1.0f - clamp01(damping);
    for (size_t i = 0; i < m_particles.size(); ++i) {
        PBDParticle& particle = m_particles[i];
        if (particle.inverseMass <= 0.0f) {
            particle.velocity = glm::vec3(0.0f);
            particle.previousPosition = particle.position;
            continue;
        }

        particle.velocity = (particle.position - particle.previousPosition) / dt;
        particle.velocity *= damp;
    }
}

void PBDSolver::step(float dt, int solverIterations, const glm::vec3& externalForce, float damping)
{
    if (!m_mesh || dt <= 0.0f) return;

    m_damping = damping;
    applyExternalForces(externalForce);
    predictPositions(dt);
    projectConstraints(solverIterations);
    updateVelocities(dt, m_damping);

    for (size_t i = 0; i < m_particles.size() && i < m_mesh->vertices.size(); ++i) {
        m_mesh->vertices[i].Position = m_particles[i].position;
    }
    m_thinFilm.step(dt);
}

void PBDSolver::applyToMesh()
{
    if (!m_mesh) return;

    const size_t count = std::min(m_particles.size(), m_mesh->vertices.size());
    for (size_t i = 0; i < count; ++i) {
        m_mesh->vertices[i].Position = m_particles[i].position;

    }

    m_thinFilm.applyToMesh();
    recomputeNormals();
    m_mesh->updateVertexBuffer();
}

void PBDSolver::addImpulse(unsigned int particleIdx, const glm::vec3& velocity)
{
    if (particleIdx >= m_particles.size()) return;

    PBDParticle& particle = m_particles[particleIdx];
    if (particle.inverseMass <= 0.0f) return;

    particle.velocity += velocity;
    m_thinFilm.injectDisturbance(particleIdx, 0.018f, 0.22f);
}

void PBDSolver::injectThicknessDisturbance(unsigned int particleIdx, float deltaThickness, float radius)
{
    m_thinFilm.injectDisturbance(particleIdx, deltaThickness, radius);
}

void PBDSolver::recomputeNormals()
{
    if (!m_mesh) return;

    for (size_t i = 0; i < m_mesh->vertices.size(); ++i) {
        m_mesh->vertices[i].Normal = glm::vec3(0.0f);
    }

    for (size_t i = 0; i + 2 < m_mesh->indices.size(); i += 3) {
        const unsigned int i0 = m_mesh->indices[i];
        const unsigned int i1 = m_mesh->indices[i + 1];
        const unsigned int i2 = m_mesh->indices[i + 2];

        const glm::vec3 p0 = m_mesh->vertices[i0].Position;
        const glm::vec3 p1 = m_mesh->vertices[i1].Position;
        const glm::vec3 p2 = m_mesh->vertices[i2].Position;
        glm::vec3 normal = glm::cross(p1 - p0, p2 - p0);
        if (glm::length(normal) <= 1e-6f) continue;

        normal = glm::normalize(normal);
        m_mesh->vertices[i0].Normal += normal;
        m_mesh->vertices[i1].Normal += normal;
        m_mesh->vertices[i2].Normal += normal;
    }

    for (size_t i = 0; i < m_mesh->vertices.size(); ++i) {
        Vertex& vertex = m_mesh->vertices[i];
        if (glm::length(vertex.Normal) > 1e-6f) {
            vertex.Normal = glm::normalize(vertex.Normal);
        } else {
            vertex.Normal = glm::vec3(0.0f, 1.0f, 0.0f);
        }
    }
}
