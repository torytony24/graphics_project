#include "pbd.h"
#include <unordered_set>
#include <tuple>
#include <algorithm>
#include <glm/gtc/type_ptr.hpp>

PBDSolver::PBDSolver(Mesh* mesh)
    : m_mesh(mesh)
{
}

void PBDSolver::initialize()
{
    m_particles.clear();
    m_constraints.clear();

    if (!m_mesh) return;

    m_particles.resize(m_mesh->vertices.size());
    for (size_t i = 0; i < m_mesh->vertices.size(); ++i) {
        const auto& v = m_mesh->vertices[i];
        PBDParticle p;
        p.position = v.Position;
        p.prevPosition = v.Position;
        p.acceleration = glm::vec3(0.0f);
        p.invMass = 1.0f;
        m_particles[i] = p;
    }

    m_neighbors.assign(m_particles.size(), std::vector<unsigned int>());
    float initialThickness = 0.05f;

    // [수정] 모든 정점의 baseArea를 1.0f로 두지 않고, 실제 초기 삼각형 면적으로 계산합니다.
    for (size_t i = 0; i < m_particles.size(); ++i) {
        m_particles[i].thickness = initialThickness;
        m_particles[i].prevThickness = initialThickness;
        m_particles[i].thicknessVelocity = 0.0f;
        m_particles[i].baseArea = 0.0f;
    }

    for (size_t i = 0; i + 2 < m_mesh->indices.size(); i += 3) {
        unsigned int i0 = m_mesh->indices[i];
        unsigned int i1 = m_mesh->indices[i + 1];
        unsigned int i2 = m_mesh->indices[i + 2];

        glm::vec3 p0 = m_mesh->vertices[i0].Position;
        glm::vec3 p1 = m_mesh->vertices[i1].Position;
        glm::vec3 p2 = m_mesh->vertices[i2].Position;

        // 삼각형의 면적 계산 후 각 정점에 1/3씩 분배
        float triArea = 0.5f * glm::length(glm::cross(p1 - p0, p2 - p0));
        m_particles[i0].baseArea += triArea / 3.0f;
        m_particles[i1].baseArea += triArea / 3.0f;
        m_particles[i2].baseArea += triArea / 3.0f;
    }

    m_initialTotalVolume = 0.0f;
    for (size_t i = 0; i < m_particles.size(); ++i) {
        m_initialTotalVolume += initialThickness * m_particles[i].baseArea;
    }

    buildConstraintsFromTriangles();
}

struct EdgeKey {
    unsigned int a, b;
    EdgeKey(unsigned int x, unsigned int y) {
        if (x < y) { a = x; b = y; }
        else { a = y; b = x; }
    }
    bool operator==(EdgeKey const& o) const { return a == o.a && b == o.b; }
};
struct EdgeHash {
    size_t operator()(EdgeKey const& k) const noexcept {
        return (size_t)k.a * 1000003u + k.b;
    }
};

void PBDSolver::buildConstraintsFromTriangles()
{
    if (!m_mesh) return;

    std::unordered_set<EdgeKey, EdgeHash> edges;
    for (size_t i = 0; i + 2 < m_mesh->indices.size(); i += 3) {
        unsigned int i0 = m_mesh->indices[i];
        unsigned int i1 = m_mesh->indices[i + 1];
        unsigned int i2 = m_mesh->indices[i + 2];

        EdgeKey e0(i0, i1), e1(i1, i2), e2(i2, i0);
        edges.insert(e0); edges.insert(e1); edges.insert(e2);
    }

    m_constraints.reserve(edges.size());
    for (auto const& e : edges) {
        PBDConstraint c;
        c.a = e.a;
        c.b = e.b;
        glm::vec3 pa = m_particles[c.a].position;
        glm::vec3 pb = m_particles[c.b].position;
        c.restLength = glm::length(pb - pa);
        m_constraints.push_back(c);

        m_neighbors[e.a].push_back(e.b);
        m_neighbors[e.b].push_back(e.a);
    }

    for (size_t i = 0; i < m_particles.size(); ++i) {
        for (size_t j = i + 1; j < m_particles.size(); ++j) {
            if (glm::length(m_particles[i].position - m_particles[j].position) < 1e-4f) {
                PBDConstraint weld;
                weld.a = i;
                weld.b = j;
                weld.restLength = 0.0f;
                m_constraints.push_back(weld);
            }
        }
    }
}

void PBDSolver::integrate(float dt, float damping)
{
    if (dt <= 0.0f) return;
    float dt2 = dt * dt;
    for (auto& p : m_particles) {
        if (p.invMass == 0.0f) {
            p.acceleration = glm::vec3(0.0f);
            continue;
        }
        glm::vec3 velocity = (p.position - p.prevPosition) * (1.0f - damping);
        glm::vec3 newPos = p.position + velocity + p.acceleration * dt2;
        p.prevPosition = p.position;
        p.position = newPos;
        p.acceleration = glm::vec3(0.0f);
    }
}

void PBDSolver::solveConstraints(int iterations)
{
    if (iterations <= 0) iterations = 1;
    for (int it = 0; it < iterations; ++it) {
        for (auto& c : m_constraints) {
            PBDParticle& A = m_particles[c.a];
            PBDParticle& B = m_particles[c.b];
            glm::vec3 delta = B.position - A.position;
            float len = glm::length(delta);
            if (len <= 1e-6f) continue;
            float diff = (len - c.restLength) / len;

            float w1 = A.invMass;
            float w2 = B.invMass;
            float wsum = w1 + w2;
            if (wsum == 0.0f) continue;

            glm::vec3 correction = delta * diff;
            if (A.invMass > 0.0f) A.position += correction * (w1 / wsum);
            if (B.invMass > 0.0f) B.position -= correction * (w2 / wsum);
        }
    }
}

void PBDSolver::recomputeNormals()
{
    for (auto& v : m_mesh->vertices) {
        v.Normal = glm::vec3(0.0f);
    }

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
        else v.Normal = glm::vec3(0.0f, 1.0f, 0.0f);
    }
}

void PBDSolver::step(float dt, int solverIterations, const glm::vec3& gravity, float damping)
{
    recomputeNormals();

    // 1. 공기압(Pressure) 적용: 비눗방울 팽창력
    float pressure = 2.0f;
    for (size_t i = 0; i < m_particles.size(); ++i) {
        if (m_particles[i].invMass > 0.0f) {
            m_particles[i].acceleration += gravity;
            m_particles[i].acceleration += m_mesh->vertices[i].Normal * pressure;
        }
    }

    // 2. 물리적 위치 이동 및 파동 흐름
    integrate(dt, damping);
    integrateThickness(dt);

    // 3. 제약 조건 반복 해결 (형태 + 두께 부피)
    for (int i = 0; i < solverIterations; ++i) {
        solveConstraints(1); // 1번 풀고

        // [핵심] 매 iteration마다 실시간 면적을 다시 계산합니다.
        std::vector<float> currentAreas(m_particles.size(), 0.0f);
        for (size_t j = 0; j + 2 < m_mesh->indices.size(); j += 3) {
            unsigned int i0 = m_mesh->indices[j];
            unsigned int i1 = m_mesh->indices[j + 1];
            unsigned int i2 = m_mesh->indices[j + 2];

            float triArea = 0.5f * glm::length(glm::cross(
                m_particles[i1].position - m_particles[i0].position,
                m_particles[i2].position - m_particles[i0].position));

            currentAreas[i0] += triArea / 3.0f;
            currentAreas[i1] += triArea / 3.0f;
            currentAreas[i2] += triArea / 3.0f;
        }

        // 실시간 면적을 바탕으로 엄밀한 PBD 두께 보정
        solveThicknessConstraints(currentAreas);
    }

    // 4. 속도 갱신 및 질량 커플링
    for (auto& p : m_particles) {
        p.thicknessVelocity = (p.thickness - p.prevThickness) / dt;
        if (p.invMass > 0.0f) {
            float mass = p.baseArea * p.thickness * 100.0f;
            p.invMass = 1.0f / (mass + 1e-6f);
        }
    }
    applyToMesh();
}

void PBDSolver::applyToMesh()
{
    if (!m_mesh) return;
    size_t n = std::min(m_particles.size(), m_mesh->vertices.size());

    // [핵심] 파동의 진폭이 매우 작으므로, 범위를 아주 좁게(민감하게) 설정해야 합니다!
    float baseT = 0.05f;             // 초기 기본 두께
    float waveAmplitude = 0.001f;    // 감지할 최대 두께 변화량 (민감도)
    float minT = baseT - waveAmplitude; // 0.048f
    float maxT = baseT + waveAmplitude; // 0.052f

    for (size_t i = 0; i < n; ++i) {
        m_mesh->vertices[i].Position = m_particles[i].position;

        float thickness = m_particles[i].thickness;

        // 정규화 (0.0 ~ 1.0)
        float t = (thickness - minT) / (maxT - minT);
        t = glm::clamp(t, 0.0f, 1.0f);

        glm::vec3 color;
        // 직관적인 3색 그라데이션: 파랑(얇음) -> 옅은 회백색(기본) -> 빨강(두꺼움)
        if (t < 0.5f) {
            // 0.0 ~ 0.5 구간: 파란색에서 흰색으로
            float blend = t * 2.0f;
            color = glm::mix(glm::vec3(0.1f, 0.3f, 1.0f), glm::vec3(0.8f, 0.8f, 0.8f), blend);
        }
        else {
            // 0.5 ~ 1.0 구간: 흰색에서 빨간색으로
            float blend = (t - 0.5f) * 2.0f;
            color = glm::mix(glm::vec3(0.8f, 0.8f, 0.8f), glm::vec3(1.0f, 0.2f, 0.1f), blend);
        }

        m_mesh->vertices[i].Color = color;
    }

    recomputeNormals();
    m_mesh->updateVertexBuffer();
}

void PBDSolver::addImpulse(unsigned int particleIdx, const glm::vec3& velocity)
{
    if (particleIdx >= m_particles.size() || m_particles[particleIdx].invMass == 0.0f)
        return;

    // 위치 충격
    m_particles[particleIdx].acceleration += velocity * 50.0f;

}

void PBDSolver::integrateThickness(float dt)
{
    // 추천 디버깅 값: c2를 늘리면 파동이 빨라지고, k_damp를 낮추면 일렁임이 오래갑니다.
    float c2 = 5.0f;    // 파동 속도를 50에서 150으로 상향 (색 변화 가속)
    float k_damp = 0.001f; // 감쇠를 아주 낮추어 오랫동안 찰랑거리게 함

    for (size_t i = 0; i < m_particles.size(); ++i) {
        auto& p = m_particles[i];

        float laplacian = 0.0f;
        for (unsigned int neighborIdx : m_neighbors[i]) {
            laplacian += (m_particles[neighborIdx].thickness - p.thickness);
        }

        float thicknessAccel = c2 * laplacian;

        p.thicknessVelocity += thicknessAccel * dt;
        p.thicknessVelocity *= (1.0f - k_damp);

        p.prevThickness = p.thickness;
        p.thickness += p.thicknessVelocity * dt;

        if (p.thickness < 0.001f) p.thickness = 0.001f;
    }
}

// 헤더(pbd.h)에도 함수 인자를 void solveThicknessConstraints(const std::vector<float>& currentAreas); 로 변경해주세요.
void PBDSolver::solveThicknessConstraints(const std::vector<float>& currentAreas)
{
    // Constraint 식: C = Sum(h_i * A_i) - V_initial = 0
    float currentTotalVolume = 0.0f;
    float gradientSumSq = 0.0f; // 분모 (Sum of gradients squared)

    for (size_t i = 0; i < m_particles.size(); ++i) {
        currentTotalVolume += m_particles[i].thickness * currentAreas[i];
        // h_i에 대한 편미분(Gradient)은 currentAreas[i] 입니다.
        gradientSumSq += currentAreas[i] * currentAreas[i];
    }

    float C = currentTotalVolume - m_initialTotalVolume;

    // 분모가 0이 되는 것 방지
    if (gradientSumSq > 1e-6f) {
        float lambda = C / gradientSumSq; // PBD의 라그랑주 승수(Lagrange Multiplier)

        for (size_t i = 0; i < m_particles.size(); ++i) {
            // [정석 뺄셈] PBD 공식에 따른 올바른 보정량
            float deltaH = -lambda * currentAreas[i];
            m_particles[i].thickness += deltaH;

            if (m_particles[i].thickness < 0.001f) m_particles[i].thickness = 0.001f;
        }
    }
}