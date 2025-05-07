#include "consensus.h"

Consensus::Consensus(float K_[], float d_, float L_, float cost_coeff) {
    std::memcpy(K, K_, sizeof(float) * MAX_LUMINAIRES);
    d = d_;
    L = L_;

    for (int i = 0; i < MAX_LUMINAIRES; ++i) {
        u[i] = 0;
        u_av[i] = 0;
        u_best[i] = 0;
        lambda[i] = 0;
        c[i] = 0;
    }
    c[2] = PMAX*cost_coeff; // Assuming cost only for own duty cycle

    rho = 0.2f;

    n = 0;
    for (int i = 0; i < MAX_LUMINAIRES; ++i)
        n += K[i] * K[i];

    m = n - K[0] * K[0];
}

bool Consensus::check_feasibility() {
    float tol = 0.001f;
    if (u[1] < -tol || u[1] > 100 + tol)
        return false;

    float dot_prod = 0;
    for (int i = 0; i < MAX_LUMINAIRES; ++i)
        dot_prod += u[i] * K[i];

    if (dot_prod < L - d - tol)
        return false;

    return true;
}

float Consensus::evaluate_cost() {
    float cost = 0;
    float diff[3], norm2 = 0;

    for (int i = 0; i < MAX_LUMINAIRES; ++i) {
        cost += c[i] * u[i];
        cost += lambda[i] * (u[i] - u_av[i]);
        diff[i] = u[i] - u_av[i];
        norm2 += diff[i] * diff[i];
    }

    cost += 0.5f * rho * norm2;
    return cost;
}

void Consensus::consensus_iterate() {
    float cost_best = 1e6f;
    float z[3], temp_u[3];

    for (int i = 0; i < MAX_LUMINAIRES; ++i)
        z[i] = rho * u_av[i] - lambda[i] - c[i];

    for (int i = 0; i < MAX_LUMINAIRES; ++i)
        u[i] = z[i] / rho;

    if (check_feasibility()) {
        float cost = evaluate_cost();
        if (cost < cost_best) {
            std::memcpy(u_best, u, sizeof(float) * MAX_LUMINAIRES);
            cost_best = cost;
            std::memcpy(u, u_best, sizeof(float) * MAX_LUMINAIRES);
            return;
        }
    }

    float dot_zk = 0, dot_kk = 0;
    for (int i = 0; i < MAX_LUMINAIRES; ++i) {
        dot_zk += z[i] * K[i];
        dot_kk += K[i] * K[i];
    }

    for (int i = 0; i < MAX_LUMINAIRES; ++i)
        u[i] = z[i] / rho - K[i] / n * (d - L + dot_zk / rho);

    if (check_feasibility()) {
        float cost = evaluate_cost();
        if (cost < cost_best) {
            std::memcpy(u_best, u, sizeof(float) * MAX_LUMINAIRES);
            cost_best = cost;
        }
    }

    std::memcpy(u, z, sizeof(float) * MAX_LUMINAIRES);
    u[2] = 0;
    if (check_feasibility()) {
        float cost = evaluate_cost();
        if (cost < cost_best) {
            std::memcpy(u_best, u, sizeof(float) * MAX_LUMINAIRES);
            cost_best = cost;
        }
    }

    std::memcpy(u, z, sizeof(float) * MAX_LUMINAIRES);
    u[2] = 100;
    if (check_feasibility()) {
        float cost = evaluate_cost();
        if (cost < cost_best) {
            std::memcpy(u_best, u, sizeof(float) * MAX_LUMINAIRES);
            cost_best = cost;
        }
    }

    for (int i = 0; i < MAX_LUMINAIRES; ++i)
        u[i] = z[i] / rho - (1 / m) * K[i] * (d - L) + (K[i] / (rho * m)) * (K[0] * z[2] - dot_zk);

    u[2] = 0;
    if (check_feasibility()) {
        float cost = evaluate_cost();
        if (cost < cost_best) {
            std::memcpy(u_best, u, sizeof(float) * MAX_LUMINAIRES);
            cost_best = cost;
        }
    }

    for (int i = 0; i < MAX_LUMINAIRES; ++i)
        u[i] = z[i] / rho - (1 / m) * K[i] * (d - L + 100 * K[0]) + (K[i] / (rho * m)) * (K[0] * z[2] - dot_zk);

    u[2] = 100;
    if (check_feasibility()) {
        float cost = evaluate_cost();
        if (cost < cost_best) {
            std::memcpy(u_best, u, sizeof(float) * MAX_LUMINAIRES);
            cost_best = cost;
        }
    }

    std::memcpy(u, u_best, sizeof(float) * MAX_LUMINAIRES);
}


float* Consensus::get_u_best() { return u_best; }


void Consensus::update_u_av(const float new_u_av[MAX_LUMINAIRES]) {
    for (int i = 0; i < MAX_LUMINAIRES; ++i) {
        u_av[i] = new_u_av[i];
    }
}

// Atualiza o vetor lambda com base na diferença entre u e u_av
void Consensus::update_lambda() {
    for (int i = 0; i < MAX_LUMINAIRES; ++i) {
        lambda[i] += rho * (u[i] - u_av[i]);
    }
}
