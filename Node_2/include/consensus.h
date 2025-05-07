#ifndef CONSENSUS_H
#define CONSENSUS_H

#include <cmath>
#include <cstring>
#define MAX_LUMINAIRES 2
#define PMAX 0.00638

class Consensus {

    public:
    
    float u[MAX_LUMINAIRES];
    float u_av[MAX_LUMINAIRES];
    float u_best[MAX_LUMINAIRES];
    float K[MAX_LUMINAIRES];
    float c[MAX_LUMINAIRES];

    float d;
    float L;

    float n;
    float m;
    float lambda[MAX_LUMINAIRES];
    float rho;

    
public:

    explicit Consensus(float K_[], float d_, float L_, float cost_coeff);

    // Destructor
    ~Consensus(){};

    //Metrics(float Pmax = 1.0); // Constructor with default value

    void consensus_iterate();
    
    bool check_feasibility();
    
    float evaluate_cost();

    float * get_u_best();

    void update_u_av(const float new_u_av[MAX_LUMINAIRES]);

    void update_lambda();
    // Update metrics with new data point
    //void update(float duty_cycle, float current_time, float reference, float measurement);
};












#endif
