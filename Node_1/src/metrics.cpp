#include "metrics.h"
#include <cmath> 

Metrics::Metrics(float Pmax) : 
    P_max(Pmax),
    energy_acc(0.0),
    last_time(0.0),
    last_duty(0.0),
    vis_error_acc(0.0),
    vis_samples(0),
    flicker_acc(0.0),
    d_prev1(0.0),
    d_prev2(0.0) {}

void Metrics::update(float duty_cycle, float current_time, float reference, float measurement) {
    
    // 1. Energy calculation (integration of power over time)
    if(last_time > 0) { // Skip first sample (no delta time)
        float delta_time = current_time - last_time;
        energy_acc += P_max * last_duty * delta_time;
    }
    last_duty = duty_cycle;
    last_time = current_time;

    
    // 2. Visibility error calculation (average of under-shoots)
    if(measurement < reference) {
        vis_error_acc += (reference - measurement);
        vis_samples++;
    }

    // 3. Flicker calculation (detect duty cycle oscillations)
    if(d_prev2 != 0.0) { // Need at least 3 samples to start
        float delta1 = d_prev1 - d_prev2;    // First derivative
        float delta2 = duty_cycle - d_prev1; // Second derivative
        
        // Detect sign change in derivatives
        if((delta1 * delta2) < 0) {
            flicker_acc += fabs(delta1) + fabs(delta2);
        }
        else{
            flicker_acc = 0;
        }
    }
    
    // Update history for next iteration
    d_prev2 = d_prev1;
    d_prev1 = duty_cycle;
}

void Metrics::reset() {
    energy_acc = 0.0;
    vis_error_acc = 0.0;
    vis_samples = 0;
    flicker_acc = 0.0;
    last_time = 0.0;
    d_prev1 = 0.0;
    d_prev2 = 0.0;
}

float Metrics::getEnergy() const {
    return energy_acc;
}

float Metrics::getVisibilityError() const {
    return (vis_samples > 0) ? (vis_error_acc / vis_samples) : 0.0;
}

float Metrics::getFlicker() const {
    return flicker_acc;
}