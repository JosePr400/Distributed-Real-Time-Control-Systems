#ifndef METRICS_H
#define METRICS_H

class Metrics {

    private:

    // Variables for energy calculation
    float P_max;         // Maximum LED power
    float energy_acc;    // Energy accumulator
    float last_duty;     // Previous duty cycle value
    float last_time;     // Last timestamp
    
    // Variables for visibility error
    float vis_error_acc; // Visibility error accumulator
    int vis_samples;     // Valid sample count
    
    // Variables for flicker calculation
    float flicker_acc;   // Flicker accumulator
    float d_prev1;       // Previous duty cycle value (t-1)
    float d_prev2;       // Older duty cycle value (t-2)
    
public:

    Metrics(float Pmax = 1.0); // Constructor with default value
    
    // Update metrics with new data point
    void update(float duty_cycle, float current_time, float reference, float measurement);
    
    // Reset all accumulators
    void reset();
    
    // Getters for calculated metrics
    float getEnergy() const;
    float getVisibilityError() const;
    float getFlicker() const;
};

#endif // METRICS_H