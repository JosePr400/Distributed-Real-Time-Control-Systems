#ifndef PI_H
#define PI_H

class pi {

private:

  float I, P, K, Ti, Tt, b, h, K_old, b_old, G;
  float ao, bi;
  float u, v, ufd, uff; // Feedback control and feedforward control
  float KFF; // Feedforward gains

public:

    explicit pi(float _h = 0.01, float _K = 1, float b_ = 1, float Ti_ = 1, float Tt_ = 1, float KFF_ = 1, float G_ = 1); // Constructor

    ~pi() {} // Destructor

    float compute_control(float r, float y);
    void housekeep(float r, float y);
    void set_coefficients(float h_, float K_, float _b, float _Ti, float _Tt, float _G);
    void parameters_validation(float K_, float _b, float _Ti, float _Tt, float _KFF, float _G);
    void set_anti_windup(boolean set);
    void turn_feedback(boolean set);
};


inline void pi::housekeep(float r, float y) {

    float e = r - y;
    I += bi * e + ao * (u - v); // Integral update moved here
    K_old = K;
    b_old = b;
  
  }

#endif // PI_H
