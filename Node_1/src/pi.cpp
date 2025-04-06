#include "Arduino.h"
#include "pi.h"


/*Parameters inicialization*/
pi::pi(float _h, float _K, float b_, float Ti_, float Tt_, float KFF_, float G_)
    : h{_h}, K{_K}, b{b_}, Ti{Ti_}, Tt{Tt_}, I{0.0},
      P{0.0},  K_old{0.0}, b_old{0.0}, KFF{KFF_} , G{G_}
      
    {

        parameters_validation(K,b,Ti,Tt,KFF,G);

    } 


void pi::parameters_validation(float K_, float b_, float _Ti, float _Tt, float _KFF, float _G) {

    // Parameters validation


    if(K_ < 0) {

        Serial.println("K cannot be negative");

    }

    if(b_ < 0) {

        Serial.println("b cannot be negative");


    }

    
    if (_Ti <= 0) {

        Serial.println("Ti must be positive!!!");

    }
        


    if (_Tt <= 0) {


        Serial.println("Tt must be positive");

    }   


    if (_KFF < 0) {

        Serial.println("KFF cannot be negative");

    }

    if (_G <= 0) {

        Serial.println("Gain must be positive");

    }
    

}

void pi::set_coefficients(float h_, float K_, float _b, float _Ti, float _Tt, float _G) {

    // Update new parameters
    h = h_;
    K = K_;
    Ti = _Ti;
    b = _b;
    Tt = _Tt;
    G = _G;

    KFF = 1/G;

    parameters_validation(K,b,Ti,Tt,KFF,G);

    // Recompute coefficients to avoid redundant calculations
    bi = K * h / Ti;
    ao = h / Tt; // Anti-windup

}

float pi::compute_control(float r, float y) {

    P = K * (b * r - y);

    // Apply Bumpless Transfer to integral term
    I += K_old * (b_old*r-y) - K * (b*r-y);

    // Feedforward
    uff = KFF*r;

    // Feedback
    ufd = P + I;

    v = ufd + uff;

    // Output saturation
    u = (v < 0) ? 0 : (v > 4095) ? 4095 : v;

    return u;
}


void pi::set_anti_windup(boolean set) {

    ao = set ? h/Tt : 0;

}

void pi::turn_feedback(boolean set) {

    if(set == 0) {

        I = 0.0;
        P = 0.0;
        ufd = 0.0;

    }
    else if (set == 1) {

       ufd = P + I; 

    }

}