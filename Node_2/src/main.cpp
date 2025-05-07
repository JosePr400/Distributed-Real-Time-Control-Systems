#include "Arduino.h"
#include "pi.h"
#include "metrics.h"
#include "mcp2515.h"
#include "pico/multicore.h"
#include "consensus.h"

/*************************************************
                    NODE 2
***************************************************/

#define CANID_U(n)           (0x110 + (n))
#define CANID_REF(n)         (0x120 + (n))
#define CANID_Y(n)           (0x130 + (n))
#define CANID_V(n)           (0x140 + (n))
#define CANID_OCC(n)         (0x150 + (n))
#define CANID_A(n)           (0x160 + (n))
#define CANID_F(n)           (0x170 + (n))
#define CANID_D(n)           (0x180 + (n))
#define CANID_P(n)           (0x190 + (n))
#define CANID_T(n)           (0x200 + (n))
#define CANID_BUFF(n)        (0x210 + (n))
#define CANID_ENERGY(n)      (0x220 + (n))
#define CANID_VIS(n)         (0x230 + (n))
#define CANID_FLICK(n)       (0x240 + (n))
#define CANID_MAN(n)         (0x250 + (n))
#define CANID_BOUND_OCC(n)   (0x260 + (n))
#define CANID_BOUND_UNOCC(n) (0x270 + (n))
#define CANID_COST(n)        (0x280 + (n))
#define CANID_RES(n)         (0x300 + (n))
#define CANID_LED_CONTROL    (0x320)
#define BUFFER_SIZE 6000  // 100Hz * 60s = 6000 samples
#define CALIB_DONE_0 0x10
#define CALIB_DONE_1 0x11
#define CALIB_DONE_2 0x12
#define CALIB_DONE_12 0x13
#define NUM_NODES 3


const uint8_t node_id = 2; // Node ID for this node

struct Sample {
    float y;
    float u;
};

// Restart the system

bool restart = 0;

// Time taken to control, serial comms and can messages
volatile unsigned long control_time_total = 0;
volatile unsigned long serial_time_total = 0;
volatile unsigned long can_time_total = 0;
volatile unsigned long control_cycles = 0;
volatile unsigned long can_messages = 0;

unsigned long lastConsensusTime = 0;
const unsigned long CONSENSUS_INTERVAL = 500;

// Metrics variables
float E[3] = {0.0,0.0,0.0};
float V[3] = {0.0,0.0,0.0};
float F[3] = {0.0,0.0,0.0};

long long lastSendAll = 0;
float elapsed[3] = {0.0,0.0,0.0};

// Variable to turn off and on feedback
bool is_active[3] = {1,1,1};

// Variable to turn on or off controler, manual control
bool manual[3] = {0,0,0};

// Variable to set the occupancy
bool is_occupied[3] = {1,1,1};

// Variable to turn off and on anti-windup
bool anti_windup[3] = {1,1,1};

// Variables to stream y and u in real time
bool stream_y[3] = {0,0,0};
bool stream_u[3] = {0,0,0};


// Variable to compute voltage at LDR
float v_ldr[3] = {0,0,0};

// External disturbance d
float d[3] = {0.0,0.0,0.0};

// Instantaneous power
float power[3] = {0.0,0.0,0.0};

// Variable to get the elpased time
unsigned long start_time = 0;  // Capture startup time

// Timing control
unsigned long previous_control_time = 0;
const unsigned long CONTROL_INTERVAL = 10; // 10ms = 100Hz

// Hardware constants
const int LED_PIN = 15;
const int DAC_RANGE = 4096;
const float P_MAX = 0.00638;
int sensorValue = 0.0;  

// System components
pi pi_control(0.01,1,1,1,1,1,1,1,1,1);
Metrics system_metrics(P_MAX);
MCP2515 can0(spi0, 17, 19, 16, 18, 10000000);
mutex_t mut_CAN;

// Data logging
Sample data_buffer0[BUFFER_SIZE],data_buffer1[BUFFER_SIZE],data_buffer2[BUFFER_SIZE];
int write_index = 0;
bool buffer_full = false;
int write_index_1 = 0; 
bool buffer_full_1 = false;
int write_index_2 = 0; 
bool buffer_full_2 = false;

// Control variables
float reference[3] = {15,15,15};  // Initial reference
float u[3] = {0,0,0}; //Initial control signal
float y_filtered[3] = {0.0,0.0,0.0};
const float ALPHA = 0.1;  // Filter coefficient
int pwm[3] = {0,0,0}; // Pulse module width to control the led
float G[3] = {0,0,0}; // System gain

// Lux variables to calculate the inicial gain

float L2 = 0.0;

float L1 = 0.0;

// Lower and upper bounds for the occupancy
float cost_coeff[3] = {1,1,1};
float lower_bound_occ[3] = {20,20,20};   
float lower_bound_unocc[3] = {5,5,5}; 

// Functions
float convertToLux(int adcValue); // Convert to lux values
void Hub(String input); // Interface
void sendFloat(uint32_t canId, float value1, float value2, float value3); // Send a value to other node
void sendBuffer(uint32_t canId, Sample data_buffer[BUFFER_SIZE]); // Send the buffer to other node
void sendoneBool(uint32_t canId, bool value);
void sendBool(uint32_t canId, bool value1, bool value2, bool value3);
void sendAllValues(); // Always sending values
void readAllValues(); // Always reading values
void setRemoteLed(int node, bool on); // Set the led on or off
void CalibrationAll(); // Calibration of the system
void sendCalibrationDone(uint8_t id); // Send calibration done signal
void waitForCalibrationDone(uint8_t expected_id); // Wait for calibration done signal
void apply_consensus(); // Apply consensus algorithm to the system

void setup() {
    
    
    start_time = millis();

    if (can0.reset() == MCP2515::ERROR_OK && can0.setBitrate(CAN_500KBPS) == MCP2515::ERROR_OK) {
        Serial.println("MCP2515 Initialized.");
        can0.setNormalMode();
    } else {
        Serial.println("MCP2515 Initialization Failed!");
        while (1);
    }


    mutex_init(&mut_CAN); 

    // Node 1 waits for calibration done signal from node 0
    waitForCalibrationDone(CALIB_DONE_12);
    
    CalibrationAll();

    // Node 1 sends calibration signal concluded
    sendCalibrationDone(CALIB_DONE_2);

    pi_control.set_coefficients(0.01,1,0.3,0.08,sqrt(0.08),G[2],G[0],G[1],L1);

    
    analogWrite(LED_PIN, 0);
    delay(1000);

    multicore_launch_core1(sendAllValues); // Using Core 1

}

void loop() { // Core 0


    unsigned long current_time = millis();
    unsigned long time_start = micros();

    // Read data from the network
    readAllValues();
    
    mutex_enter_blocking(&mut_CAN);
    // Handle serial commands
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        Hub(input);
    }
    mutex_exit(&mut_CAN);

    serial_time_total += micros() - time_start;

    // Calculate powers
    power[0] = P_MAX * (u[0]/(DAC_RANGE-1.0f));
    power[1] = P_MAX * (u[1]/(DAC_RANGE-1.0f));
    power[2] = P_MAX * (u[2]/(DAC_RANGE-1.0f));

    // Calculate Energy
    //E[0] = system_metrics.getEnergy();
    //E[1] = system_metrics.getEnergy();
    E[2] = system_metrics.getEnergy();

    // Calculate Visibility Error 
    //V[0] = system_metrics.getVisibilityError();
    //V[1] = system_metrics.getVisibilityError();
    V[2] = system_metrics.getVisibilityError();

    // Calculate Flicker
    //F[0] = system_metrics.getFlicker();
    //F[1] = system_metrics.getFlicker();
    F[2] = system_metrics.getFlicker();

    readAllValues();

    // Execute control at 100Hz
    if (current_time - previous_control_time >= CONTROL_INTERVAL) {

        unsigned long control_start = micros();
        previous_control_time = current_time;
        sensorValue = analogRead(A0);
        v_ldr[2] = sensorValue * (3.3 / 4095.0);
        float y_raw = convertToLux(sensorValue);
                    
        // Apply exponential filter
        y_filtered[2] = ALPHA * y_raw + (1 - ALPHA) * y_filtered[2];

        // Check if controller is on/off
        if(is_active[2] == 0){

            if(manual[2] == 0) {

                pi_control.turn_feedback(is_active[1]);
                pi_control.set_anti_windup(anti_windup[1]); 
                u[2] = pi_control.compute_control(reference[1], y_filtered[1], u[0], u[1]);

                pwm[2] = (int)u[2];   
                d[2] = y_filtered[2] - G[2]*pwm[2];    
            }

            else if (manual[2] == 1) {

                pwm[2] = (int)u[2];   
                d[2] = y_filtered[2] - G[2]*pwm[2];

            }

        }

        else if (is_active[2] == 1) {

            pi_control.turn_feedback(is_active[2]);
            pi_control.set_anti_windup(anti_windup[2]); 
            u[2] = pi_control.compute_control(reference[2], y_filtered[2],u[0], u[1]);
            pwm[2] = (int)u[2];

            d[2] = y_filtered[2] - G[2]*pwm[2];

        }

        if(stream_y[0] == 1) {
            Serial.print("s y 0 ");
            Serial.print(y_filtered[0],2);
            Serial.print(" ");
            Serial.println(millis());
        }

        if (stream_u[0] == 1) {
            Serial.print("s u 0 ");
            Serial.print(u[0],2);
            Serial.print(" ");
            Serial.println(millis());
        }

        else if (stream_y[1] == 1) {
            Serial.print("s y 1 ");
            Serial.print(y_filtered[1],2);
            Serial.print(" ");
            Serial.println(millis());
        }

        else if(stream_u[1] == 1) {
            Serial.print("s u 1 ");
            Serial.print(u[1],2);
            Serial.print(" ");
            Serial.println(millis());
        }

        else if (stream_y[2] == 1) {
            Serial.print("s y 2 ");
            Serial.print(y_filtered[2],2);
            Serial.print(" ");
            Serial.println(millis());
        }


        else if(stream_u[2] == 1) {
            Serial.print("s u 2 ");
            Serial.print(u[2],2);
            Serial.print(" ");
            Serial.println(millis());
        }

        analogWrite(LED_PIN, pwm[2]);

        pi_control.housekeep(reference[2], y_filtered[2]);

        // Update metrics
        system_metrics.update(pwm[2]/(DAC_RANGE-1.0f), current_time, reference[2], y_filtered[2]);

        control_time_total += micros() - control_start;
        control_cycles++;

        Serial.println(u[1]);

    }

    if (current_time - lastConsensusTime >= CONSENSUS_INTERVAL) {
        lastConsensusTime = current_time;
        apply_consensus();
    }

    // Store data in buffer
    noInterrupts();
    data_buffer1[write_index_1] = {
        .y = y_filtered[1],
        .u = u[1],
    };
    write_index_1 = (write_index_1 + 1) % BUFFER_SIZE;
    if(write_index_1 == 0) buffer_full_1 = true;
    interrupts();
    
}

void sendCalibrationDone(uint8_t id) {
    struct can_frame frame;
    frame.can_id = id;
    frame.can_dlc = 1;
    frame.data[0] = 1;

    can0.sendMessage(&frame);
}

void waitForCalibrationDone(uint8_t expected_id) {
    struct can_frame frame;

    while (true) {
        if (can0.readMessage(&frame) == MCP2515::ERROR_OK) {

            if (frame.can_dlc >= 2) {
                uint8_t target_node = frame.data[0]; 
                uint8_t state = frame.data[1];        
    
                if (target_node == node_id) {
                    if (state == 1){
                        analogWrite(LED_PIN, DAC_RANGE); 
                        delay(3000);
                    }
                    else{
                        analogWrite(LED_PIN, 0);
                        delay(3000);
                    }
                }
            }

            if (frame.can_id == expected_id && frame.data[0] == 1) {
                Serial.print("Received calib done signal from ID 0x");
                Serial.println(expected_id, HEX);
                break;
            }
        }
        delay(10);
    }
}


void setRemoteLed(int node, bool on) {
    struct can_frame frame;
    frame.can_id = CANID_LED_CONTROL;
    frame.can_dlc = 2;
    frame.data[0] = node;    
    frame.data[1] = on ? 1 : 0;  
    can0.sendMessage(&frame);
    delay(2000);
    
}

void CalibrationAll(){
    
    for (int node = 0; node < NUM_NODES; node++) {
        analogWrite(LED_PIN, 0);  
        
        setRemoteLed(0, false);
        if(NUM_NODES>2)
            setRemoteLed(1, false);
        delay(3000);
          
        if (node == 0){
          L1 = analogRead(A0);
          L1 = convertToLux(L1);
        }

        if (node == node_id){
            analogWrite(LED_PIN, DAC_RANGE);
            delay(3000);
        }
        else{
            setRemoteLed(node, true);
            delay(5000);
        }

        L2 = analogRead(A0);
        L2 = convertToLux(L2);

        G[node] = (L2 - L1) / DAC_RANGE;
        Serial.print("G["); Serial.print(node); Serial.print("] = ");
        Serial.println(G[node],6);
        delay(2000);
    }
}


void Hub(String input) {

    input.trim();

    // Set duty cycle 'u <i> <val>'
    if (input.startsWith("u ")) {

        int i = input.substring(2).toInt();
        float val = input.substring(4).toFloat();
        if ((i == 0) && val >= 0) {
            u[0] = val;
            Serial.println("ack");
            is_active[0] = 0;
            manual[0] = 1; // User has the control of the led, so, controller is off
        }
        else if(i == 1 && val >= 0) {
            u[1] = val;
            Serial.println("ack");
            is_active[1] = 0;
            manual[1] = 1;
        }
        else if(i == 2 && val >= 0) {
            u[2] = val;
            Serial.println("ack");
            is_active[2] = 0;
            manual[2] = 1;
        }

         else {
            Serial.println("err");
        } 

    }

    // Get duty cycle 'g u <i>'
    else if (input.startsWith("g u")) {

        int i = input.substring(4).toInt();
        if (i == 0) {
            Serial.print("u 0 ");
            Serial.println(u[0], 1);
        } 
        else if(i == 1) {
            Serial.print("u 1 ");
            Serial.println(u[1], 1);
            // Reiceive value
        }
        else if(i == 2) {
            Serial.print("u 2 ");
            Serial.println(u[2], 1);
            // Reiceive value
        }
        else {
            Serial.println("err");
        } 

    }
    
    // Set reference illuminance: 'r <i> <val>'
    else if (input.startsWith("r ")) {

        int i = input.substring(2).toInt();
        float val = input.substring(4).toFloat();
        
        if (i == 0 && val >= 0) {  // Only node 0 supported initially
            reference[0] = val;
            Serial.println("ack ");
        } 
        else if(i == 1 && val >= 0) {
            reference[1] = val;
            Serial.println("ack ");
            // Send value
        }
        else if(i == 2 && val >= 0) {
            reference[2] = val;
            Serial.println("ack ");
            // Send value
        }
        else {
            Serial.println("err");
        }
    }

    // Get reference illuminance: 'g r <i>'
    else if (input.startsWith("g r ")) {

        int i = input.substring(4).toInt();
        if (i == 0) {
            Serial.print("r 0 ");
            Serial.print(reference[0], 1);
            Serial.println(" [LUX]");
        } 
        else if(i == 1) {
            Serial.print("r 1 ");
            Serial.print(reference[1], 1);
            Serial.println(" [LUX]");
            // Reiceive value
        }
        else if(i == 2) {
            Serial.print("r 2 ");
            Serial.print(reference[2], 1);
            Serial.println(" [LUX]");
            // Reiceive value
        }
        else {
            Serial.println("err");
        }
    }

    // Measure the actual illuminance 'g y <i>'
    else if (input.startsWith("g y ")) {

        int i = input.substring(4).toInt();
        if (i == 0) {
            Serial.print("y 0 ");
            Serial.print(y_filtered[0], 1);
            Serial.println(" [LUX]");
        }
        else if(i == 1){
            // Reiceive value
            Serial.print("y 1 ");
            Serial.print(y_filtered[1], 1);
            Serial.println(" [LUX]"); 
        }
        else if(i == 2) {
            // Receive value
            Serial.print("y 2 ");
            Serial.print(y_filtered[2], 1);
            Serial.println(" [LUX]");
        } 
        else {
            Serial.println("err");
        }
    }


    // Measure the actual voltage 'g v <i>'
    else if (input.startsWith("g v ")) {

        int i = input.substring(4).toInt();
        if (i == 0) {
            Serial.print("v 0 ");
            Serial.print(v_ldr[0], 2);
            Serial.println(" [VOLTS]");
        } 
        else if(i == 1){
            // Reiceive value
            Serial.print("v 1 ");
            Serial.print(v_ldr[1], 2);
            Serial.println(" [VOLTS]");
        }
        else if(i == 2) {
            // Reiceive value
            Serial.print("v 2 ");
            Serial.print(v_ldr[2], 2);
            Serial.println(" [VOLTS]"); 
        }
        else {
            Serial.println("err");
        }
    }

    //Set the current occupancy 'o <i> <val>'
    else if (input.startsWith("o ")) {

        int i = input.substring(2).toInt();
        int val = input.substring(4).toInt();
        if (i == 0 && (val == 1 || val == 0)) {
            Serial.println("ack");
            if(val == 1){
                is_occupied[0] = 1; // Ocuppied
                reference[0] = 15;
            }
            else if (val == 0){
                is_occupied[0] = 0; // Unoccupied 
                reference[0] = 0;
            }
        }
        else if(i == 1 && (val == 1 || val == 0)) {
            Serial.println("ack");
            if(val == 1){
                is_occupied[1] = 1; // Ocuppied
                reference[1] = 15;
                // Send value
            }
            else if (val == 0){
                is_occupied[1] = 0; // Unoccupied 
                reference[1] = 0;
                // Send value
            }
        }
        else if(i == 2 && (val == 1 || val == 0)) {
            Serial.println("ack");
            if(val == 1){
                is_occupied[2] = 1; // Ocuppied
                reference[2] = 15;
                // Send value
            }
            else if (val == 0){
                is_occupied[2] = 0; // Unoccupied 
                reference[2] = 0;
                // Send value
            }
        }
        else {
            Serial.println("err");
        }
    }

    // Get the current occupancy state of desk 'g o <i>'
    else if (input.startsWith("g o ")) {

        int i = input.substring(4).toInt();
        if (i == 0) {
            Serial.print("o 0 ");
            Serial.println(is_occupied[0], 1);
        }
        else if(i == 1) {
            // Reiceive value
            Serial.print("o 1 ");
            Serial.println(is_occupied[1], 1); 
        } 
        else if(i == 2) {
            // Reiceive value
            Serial.print("o 2 ");
            Serial.println(is_occupied[2], 1); 
        }
        else {
            Serial.println("err");
        }
    }

    // Set anti-windup on/off on at desk 'a <i> <val>'
    else if (input.startsWith("a ")) {

        int i = input.substring(2).toInt();
        int val = input.substring(4).toInt();
        if (i == 0 && (val == 1 || val == 0)) {
            Serial.println("ack");
            if(val == 1){
                 anti_windup[0] = 1; // Anti-Windup on 
            }
            else if (val == 0) {
                anti_windup[0] = 0; // Anti-Windup off
            }
        }
        else if(i == 1 && (val == 1 || val == 0)) {
            Serial.println("ack");
            if(val == 1){
                 anti_windup[1] = 1; // Anti-Windup on 
                 // Send value
            }
            else if (val == 0) {
                anti_windup[1] = 0; // Anti-Windup off
                // Send value
            }
        }
        else if(i == 2 && (val == 1 || val == 0)) {
            Serial.println("ack");
            if(val == 1){
                 anti_windup[2] = 1; // Anti-Windup on 
                 // Send value
            }
            else if (val == 0) {
                anti_windup[2] = 0; // Anti-Windup off
                // Send value
            }
        }
        else {
            Serial.println("err");
        }
    }

    // Get anti-windup state of desk 'g a <i>'
    else if (input.startsWith("g a ")) {

        int i = input.substring(4).toInt();
        if (i == 0) {
            Serial.print("a 0 ");
            Serial.println(anti_windup[0], 1);
        } 
        else if(i == 1) {
            // Reiceive values
            Serial.print("a 1 ");
            Serial.println(anti_windup[1], 1); 
        }
        else if(i == 2) {
            // Reiceive values
            Serial.print("a 2 ");
            Serial.println(anti_windup[2], 1); 
        }
        else {
            Serial.println("err");
        }
    }

    // Set feedback control on/off on desk 'f <i> <val>'
    else if (input.startsWith("f ")) {

        int i = input.substring(2).toInt();
        float val = input.substring(4).toInt();
        if (i == 0 && (val == 1 || val == 0)) {
            Serial.println("ack");
            if(val == 1) {
                is_active[0] = 1; // Feedback on
                manual[0] = 0;
            }
            else if (val == 0) {
                is_active[0] = 0; // Feedback off
                manual[0] = 0;
            }
        }
        else if(i == 1 && (val == 1 || val == 0)) {
            Serial.println("ack");
            if(val == 1) {
                is_active[1] = 1; // Feedback on
                manual[1] = 0;
                // Send values
            }
            else if (val == 0) {
                is_active[1] = 0; // Feedback off
                manual[1] = 0;
                // Send values
            }
        } 
        else if(i == 2 && (val == 1 || val == 0)) {
            Serial.println("ack");
            if(val == 1) {
                is_active[2] = 1; // Feedback on
                manual[2] = 0;
                // Send values
            }
            else if (val == 0) {
                is_active[2] = 0; // Feedback off
                manual[2] = 0;
                // Send values
            }
        } 
        else {
            Serial.println("err");
        } 

    }

    // Get feedback control state of desk 'g f <i>'
    else if (input.startsWith("g f ")) {

        int i = input.substring(4).toInt();
        if (i == 0) {
            Serial.print("f 0 ");
            Serial.println(is_active[0], 1);
        }
        else if(i == 1) {
            // Reiceive values
            Serial.print("f 1 ");
            Serial.println(is_active[1], 1);
        } 
        else if(i == 2) {
            // Reiceive values
            Serial.print("f 2 ");
            Serial.println(is_active[2], 1);
        } 
        else {
            Serial.println("err");
        }
    }


    // Get current external illuminance of desk 'g d <i>'
    else if (input.startsWith("g d ")) {

        int i = input.substring(4).toInt();
        if (i == 0) {
            Serial.print("d 0 ");
            Serial.println(d[0], 1);
        } 
        else if(i == 1){
            // Reiceive values
            Serial.print("d 1 ");
            Serial.println(d[1], 1); 
        }
        else if(i == 2){
            // Reiceive values
            Serial.print("d 2 ");
            Serial.println(d[2], 1); 
        }
        else {
            Serial.println("err");
        }
    }

    // Get instantaneous power consumption of desk 'g p <i>'
    else if (input.startsWith("g p ")) {
        int i = input.substring(4).toInt();
        if (i == 0) {
            Serial.print("p 0 ");
            Serial.print(power[0], 5);
            Serial.println(" [W]");
        }
        else if(i == 1){
            // Reiceive values
            Serial.print("p 1 ");
            Serial.print(power[1], 5);
            Serial.println(" [W]");
        }
        else if(i == 2){
            // Reiceive values
            Serial.print("p 2 ");
            Serial.print(power[2], 5);
            Serial.println(" [W]");
        }
        else {
            Serial.println("err");
        }
    }

    // Get the elapsed time since the last restart 'g t <i>' 
    else if (input.startsWith("g t ")) {
        int i = input.substring(4).toInt();
        if (i == 0) {
            elapsed[0] = (millis() - start_time) / 1000.0f;
            Serial.print("t 0 ");
            Serial.print(elapsed[0], 2);
            Serial.println(" [s]");
        } 
        else if(i == 1){
            // Reiceive values
            elapsed[1] = (millis() - start_time) / 1000.0f;
            Serial.print("t 1 ");
            Serial.print(elapsed[1], 2);
            Serial.println(" [s]");
        }
        else if(i == 2){
            // Reiceive values
            elapsed[2] = (millis() - start_time) / 1000.0f;
            Serial.print("t 2 ");
            Serial.print(elapsed[2], 2);
            Serial.println(" [s]");
        }
        else {
            Serial.println("err");
        }
    }


    // Start the stream of the real-time variable <x> of desk 's <x> <i>'. 
    else if (input.startsWith("s ")) {
        String var = input.substring(2, 3);
        int i = input.substring(4).toInt();
        
        if(i == 0) {
            if(var == "y") {
                stream_y[0] = 1;
                Serial.println("ack");
            }
            else if(var == "u") {
                stream_u[0] = 1;
                Serial.println("ack");
            }
            else {
                Serial.println("err: invalid variable");
            }
        } 
        else if(i == 1){
            if(var == "y") {
                stream_y[1] = 1;
                Serial.println("ack");
            }
            else if(var == "u") {
                stream_u[1] = 1;
                Serial.println("ack");
            }
            else {
                Serial.println("err: invalid variable");
            }
        }
        else if(i == 2){
            if(var == "y") {
                stream_y[2] = 1;
                Serial.println("ack");
            }
            else if(var == "u") {
                stream_u[2] = 1;
                Serial.println("ack");
            }
            else {
                Serial.println("err: invalid variable");
            }
        }
        else {
            Serial.println("err: invalid node");
        }
    }

    // Stop the stream of the real-time variable <x> of desk 'S <x> <i>'
    else if (input.startsWith("S ")) {
        String var = input.substring(2, 3);
        int i = input.substring(4).toInt();
        
        if(i == 0) {
            if(var == "y") {
                stream_y[0] = 0;
                Serial.println("ack");
            }
            else if(var == "u") {
                stream_u[0] = 0;
                Serial.println("ack");
            }
            else {
                Serial.println("err: invalid variable");
            }
        } 
        else if(i == 1){
            if(var == "y") {
                stream_y[1] = 0;
                Serial.println("ack");
            }
            else if(var == "u") {
                stream_u[1] = 0;
                Serial.println("ack");
            }
            else {
                Serial.println("err: invalid variable");
            }
        }
        else if(i == 2){
            if(var == "y") {
                stream_y[2] = 0;
                Serial.println("ack");
            }
            else if(var == "u") {
                stream_u[2] = 0;
                Serial.println("ack");
            }
            else {
                Serial.println("err: invalid variable");
            }
        }
        else {
            Serial.println("err: invalid node");
        }
    }

    // Get the last minute buffer of the variable <x> of the desk 'g b <x> <i>'
    else if (input.startsWith("g b ")) {
        // Parse command
        String rest = input.substring(4);
        int spacePos = rest.indexOf(' ');
        if (spacePos == -1) {
            Serial.println("err: invalid format");
            return;
        }
        String var = rest.substring(0, spacePos);
        String nodeStr = rest.substring(spacePos + 1);
        int i = nodeStr.toInt();
    
        // Validation
        if (i != 0 && i != 1 && i != 2) {
            Serial.println("err: invalid node");
            return;
        }

        if (var != "y" && var != "u") {
            Serial.println("err: invalid variable");
            return;
        }
    
        // Determine buffer parameters
        int num_elements = buffer_full ? BUFFER_SIZE : write_index;
        int start_index = buffer_full ? write_index : 0;
    
        if(i == 0) {
            Serial.print("b ");
            Serial.print(var);
            Serial.print(" 0 ");
            // Stream values in chronological order
            for(int count = 0; count < num_elements; count++) {
                int index = (start_index + count) % BUFFER_SIZE;
                
                if(var == "y") {
                    Serial.print(data_buffer0[index].y, 3);  // 3 decimal places for illuminance
                } else {
                    Serial.print(data_buffer0[index].u);   // Integer PWM values
                }
                
                if(count < num_elements - 1) Serial.print(",");
        }
        Serial.println();  // End with newline
       }
        else if(i == 1){
            Serial.print("b ");
            Serial.print(var);
            Serial.print(" 1 ");
            // Stream values in chronological order
            for(int count = 0; count < num_elements; count++) {
                int index = (start_index + count) % BUFFER_SIZE;
                
                if(var == "y") {
                    Serial.print(data_buffer1[index].y, 3);  // 3 decimal places for illuminance
                } else {
                    Serial.print(data_buffer1[index].u);   // Integer PWM values
                }
                
                if(count < num_elements - 1) Serial.print(",");
            }
        Serial.println();  // End with newline
        }
        else if(i == 2){
            Serial.print("b ");
            Serial.print(var);
            Serial.print(" 2 ");
            // Stream values in chronological order
            for(int count = 0; count < num_elements; count++) {
                int index = (start_index + count) % BUFFER_SIZE;
                
                if(var == "y") {
                    Serial.print(data_buffer2[index].y, 3);  // 3 decimal places for illuminance
                } else {
                    Serial.print(data_buffer2[index].u);   // Integer PWM values
                }
                
                if(count < num_elements - 1) Serial.print(",");
            }
        Serial.println();  // End with newline
        }
            
    }

    // Get energy: 'g E <i>'
    else if (input.startsWith("g E ")) {

        int i = input.substring(4).toInt();
        if (i == 0) {
            Serial.print("E 0 ");
            Serial.print(E[0], 3);
            Serial.println(" [j]");
        } 
        else if(i == 1){
            // Reiceive values
            Serial.print("E 1 ");
            Serial.print(E[1], 3);
            Serial.println(" [j]");
        }
        else if(i == 2){
            // Reiceive values
            Serial.print("E 2 ");
            Serial.print(E[2], 3);
            Serial.println(" [j]");
        }
        else {
            Serial.println("err");
        }
    }
    // Get visibility error: 'g V <i>'
    else if (input.startsWith("g V ")) {

        int i = input.substring(4).toInt();
        if (i == 0) {
            Serial.print("V 0 ");
            Serial.print(V[0], 3);
            Serial.println(" [LUX]");
        }
        else if(i == 1){
            // Reiceive values
            Serial.print("V 1 ");
            Serial.print(V[1], 3);
            Serial.println(" [LUX]");
        }
        else if(i == 2){
            // Reiceive values
            Serial.print("V 2 ");
            Serial.print(V[2], 3);
            Serial.println(" [LUX]");  
        }
        else {
            Serial.println("err");
        }
    }
    // Get flicker: 'g F <i>'
    else if (input.startsWith("g F ")) {

        int i = input.substring(4).toInt();
        if (i == 0) {
            Serial.print("F 0 ");
            Serial.print(F[0], 3);
            Serial.println(" [s^(-1)]");
        }
        else if(i == 1){
            // Reiceive values
            Serial.print("F 1 ");
            Serial.print(F[1], 3);
            Serial.println(" [s^(-1)]");
        } 
        else if(i == 2){
            // Reiceive values
            Serial.print("F 2 ");
            Serial.print(F[2], 3);
            Serial.println(" [s^(-1)]");
        } 
        else {
            Serial.println("err");
        }
    }

    else if (input == "g times") {

        Serial.println("\n--- System Timing (seconds) ---");
        Serial.print("Control Time/cycle: ");
        Serial.println(control_time_total / (1000000.0 * control_cycles), 6);
        
        Serial.print("Serial Time/cycle: ");
        Serial.println(serial_time_total / (1000000.0 * control_cycles), 6);
        
        Serial.print("CAN Time/message: ");
        Serial.println(can_time_total / (1000000.0 * can_messages), 6);
        
        Serial.print("\nTotal Control Time: ");
        Serial.println(control_time_total / 1000000.0, 3);
        Serial.print("Total Serial Time: ");
        Serial.println(serial_time_total / 1000000.0, 3);
        Serial.print("Total CAN Time: ");
        Serial.println(can_time_total / 1000000.0, 3);
    }


    else if (input.startsWith("g O ")) {
        int i = input.substring(4).toInt();
        if (i >= 0 && i < 3) {
            Serial.printf("O %d %.1f [LUX]\n", i, lower_bound_occ[i]);
        } else {
            Serial.println("err");
        }
    }
    
    else if (input.startsWith("O ")) {
        int i = input.substring(2).toInt();
        float val = input.substring(4).toFloat();
        if (i >= 0 && i < 3 && val >= 0) {
            lower_bound_occ[i] = val;
            Serial.println("ack");
        } else {
            Serial.println("err");
        }
    }
    
    else if (input.startsWith("g U ")) {
        int i = input.substring(4).toInt();
        if (i >= 0 && i < 3) {
            Serial.printf("U %d %.1f [LUX]\n", i, lower_bound_unocc[i]);
        } else {
            Serial.println("err");
        }
    }
    
    else if (input.startsWith("U ")) {
        int i = input.substring(2).toInt();
        float val = input.substring(4).toFloat();
        if (i >= 0 && i < 3 && val >= 0) {
            lower_bound_unocc[i] = val;
            Serial.println("ack");
        } else {
            Serial.println("err");
        }
    }
    
    else if (input.startsWith("g L ")) {
        int i = input.substring(4).toInt();
        if (i >= 0 && i < 3) {
            Serial.printf("g L %d %.1f [LUX]\n", i, lower_bound_occ[i]);
            Serial.printf("g L %d %.1f [LUX]\n", i, lower_bound_unocc[i]);
        } else {
            Serial.println("err");
        }
    }
    
    else if (input.startsWith("g C ")) {
        int i = input.substring(4).toInt();
        if (i >= 0 && i < 3) {
            Serial.printf("C %d %.2f\n", i, cost_coeff[i]);
        } else {
            Serial.println("err");
        }
    }
    
    else if (input.startsWith("C ")) {
        int i = input.substring(2).toInt();
        float val = input.substring(4).toFloat();
        if (i >= 0 && i < 3 && val >= 0) {
            cost_coeff[i] = val;
            Serial.println("ack");
        } else {
            Serial.println("err");
        }
    }
    
    else if (input == "R") {
        restart = 1;
    }

    else {
        Serial.println("err: unknown command");
    }

    


}

float convertToLux(int adcValue) {
    const float m = -0.8;
    const float R_ldr = 1700000.0;
    const float Ro = R_ldr / pow(10, m);
    const float b = log10(Ro);

    float voltage = adcValue * (3.3 / 4095.0);
    float R_fixed = 10000.0;
    float LDR_resistance = R_fixed * (3.3 - voltage) / voltage;

    float log_LDR = log10(LDR_resistance);
    float log_LUX = (log_LDR - b) / m;
    return pow(10, log_LUX);
}

void sendFloat(uint32_t canId, float value1, float value2, float value3) {
    // Frame for subIndex=0
    {
        struct can_frame frame;
        frame.can_id  = canId; // same ID for all
        frame.can_dlc = 5;     // 1 byte subIndex + 4 bytes float
        frame.data[0] = 0;     // subIndex
        memcpy(&frame.data[1], &value1, 4);
        can0.sendMessage(&frame);
        delayMicroseconds(100);
    }

    // Frame for subIndex=1
    {
        struct can_frame frame;
        frame.can_id  = canId;
        frame.can_dlc = 5;
        frame.data[0] = 1;
        memcpy(&frame.data[1], &value2, 4);
        can0.sendMessage(&frame);
        delayMicroseconds(100);
    }

    // Frame for subIndex=2
    {
        struct can_frame frame;
        frame.can_id  = canId;
        frame.can_dlc = 5;
        frame.data[0] = 2;
        memcpy(&frame.data[1], &value3, 4);
        can0.sendMessage(&frame);
        delayMicroseconds(100);
    }

}

void sendBool(uint32_t canId, bool value1, bool value2, bool value3) {
    // Frame for subIndex = 0
    {
        struct can_frame frame;
        frame.can_id  = canId; 
        frame.can_dlc = 2;     // 1 byte for subIndex, 1 byte for bool
        frame.data[0] = 0;     // subIndex
        frame.data[1] = value1;
        can0.sendMessage(&frame);
        delayMicroseconds(100);
    }

    // Frame for subIndex = 1
    {
        struct can_frame frame;
        frame.can_id  = canId;
        frame.can_dlc = 2;
        frame.data[0] = 1;     // subIndex
        frame.data[1] = value2;
        can0.sendMessage(&frame);
        delayMicroseconds(100);
    }

    // Frame for subIndex = 2
    {
        struct can_frame frame;
        frame.can_id  = canId;
        frame.can_dlc = 2;
        frame.data[0] = 2;     // subIndex
        frame.data[1] = value3;
        can0.sendMessage(&frame);
        delayMicroseconds(100); 
    }
}


void sendBuffer(uint32_t canId, Sample data_buffer[BUFFER_SIZE]) {
    for (int i = 0; i < BUFFER_SIZE; i++) {
    
        struct can_frame frame;
        frame.can_id  = canId;
        frame.can_dlc = 8; 
        memcpy(&frame.data[0], &data_buffer[i].y, 4);
        memcpy(&frame.data[4], &data_buffer[i].u, 4);
        can0.sendMessage(&frame);

        delayMicroseconds(100);
    }
}

void sendoneBool(uint32_t canId, bool value) {
    struct can_frame frame;
    frame.can_id  = canId;
    frame.can_dlc = 1; // apenas 1 byte
    frame.data[0] = value;
    can0.sendMessage(&frame);
    delayMicroseconds(100);
}

void sendAllValues() {

    while (true) {

        mutex_enter_blocking(&mut_CAN);
        if (millis() - lastSendAll >= 500) {
            lastSendAll = millis();
            sendFloat(CANID_U(1), u[0],u[1],u[2]);
            sendFloat(CANID_REF(1), reference[0],reference[1],reference[2]);
            sendFloat(CANID_Y(1), y_filtered[0],y_filtered[1],y_filtered[2]);
            sendFloat(CANID_V(1),v_ldr[0],v_ldr[1],v_ldr[2]);
            sendBool(CANID_A(1),anti_windup[0],anti_windup[1],anti_windup[2]);
            sendBool(CANID_OCC(1),is_occupied[0],is_occupied[1],is_occupied[2]);
            sendBool(CANID_F(1),is_active[0],is_active[1],is_active[2]);
            sendFloat(CANID_D(1),d[0],d[1],d[2]);
            sendFloat(CANID_P(1),power[0],power[1],power[2]);
            sendFloat(CANID_T(1),elapsed[0],elapsed[1],elapsed[2]);
            //sendBuffer(CANID_BUFF(1),data_buffer1);
            sendFloat(CANID_ENERGY(1),E[0],E[1],E[2]);
            sendFloat(CANID_VIS(1),V[0],V[1],V[2]);
            sendFloat(CANID_FLICK(1),F[0],F[1],F[2]);
            sendBool(CANID_MAN(1),manual[0],manual[1],manual[2]);
            sendFloat(CANID_BOUND_OCC(1),lower_bound_occ[0],lower_bound_occ[1],lower_bound_occ[2]);
            sendFloat(CANID_BOUND_UNOCC(1),lower_bound_unocc[0],lower_bound_unocc[1],lower_bound_unocc[2]);
            sendFloat(CANID_COST(1),cost_coeff[0],cost_coeff[1],cost_coeff[2]);
            //sendoneBool(CANID_RES(1),restart);

        }
        mutex_exit(&mut_CAN);
        delayMicroseconds(100);
    }
}


void readAllValues()
{
    struct can_frame rx;

    mutex_enter_blocking(&mut_CAN);
    while (can0.readMessage(&rx) == MCP2515::ERROR_OK)
    {   
        uint16_t baseId = rx.can_id;
        uint8_t subIndex = rx.data[0];


            switch (baseId) {

            case CANID_U(0):
                if (rx.can_dlc == 5) {
                    float fval;
                    memcpy(&fval, &rx.data[1], 4);
                    if (subIndex < 3) {
                        u[subIndex] = fval;
                    }
                }
                break;

            case CANID_REF(0):
                if (rx.can_dlc == 5) {
                    float fval;
                    memcpy(&fval, &rx.data[1], 4);
                    if (subIndex < 3) {
                        reference[subIndex] = fval;
                    }
                }
                break;

            case CANID_Y(0):
                if (rx.can_dlc == 5) {

                    float fval;
                    memcpy(&fval, &rx.data[1], 4);
                    if (subIndex < 3) {
                        y_filtered[subIndex] = fval;
                    }
                }
                break;

            case CANID_V(0):
                if (rx.can_dlc == 5) {
                    float fval;
                    memcpy(&fval, &rx.data[1], 4);
                    if (subIndex < 3) {
                        v_ldr[subIndex] = fval;
                    }
                }
                break;

            case CANID_OCC(0):
                if (rx.can_dlc == 2) {
                    bool bval = rx.data[1];
                    if (subIndex < 3) {
                        is_occupied[subIndex] = bval;
                    }
                }
                break;

            case CANID_A(0):
                if (rx.can_dlc == 2) {
                    bool bval = rx.data[1];
                    if (subIndex < 3) {
                        anti_windup[subIndex] = bval;
                    }
                }
                break;

            case CANID_F(0):
                if (rx.can_dlc == 2) {
                    bool bval = rx.data[1];
                    if (subIndex < 3) {
                        is_active[subIndex] = bval;
                    }
                }
                break;

            case CANID_D(0):
                if (rx.can_dlc == 5) {
                    float fval;
                    memcpy(&fval, &rx.data[1], 4);
                    if (subIndex < 3) {
                        d[subIndex] = fval;
                    }
                }
                break;

            case CANID_P(0):
                if (rx.can_dlc == 5) {
                    float fval;
                    memcpy(&fval, &rx.data[1], 4);
                    if (subIndex < 3) {
                        power[subIndex] = fval;
                    }
                }
                break;

            case CANID_T(0):
                if (rx.can_dlc == 5) {
                    float fval;
                    memcpy(&fval, &rx.data[1], 4);
                    if (subIndex < 3) {
                        elapsed[subIndex] = fval;
                    }
                }
                break;

            case CANID_BUFF(0):

                if (rx.can_dlc == 8 && buffer_full < BUFFER_SIZE) {
                    float y, u;
                    memcpy(&y, &rx.data[0], 4);
                    memcpy(&u, &rx.data[4], 4);
                    data_buffer0[write_index] = {.y = y, .u = u};
                    write_index++;
            
                    if (write_index == BUFFER_SIZE) {
                        buffer_full = true;
                    }
                }
                break;

            case CANID_ENERGY(0):
                if (rx.can_dlc == 5) {
                    float fval;
                    memcpy(&fval, &rx.data[1], 4);
                    if (subIndex < 3) {
                        E[subIndex] = fval;
                    }
                }
                break;

            case CANID_VIS(0):
                if (rx.can_dlc == 5) {
                    float fval;
                    memcpy(&fval, &rx.data[1], 4);
                    if (subIndex < 3) {
                        V[subIndex] = fval;
                    }
                }
                break;

            case CANID_FLICK(0):
                if (rx.can_dlc == 2) {
                    float fval;
                    memcpy(&fval, &rx.data[1], 4);
                    if (subIndex < 3) {
                        F[subIndex] = fval;
                    }
                }
                break;

            case CANID_MAN(0):
                if (rx.can_dlc == 2) {
                    bool bval = rx.data[1];
                    if (subIndex < 3) {
                        manual[subIndex] = bval;
                    }
                }
                break;

            case CANID_BOUND_OCC(0):
                if (rx.can_dlc == 5) {
                    float fval;
                    memcpy(&fval, &rx.data[1], 4);
                    if (subIndex < 3) {
                        lower_bound_occ[subIndex] = fval;
                    }
                }
                break;

            case CANID_BOUND_UNOCC(0):
                if (rx.can_dlc == 5) {
                    float fval;
                    memcpy(&fval, &rx.data[1], 4);
                    if (subIndex < 3) {
                        lower_bound_unocc[subIndex] = fval;
                    }
                }
                break;

            case CANID_COST(0):
                if (rx.can_dlc == 5) {
                    float fval;
                    memcpy(&fval, &rx.data[1], 4);
                    if (subIndex < 3) {
                        cost_coeff[subIndex] = fval;
                    }
                }
                break;

            case CANID_RES(0):
                if (rx.can_dlc == 1) {
                    Serial.println("restart received");
                    bool bval = rx.data[0];
                    restart = bval;
                }
                break;

            case CANID_U(1):
                if (rx.can_dlc == 5) {
                    float fval;
                    memcpy(&fval, &rx.data[1], 4);
                    if (subIndex < 3) {
                        u[subIndex] = fval;
                    }
                }
                break;

            case CANID_REF(1):
                if (rx.can_dlc == 5) {
                    float fval;
                    memcpy(&fval, &rx.data[1], 4);
                    if (subIndex < 3) {
                        reference[subIndex] = fval;
                    }
                }
                break;

            case CANID_Y(1):
                if (rx.can_dlc == 5) {
                    float fval;
                    memcpy(&fval, &rx.data[1], 4);
                    if (subIndex < 3) {
                        y_filtered[subIndex] = fval;
                    }
                }
                break;

            case CANID_V(1):
                if (rx.can_dlc == 5) {
                    float fval;
                    memcpy(&fval, &rx.data[1], 4);
                    if (subIndex < 3) {
                        v_ldr[subIndex] = fval;
                    }
                }
                break;

            case CANID_OCC(1):
                if (rx.can_dlc == 2) {
                    bool bval = rx.data[1];
                    if (subIndex < 3) {
                        is_occupied[subIndex] = bval;
                    }
                }
                break;

            case CANID_A(1):
                if (rx.can_dlc == 2) {
                    bool bval = rx.data[1];
                    if (subIndex < 3) {
                        anti_windup[subIndex] = bval;
                    }
                }
                break;

            case CANID_F(1):
                if (rx.can_dlc == 2) {
                    bool bval = rx.data[1];
                    if (subIndex < 3) {
                        is_active[subIndex] = bval;
                    }
                }
                break;


            case CANID_D(1):
                if (rx.can_dlc == 5) {
                    float fval;
                    memcpy(&fval, &rx.data[1], 4);
                    if (subIndex < 3) {
                        d[subIndex] = fval;
                    }
                }
                break;

            case CANID_P(1):
                if (rx.can_dlc == 5) {
                    float fval;
                    memcpy(&fval, &rx.data[1], 4);
                    if (subIndex < 3) {
                        power[subIndex] = fval;
                    }
                }
                break;

            case CANID_T(1):
                if (rx.can_dlc == 5) {
                    float fval;
                    memcpy(&fval, &rx.data[1], 4);
                    if (subIndex < 3) {
                        elapsed[subIndex] = fval;
                    }
                }
                break;

            case CANID_BUFF(1):
                if (rx.can_dlc == 8 && buffer_full_2 < BUFFER_SIZE) {
                    float y, u;
                    memcpy(&y, &rx.data[0], 4);
                    memcpy(&u, &rx.data[4], 4);
                    data_buffer2[write_index_2] = {.y = y, .u = u};
                    write_index_2++;
            
                    if (write_index_2 == BUFFER_SIZE) {
                        buffer_full_2 = true;
                    }
                }
                break;

            case CANID_ENERGY(1):
                if (rx.can_dlc == 5) {
                    float fval;
                    memcpy(&fval, &rx.data[1], 4);
                    if (subIndex < 3) {
                        E[subIndex] = fval;
                    }
                }
                break;


            case CANID_VIS(1):
                if (rx.can_dlc == 5) {
                    float fval;
                    memcpy(&fval, &rx.data[1], 4);
                    if (subIndex < 3) {
                        V[subIndex] = fval;
                    }
                }
                break;

            case CANID_FLICK(1):
                if (rx.can_dlc == 5) {
                    float fval;
                    memcpy(&fval, &rx.data[1], 4);
                    if (subIndex < 3) {
                        F[subIndex] = fval;
                    }
                }
                break;

            case CANID_MAN(1):
            if (rx.can_dlc == 2) {
                bool bval = rx.data[1];
                if (subIndex < 3) {
                    manual[subIndex] = bval;
                }
            }
            break;

            case CANID_BOUND_OCC(1):
                if (rx.can_dlc == 5) {
                    float fval;
                    memcpy(&fval, &rx.data[1], 4);
                    if (subIndex < 3) {
                        lower_bound_occ[subIndex] = fval;
                    }
                }
                break;
            
            case CANID_BOUND_UNOCC(1):
                if (rx.can_dlc == 5) {
                    float fval;
                    memcpy(&fval, &rx.data[1], 4);
                    if (subIndex < 3) {
                        lower_bound_unocc[subIndex] = fval;
                    }
                }

            case CANID_COST(1):
                if (rx.can_dlc == 5) {
                    float fval;
                    memcpy(&fval, &rx.data[1], 4);
                    if (subIndex < 3) {
                        cost_coeff[subIndex] = fval;
                    }
                }
                break;

            case CANID_RES(1):
                if (rx.can_dlc == 1) {
                    bool bval = rx.data[0];
                    restart = bval;
                }
                break;

            default:
                Serial.println("ID not Found!!!!");
                break;
            }
    }
    delayMicroseconds(100);
    mutex_exit(&mut_CAN);
}

void apply_consensus(){

    mutex_enter_blocking(&mut_CAN);
    float best_u[2] ={};
    float L = 0;
    float u_avg[2]={};
    float new_y = 0.0;
 ;

    Consensus consensus(G, d[2], y_filtered[2], cost_coeff[2]);

    for(int j =0; j<200; j++){
      consensus.consensus_iterate();
  
      for (int i = 0; i < 2; ++i) {
          best_u[i] = consensus.u_best[i];
      }
    }

    u[2] = best_u[2];

    for(int i=0; i<2; i++){
        u_avg[i] = best_u[i]/NUM_NODES;
    }

    consensus.update_u_av(u_avg);

    consensus.update_lambda();


    mutex_exit(&mut_CAN);
    delayMicroseconds(100);
}