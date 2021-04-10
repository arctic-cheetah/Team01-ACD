//Arduino code for a PID loop by measuring the motor RPM, developed from the motor car code
//Uses the PID library and works for both motors.
//Different PID constants for both motors
//Starts from 0 speed and accelerates
//Compares both motor speeds to ensure that motor B is the same as A
//Jeremy Mang
//25-03-2021

#include <PID_v1.h>

//Important constants
int motor_pins[] = {10, 9, 8, 7, 6, 5};
#define NUM_OF_MOTOR_PIN 5
#define ENA 10
#define IN1 9
#define IN2 8
#define ENB 5
#define IN3 7
#define IN4 6
#define MOTOR_SPEED 220

#define MOTOR_A_ENCODER 2
#define MOTOR_B_ENCODER 3
#define ENCODER_TICKS 20

//PID loop constants
#define TIME_STEP 10 //Tells the Uno to sample and adjust the motor speed in milliseconds
double prev_error, integral = 0;
//Motor_A PID constants
double KpA = 100;
double KiA = 4444;
double KdA = 0.5625;
//Motor_B PID constants
double KpB = 400;
double KiB = 17777.8;
double KdB = 0;

#define MAX_PWM 255
#define MIN_PWM 0
#define BIAS 2
double motor_A_freq_input, motor_A_freq_output;
double motor_B_freq_input, motor_B_freq_output;
double same_input, same_output;  
double motor_freq_set = 80;//Set the encoder frequency of the motor
#define TIME_STEP 10

//Exponential smoothing constants
double prev_term = 0;
#define ALPHA 0.1

//Object declarations
PID motor_A(&motor_A_freq_input, &motor_A_freq_output, &motor_freq_set, KpB, KiB, KdB, DIRECT);
PID motor_B(&motor_B_freq_input, &motor_B_freq_output, &motor_freq_set, KpB, KiB, KdB, DIRECT);

//Function prototypes
//Initialisers
void init_motor();
void init_encoder();
//Motor directions
void motor_direction(char c);
void forward();
void reverse();
void left();
void right();
void halt();
//PID control
double motor_speed(int motor_encoder);
double encoder_frequency(int motor_encoder);
double duty_cycle(double T_period, unsigned long T_on);
double exponential_smoothing(double prev_term, double curr_term);

void setup() {
    Serial.begin(9600);
    //Initialise systems
    init_motor();
    init_encoder();
    
    //Turn on the PID
    motor_A.SetMode(AUTOMATIC);
    motor_B.SetMode(AUTOMATIC);
    //Set constraints on PID
    motor_A.SetOutputLimits(MIN_PWM, MAX_PWM);
    motor_B.SetOutputLimits(MIN_PWM, MAX_PWM);
    Serial.println("Motor PID loop test via encoder frequency:\n");
    delay(1000);
}

void loop() {
    /*
    if (Serial.available()) {
        motor_direction(Serial.read());
    }
    */
    forward();
    //Debugging output
    //Serial.print("A,");
    motor_A_freq_input = encoder_frequency(MOTOR_A_ENCODER);
    //Serial.print("B,");
    motor_B_freq_input = encoder_frequency(MOTOR_B_ENCODER);
    //motor_speed(MOTOR_B_ENCODER);
    motor_A.Compute();
    motor_B.Compute();
    
    //Serial.println(motor_A_freq_output);
    analogWrite(ENA, motor_A_freq_output);
    analogWrite(ENB, motor_B_freq_output);
    
    delay(TIME_STEP);
}
//////////////////////////////////////////////////////////////////////////////////////
//Functions here
//Find the motor RPM
double motor_speed(int motor_encoder) {
    //Extend the motor car code here
    //Find the motor speed
    unsigned long T_on = pulseIn(motor_encoder, HIGH);
    unsigned long T_off = pulseIn(motor_encoder, LOW);

    //No signal
    if (!T_on && !T_off) {
        return 0.0;
    }
    
    //Debugging
    //Serial.println(T_on);
    //Serial.println(T_off);

    //Find the time period of the wave = Ton + Toff (One tick)
    double T_period = T_on + T_off;

    //Debugging
    //Serial.print("Period: ");
    //Serial.println(T_period / 1000000);
    //Serial.print("Frequency: ");
    //Serial.println(1 / (T_period / 1000000));
    
    //Get the time taken for 20 ticks(one full wave) This is seconds taken per revolution
    double time_per_rev = T_period / 1000000 * ENCODER_TICKS; // Convert milliseconds to seconds
    
    //Get RPS then RPM
    //RPS
    double rev_per_sec = 1 / double(time_per_rev);
    //Serial.println(rev_per_sec);
    //RPM
    double rev_per_min = rev_per_sec * 60;
    //Do not print inf
    if ( !isinf(rev_per_min) ) {
        //Serial.println(rev_per_min);
        //Duty cycle
        //Serial.println(duty_cycle(T_period, T_on));
        return rev_per_min; 
    }
    //Serial.println(0.0);
    return 0.0;
}

//Returns the frequency of the encoder signal of the motor and is
//related to rpm
double encoder_frequency(int motor_encoder) {

    unsigned long T_on = pulseIn(motor_encoder, HIGH);//in microseconds
    unsigned long T_off = pulseIn(motor_encoder, LOW);

    //No signal
    if (!T_on && !T_off) {
        return 0.0;
    }
    
    //Debugging
    //Serial.println(T_on);
    //Serial.println(T_off);

    //Find the time period of the wave = Ton + Toff (One tick)
    double T_period = double(T_on + T_off);
    //Serial.println(T_period);

    //Do not print inf
    if ( !isinf(T_period) || !isnan(T_period)) {
        double frequency = 1000000 / T_period; 
        
        //Perform exponential smoothing
        //double curr_freq = exponential_smoothing(prev_term, frequency);
        //prev_term = curr_freq;
        Serial.println(frequency);
        return frequency;
    }
    //Serial.println(0.0);
    return 0.0;

}

//Print the duty cycle
double duty_cycle(double T_period, unsigned long T_on) {
    return double(T_on / T_period);
}

//Control the motor direction
void motor_direction(char c) {
        
        //Move Forward
        if (c == 'f') {
            Serial.print("Moving forwards\n");
            forward();
        }
        //Rotate right
        else if (c == 'r') {
            Serial.print("Rotating right\n");
            right();
        }
        //Rotate left
        else if (c == 'l') {
            Serial.print("Rotating left\n");
            left();
        }
        //Reverse
        else if (c == 'b') {
            Serial.print("Reversing\n");
            reverse();
        }
        //Stop
        else if (c == 's') {
            Serial.print("Stop\n");
            halt();
        }
}
//Motor directions here:
void forward() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}
void reverse() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}
void left() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}
void right() {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}
void halt() {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}


//Initialise motor pins
void init_motor() {
    //Set the all motor pins to be output
    for (int i = 0; i < NUM_OF_MOTOR_PIN; i +=1) {
        pinMode(motor_pins[i], OUTPUT);
    }
    //Set the speed of the motors
    //analogWrite(ENA, 220);
    //analogWrite(ENB, 0);

}
//Initialise encoder pins
void init_encoder() {
    //Set pin 2 and 3 as input
    pinMode(MOTOR_A_ENCODER, INPUT);
    pinMode(MOTOR_B_ENCODER, INPUT);
}

//Smooth the data
double exponential_smoothing(double prev_term, double curr_term) {
    return prev_term * ALPHA + (1 - ALPHA) * curr_term;
}
