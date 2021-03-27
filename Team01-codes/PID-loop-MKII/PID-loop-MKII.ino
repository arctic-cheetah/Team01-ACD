//Arduino code for a PID loop by measuring the motor RPM, developed from the motor car code
//Uses the PID library and only for Motor A
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
#define MOTOR_SPEED 255

#define MOTOR_A_ENCODER 4
#define MOTOR_B_ENCODER 3
#define ENCODER_TICKS 20

//PID loop constants
#define TIME_STEP 100 //Tells the Uno to sample and adjust the motor speed in milliseconds
double prev_error, integral = 0;
double Kp = .5;
double Ki = .3;
double Kd = .4;
#define MAX_PWM 255
#define MIN_PWM 200
double motor_freq_input, motor_freq_output; 
double motor_freq_set = 90;//Set the frequency of the motor

PID motor_A(&motor_freq_input, &motor_freq_output, &motor_freq_set, Kp, Ki, Kd, DIRECT);
PID motor_B(&motor_freq_input, &motor_freq_output, &motor_freq_set, Kp, Ki, Kd, DIRECT);

//Function prototypes
void init_motor();
void init_encoder();
void motor_direction(char c);
double motor_speed(int motor_encoder);
double motor_frequency(int motor_encoder);
double duty_cycle(double T_period, unsigned long T_on);

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
    delay(500);
}

void loop() {
    if (Serial.available()) {
        motor_direction(Serial.read());
    }
    motor_freq_input = motor_frequency(MOTOR_A_ENCODER);
    motor_A.Compute();
    //Serial.println(motor_freq_output);
    analogWrite(ENA, motor_freq_output);
    
    
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

double motor_frequency(int motor_encoder) {

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
    double T_period = double(T_on + T_off);

    //Do not print inf
    if ( !isinf(T_period) || !isnan(T_period)) {
        double frequency = 1000000 / T_period; 
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
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
        }
        //Rotate right
        else if (c == 'r') {
            Serial.print("Rotating right\n");
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
        }
        //Rotate left
        else if (c == 'l') {
            Serial.print("Rotating left\n");
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
        }
        //Reverse
        else if (c == 'b') {
            Serial.print("Reversing\n");
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
        }
        //Stop
        else if (c == 's') {
            Serial.print("Stop\n");
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, LOW);
        }
}
//Initialise motor pins
void init_motor() {
    //Set the all motor pins to be output
    for (int i = 0; i < NUM_OF_MOTOR_PIN; i +=1) {
        pinMode(motor_pins[i], OUTPUT);
    }
    //Set the speed of the motors
    analogWrite(ENA, MOTOR_SPEED);
    analogWrite(ENB, MOTOR_SPEED);

}
//Initialise encoder pins
void init_encoder() {
    //Set pin 2 and 3 as input
    pinMode(MOTOR_A_ENCODER, INPUT);
    pinMode(MOTOR_B_ENCODER, INPUT);
}
