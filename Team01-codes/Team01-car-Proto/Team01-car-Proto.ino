//The prototype code to achieve team01's goals in the ACD project
//Jeremy Mang
//25-03-2021

#include <Arduino.h>
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
double Kp = 0.4;
double Ki = 1.143;
double Kd = 0.0035;
#define MAX_PWM 255
#define MIN_PWM 200
double motor_A_freq_input, motor_A_freq_output;
double motor_B_freq_input, motor_B_freq_output;  
double motor_freq_set = 90;//Set the encoder frequency of the motor

//Object declarations
PID motor_A(&motor_A_freq_input, &motor_A_freq_output, &motor_freq_set, Kp, Ki, Kd, DIRECT);
PID motor_B(&motor_B_freq_input, &motor_B_freq_output, &motor_freq_set, Kp, Ki, Kd, DIRECT);

//Function prototypes
//Initialisers
void init_motor();
void init_encoder();
void init_pid();
//Motor directions
void motor_direction(char c);
void forward();
void reverse();
void left();
void right();
void halt();
//PID control
double encoder_frequency(int motor_encoder);

void setup() {
    Serial.begin(9600);
    init_motor();
    init_encoder();
    init_pid();

}

void loop() {
    if (Serial.available()) {
        motor_direction(Serial.read());
    }
    //Adjust speed via PID loop
    motor_A_freq_input = encoder_frequency(MOTOR_A_ENCODER);
    motor_B_freq_input = encoder_frequency(MOTOR_B_ENCODER);
    motor_A.Compute();
    motor_B.Compute();
    analogWrite(ENA, motor_A_freq_output);
    analogWrite(ENB, motor_B_freq_output);

    delay(TIME_STEP);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Function declarations here

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
//Initialise PID system
void init_pid() {
    //Turn on the PID
    motor_A.SetMode(AUTOMATIC);
    motor_B.SetMode(AUTOMATIC);
    //Set constraints on PID
    motor_A.SetOutputLimits(MIN_PWM, MAX_PWM);
    motor_B.SetOutputLimits(MIN_PWM, MAX_PWM);
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
        Serial.println(frequency);
        return frequency;
    }
    //Serial.println(0.0);
    return 0.0;

}
