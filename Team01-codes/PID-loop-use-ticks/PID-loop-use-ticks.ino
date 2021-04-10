//Arduino code for a pseudo P loop by measuring the motor RPM, developed from the motor car code
//Compares the number of ticks to match the speed
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
#define MOTOR_OFFSET 5  

#define MAX_PWM 255
#define MIN_PWM 0
double motor_A_freq_input, motor_A_freq_output;
double motor_B_freq_input, motor_B_freq_output;
double same_input, same_output;  
double motor_freq_set = 80;//Set the encoder frequency of the motor

//Tracks the ticks
volatile unsigned long enc_l = 0;
volatile unsigned long enc_r = 0;
#define TIME_STEP 10

//Exponential smoothing constants
double prev_term = 0;
#define ALPHA 0.1

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
void count_left();
void count_right();

void setup() {
    Serial.begin(9600);
    //Initialise systems
    init_motor();
    init_encoder();

    pinMode(A2, OUTPUT);
    digitalWrite(A2, HIGH);
    pinMode(A1, OUTPUT);
    digitalWrite(A1, HIGH);
    pinMode(A0, OUTPUT);
    digitalWrite(A0, HIGH);
    
    //Setup interrupts
    attachInterrupt(digitalPinToInterrupt(MOTOR_A_ENCODER), count_left, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MOTOR_B_ENCODER), count_right, CHANGE);
    Serial.println("Motor PID loop test via encoder frequency:\n");
    delay(1000);
}

void loop() {

    unsigned long num_ticks_l;
    unsigned long num_ticks_r;

    // Set initial motor power
    int power_l = MOTOR_SPEED;
    int power_r = MOTOR_SPEED;
    
    //Determine the turn to adjust trajectory
    unsigned long diff_l;
    unsigned long diff_r;
    
    // Reset encoder counts
    enc_l = 0;
    enc_r = 0;
    
    // Remember previous encoder counts
    unsigned long enc_l_prev = enc_l;
    unsigned long enc_r_prev = enc_r;

    unsigned long target_count = 1000;

    while ( (enc_l < target_count) && (enc_r < target_count) ) {
        // Sample number of encoder ticks
        num_ticks_l = enc_l;
        num_ticks_r = enc_r;

        //Drive
        drive(power_l, power_r);
        
        // Print out current number of ticks
        Serial.print(num_ticks_l);
        Serial.print("\t");
        Serial.println(num_ticks_r);

        // Number of ticks counted since last time
        //Proportional
        diff_l = num_ticks_l - enc_l_prev;
        diff_r = num_ticks_r - enc_r_prev;
    
        // Store current tick counter for next time
        enc_l_prev = num_ticks_l;
        enc_r_prev = num_ticks_r;

        // If left is faster, slow it down and speed up right
        //output
        if ( diff_l > diff_r ) {
            power_l -= MOTOR_OFFSET;
            power_r += MOTOR_OFFSET;
        }
        
        // If right is faster, slow it down and speed up left
        if ( diff_l < diff_r ) {
            power_l += MOTOR_OFFSET;
            power_r -= MOTOR_OFFSET;
        }
        
        delay(TIME_STEP);
    }

    //End program
    while(true){
        halt();
    }
    
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


//Return the number of ticks of the left or right motor
void count_left() {
    enc_l +=1;
}
void count_right() {
    enc_r +=1;
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

//Drive and adjust the car's speed
void drive(int power_a, int power_b) {
    // Constrain power to between -255 and 255
    power_a = constrain(power_a, -255, 255);
    power_b = constrain(power_b, -255, 255);

    reverse();

    //adjust the speed of the motors
    analogWrite(ENA, power_a);
    analogWrite(ENB, power_b);
}


//Initialise motor pins
void init_motor() {
    //Set the all motor pins to be output
    for (int i = 0; i < NUM_OF_MOTOR_PIN; i +=1) {
        pinMode(motor_pins[i], OUTPUT);
    }

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
