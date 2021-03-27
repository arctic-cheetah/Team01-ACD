//Arduino code to measure the motor RPM, developed from the motor car code
//Jeremy Mang
//25-03-2021

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

//Function prototypes
void motor_direction(char c);
double motor_speed(int motor_encoder);
double duty_cycle(double T_period, unsigned long T_on);

void setup() {
    Serial.begin(9600);
    //Set the all motor pins to be output
    for (int i = 0; i < NUM_OF_MOTOR_PIN; i +=1) {
        pinMode(motor_pins[i], OUTPUT);
    }
    //Set the speed of the motors
    analogWrite(ENA, MOTOR_SPEED);
    analogWrite(ENB, MOTOR_SPEED);

    //Set pin 2 and 3 as input
    pinMode(MOTOR_A_ENCODER, INPUT);
    pinMode(MOTOR_B_ENCODER, INPUT);
}

void loop() {
    if (Serial.available()) {
        motor_direction(Serial.read());
        //delay(100);
    }
    Serial.print("Motor A RPM: ");
    motor_speed(MOTOR_A_ENCODER);
    Serial.print("Motor B RPM: ");
    motor_speed(MOTOR_B_ENCODER);
    
    //delay(00);
}
//////////////////////////////////////////////////////////////////////////////////////
//Functions here

//Find the motor RPM
double motor_speed(int motor_encoder) {
    //Extend the motor car code here
    //Find the motor speed
    unsigned long T_on = pulseIn(motor_encoder, HIGH);
    unsigned long T_off = pulseIn(motor_encoder, LOW);
    
    //Debugging
    //Serial.println(T_on);
    //Serial.println(T_off);

    //Find the time period of the wave = Ton + Toff (One tick)
    double T_period = T_on + T_off;

    //Debugging
    //Serial.print("Period: ");
    //Serial.println(T_period / 1000000);
    Serial.print("Frequency: ");
    Serial.println(1 / (T_period / 1000000));
    
    //Get the time taken for 20 ticks(one full wave) This is seconds taken per revolution
    double time_per_rev = T_period / 1000000 * ENCODER_TICKS; // Convert milliseconds to seconds
    
    //Get RPS then RPM
    //RPS
    double rev_per_sec = 1 / double(time_per_rev);
    //RPM
    double rev_per_min = rev_per_sec * 60;
    Serial.println(rev_per_min);

    //Duty cycle
    //Serial.println(duty_cycle(T_period, T_on));
    return rev_per_min;
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
