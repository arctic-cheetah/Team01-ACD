//Arduino code for a PID loop to control the vehicle's trajectory via IR line followers
//Compares the number of ticks to match the speed
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
#define MOTOR_SPEED 220

#define MOTOR_A_ENCODER 2
#define MOTOR_B_ENCODER 3
#define ENCODER_TICKS 20

//PID loop variables
#define TIME_STEP 20 //Tells the Uno to sample and adjust the motor speed in milliseconds
#define MOTOR_OFFSET 5  
int IR_line_follower[3];
int Kp_prev = 0;
int integral = 0;
int spd = 200;
int c = 0;
float distance;



#define MAX_PWM 255
#define MIN_PWM 0

//IR-line following (IRLF) pins A0 - A5
//Front IR line followers
//#define IR_LF_A A10 //OUT1
#define IR_LF_A A11 //OUT2
#define IR_LF_B A12 //OUT3
#define IR_LF_C A13 //OUT4
#define IR_LF_D A14 //OUT5
#define SPEED_REDUCE 200
//Tracks the ticks
#define TIME_STEP 20


//Function prototypes
//Initialisers
void init_motor();
void init_IR_follower();
//Motor directions
void motor_direction(char c);
void forward();
void reverse();
void left();
void right();
void halt();
//PID control
int read_line();
int mod(int myNum);
void set_motors(int l, int r);

void setup() {
    Serial.begin(9600);
    //Initialise systems
    init_motor();
    init_IR_follower();

    pinMode(A2, OUTPUT);
    digitalWrite(A2, HIGH);
    pinMode(A1, OUTPUT);
    digitalWrite(A1, HIGH);
    pinMode(A0, OUTPUT);
    digitalWrite(A0, HIGH);
    
    //Setup interrupts
    Serial.println("Motor PID loop test via encoder frequency:\n");
    delay(1000);
}

void loop() {


        
    PID();
    
}
//////////////////////////////////////////////////////////////////////////////////////
//Functions here

//Initialise motor pins
void init_motor() {
    //Set the all motor pins to be output
    for (int i = 0; i < NUM_OF_MOTOR_PIN; i +=1) {
        pinMode(motor_pins[i], OUTPUT);
    }
}

//Initialise IR line followers for trajectory control
void init_IR_follower() {
    //Front IR_LF's
    pinMode(IR_LF_A, INPUT);
    pinMode(IR_LF_B, INPUT);
    pinMode(IR_LF_C, INPUT);
    //Rear IR_LF's
    pinMode(IR_LF_D, INPUT);
//  pinMode(IR_LF_E, INPUT);
}

//BEWARE SOMETIMES SENSORS ARE INVERTED
//Correct the trajectory of the vehicle if it veers away from the path
//Refer to sensor diagram
void adjust_trajectory(char direction) {
    //During moving forwards
    if (direction == 'f') {
        //Adjust to the left lightly, if it goes to the right
        //Manual adjustment
        if (!digitalRead(IR_LF_A) && digitalRead(IR_LF_B) && digitalRead(IR_LF_C)) {
            
            analogWrite(ENA, MAX_PWM - SPEED_REDUCE);
            //left();
            Serial.println("turn left");
            delay(TIME_STEP);
            //forward();
        }
        //Adjust to the right, if it veers to the left
        else if (digitalRead(IR_LF_A) && digitalRead(IR_LF_B) && !digitalRead(IR_LF_C)) {
            analogWrite(ENB, MAX_PWM - SPEED_REDUCE);
            //right();
            Serial.println("turn right");
            delay(TIME_STEP);
            //forward();
        }

    }
    else if (direction == 'b') {
        //During reverse
        //Adjust to the right, if it goes to the left
        if (digitalRead(IR_LF_C)) {
            analogWrite(ENA, MAX_PWM);
            delay(TIME_STEP);
        }
        //Adjust to the left, if it veers to the right
        if (digitalRead(IR_LF_A) && !digitalRead(IR_LF_D)) {
            analogWrite(ENB, MAX_PWM);
            delay(TIME_STEP);
        }
    }
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
//Modulus function
int mod(int myNum) {
    if (myNum < 0) {
        return -myNum;
    }
    return myNum;
}


//Read line follower
int readline() {
    IR_line_follower[0] = digitalRead(IR_LF_A);
    IR_line_follower[1] = digitalRead(IR_LF_B);
    IR_line_follower[2] = digitalRead(IR_LF_C);

    //Constants
    int v = 2000*IR_line_follower[0] + 1000*IR_line_follower[1] + 0*IR_line_follower[2] 
    / (IR_line_follower[0] + IR_line_follower[1] + IR_line_follower[2]);

    return v;
}

//
void set_motors(int l, int r){

  //Adjust left
  if(l > 0 && r > 0)  {
    analogWrite(ENA, mod(l));
    analogWrite(ENB, mod(r));

    forward();
  }
  //Adjust left
  else if(l < 0 && r > 0) {
    analogWrite(ENA, mod(l));
    analogWrite(ENB, mod(r));

    left();
  }
  //adjust right
  else if(l > 0 && r < 0) { 
    analogWrite(ENA, mod(l));
    analogWrite(ENB, mod(r));

    right();
  }
  //halt
  else if(l == 0 && r == 0) {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);

    halt();
  }
}

void PID() {
  int power_difference = 0;
  float Kp, Ki, Kd;
  unsigned int position;
  int derivative, proportional;
  while(1) {     
    position = readline();
    Serial.println(position);
    proportional = ((int)position - 1000);
    
    derivative = proportional - Kp_prev;
    integral = integral + proportional;

    Kp_prev = proportional;
    // use the tutorial to set initial values of Kp, Ki, and Kd
    Kp = 5.0; 
    Ki = 0.5;
    Kd = 1;

    power_difference = proportional*Kp + integral*Ki + derivative*Kd;
    const int max = spd/2 + 30;
    if(power_difference > max)
     power_difference = max;
    if(power_difference < -max)
     power_difference = (-1*max);

    if(power_difference < 0)  //left
     set_motors(max+power_difference, max);
    else  //right
     set_motors(max, max-power_difference);    

    readline();
    delay(TIME_STEP);
  }
}
