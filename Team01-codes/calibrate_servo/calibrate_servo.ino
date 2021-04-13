//Recalibrate the servo to x degrees
//Team01 ACD ENGG1000
//By z5312813

#include <Servo.h>

//Initialise servo
Servo unloading_gate;

#define UPPER 180
#define LOWER 0
#define SERVO_PIN 6

//The variable to store the position of the servo
long pos_servo = 0;
bool has_data = false;
long i = 0;

void setup() {
    Serial.begin(9600);
    Serial.print("Initialising servo\n");
    unloading_gate.attach(SERVO_PIN);
}

void loop() {
    while (Serial.available()) {
        //Convert to int by removing the ASCII offset
        i = Serial.read() - '0';
        pos_servo = 10 * pos_servo;
        pos_servo = pos_servo + i;
        has_data = true;
        
    }
    //Don't move the servo if it's zero
    if (has_data) {
        Serial.println(pos_servo);
        //Move the servo to the desired angle
        //constrain(pos_servo, LOWER, UPPER);
        unloading_gate.write(pos_servo);
        pos_servo = 0;
        has_data = false;
    }

    //Give serial time to read data
    delay(10);
}
