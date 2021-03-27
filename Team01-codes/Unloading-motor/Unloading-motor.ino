//////////////////////////////////////////////////////////////////////////////
//A program to test the unloading motor
//Team01-ACD
//22-03-2021

#define SPEED_CONTROL_UNLOADING_MOTOR 3
#define DIRECTION_UNLOADING_MOTOR 2

void setup() {
    //Initialise Serial communication
    Serial.begin(9600);
    //Set pin 3 as speed control
    //and pin 2 as direction
    pinMode(SPEED_CONTROL_UNLOADING_MOTOR, OUTPUT);
    pinMode(DIRECTION_UNLOADING_MOTOR, OUTPUT);
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();

        //Go forward
        if (c == 'f') {
            Serial.print("Moving Forward\n");
            analogWrite(SPEED_CONTROL_UNLOADING_MOTOR, 255);
            digitalWrite(DIRECTION_UNLOADING_MOTOR, HIGH);
        }
        //Reverse
        else if (c == 'r') {
            Serial.print("Reversing\n");
            analogWrite(SPEED_CONTROL_UNLOADING_MOTOR, 255);
            digitalWrite(DIRECTION_UNLOADING_MOTOR, LOW);
        }
        //Stop
        else if (c == 's') {
            Serial.print("Stopping\n");
            analogWrite(SPEED_CONTROL_UNLOADING_MOTOR, 0);
        }
    }
}
