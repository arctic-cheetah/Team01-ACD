//ENG1000 team01 ACD Code to run the motors for the car's movement
//23-03-2021
//Jeremy Mang

int motor_pins[] = {10, 9, 8, 7, 6, 5};
#define NUM_OF_MOTOR_PIN 5

#define ENA 10
#define IN1 9
#define IN2 8
#define ENB 5
#define IN3 7
#define IN4 6

void setup() {
    Serial.begin(9600);
    //Set the all motor pins to be output
    for (int i = 0; i < NUM_OF_MOTOR_PIN; i +=1) {
        pinMode(motor_pins[i], OUTPUT);
    }
    //Set the speed of the motors
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();

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
        delay(1000);
    }
}
