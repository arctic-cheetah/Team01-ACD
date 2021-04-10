//A program to control the gate of the unloading system
//Team01 ENGG1000
//Jeremy Mang
//Unloading motor pins
//controls speed
#define UNLOADING_MOTOR_EN 12
//controls direction
#define UNLOADING_MOTOR_PHASE 31
#define MAX_PWM 100

//Open the gate
void open_gate() {
    analogWrite(UNLOADING_MOTOR_EN, MAX_PWM);
    digitalWrite(UNLOADING_MOTOR_PHASE, LOW);
    delay(200);
    //Turn off the motor
    analogWrite(UNLOADING_MOTOR_EN, 0);
}
//Close the gate and turn off the motor
void close_gate() {
    analogWrite(UNLOADING_MOTOR_EN, MAX_PWM);
    digitalWrite(UNLOADING_MOTOR_PHASE, HIGH);
    delay(200);
    //Turn off the motor
    analogWrite(UNLOADING_MOTOR_EN, 0);
}
void stop_motor() {
    analogWrite(UNLOADING_MOTOR_EN, 0);
}

void setup() {
    Serial.begin(9600);
    //pinMode(UNLOADING_MOTOR_EN, OUTPUT);
    //pinMode(UNLOADING_MOTOR_PHASE, OUTPUT);
    Serial.println("Closing!");
}

void loop() {
    if(Serial.available()){
        char c = Serial.read();
        if (c == 'c') {
            close_gate();
        }
        else if (c == 'o') {
            open_gate();
        }
    }
    delay(100);
}
