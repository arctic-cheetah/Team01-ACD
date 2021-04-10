//Testing of obstacle detection sensors
//Team 01 ENGG1000 ACD
//z5312813

#define IR_OD_1 22
#define IR_OD_2 23
//Rear
#define IR_OD_3 24
#define IR_OD_4 25
//Egg detection
#define IR_OD_5 26

void setup() {
    Serial.begin(9600);
    pinMode(IR_OD_1, INPUT);
    pinMode(IR_OD_2, INPUT);
    pinMode(IR_OD_3, INPUT);
    pinMode(IR_OD_4, INPUT);
    pinMode(A0, OUTPUT);
    digitalWrite(A0, HIGH);
    pinMode(A1, OUTPUT);
    digitalWrite(A1, HIGH);
    pinMode(A2, OUTPUT);
    digitalWrite(A2, HIGH);
    Serial.print("Test Initialising!\n");
}

void loop() {
    Serial.println(digitalRead(IR_OD_1));
    Serial.println(digitalRead(IR_OD_2));
    Serial.println(digitalRead(IR_OD_3));
    Serial.println(digitalRead(IR_OD_4));
    Serial.print("-----------\n");
    delay(1000);
}
