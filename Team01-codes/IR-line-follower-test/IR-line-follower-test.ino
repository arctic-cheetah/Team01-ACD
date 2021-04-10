//IR line follower test
//z5312813
//Engg1000 Team01

#define IR_LF_A A10 //OUT1
#define IR_LF_B A11 //OUT2
#define IR_LF_C A12 //OUT3
#define IR_LF_D A13 //OUT4
#define IR_LF_E A14 //OUT5

void setup() {
    Serial.begin(9600);
    pinMode(IR_LF_A, INPUT);
    
    pinMode(A0, OUTPUT);
    digitalWrite(A0, HIGH);
    pinMode(A1, OUTPUT);
    digitalWrite(A1, HIGH);
    pinMode(A2, OUTPUT);
    digitalWrite(A2, HIGH);
}

void loop() {
    Serial.println(digitalRead(IR_LF_A));
    delay(500);
}
