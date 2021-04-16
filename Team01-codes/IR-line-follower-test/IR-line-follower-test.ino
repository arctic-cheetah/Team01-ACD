//A modular test program to test IR line following 
//ENGG1000
//z5312813

//IR-line following (IRLF) pins A0 - A5
//Front IR line followers
#define IR_LF_I A10 //OUT1
#define IR_LF_A A11 //OUT2
#define IR_LF_B A12 //OUT3
#define IR_LF_C A13 //OUT4
#define IR_LF_D A14 //OUT5


void setup() {
    Serial.begin(9600);
    pinMode(IR_LF_A, INPUT);
    pinMode(IR_LF_B, INPUT);
    pinMode(IR_LF_C, INPUT);
    pinMode(IR_LF_I, INPUT);
}

void loop() {
    Serial.print(digitalRead(IR_LF_I));
    Serial.print("\t");
    Serial.print(digitalRead(IR_LF_A));
    Serial.print("\t");
    Serial.print(digitalRead(IR_LF_B));
    Serial.print("\t");
    Serial.println(digitalRead(IR_LF_C));
}
