//A modular test program to test IR line following 
//ENGG1000
//z5312813

//IR-line following (IRLF) pins A0 - A5
//Front IR line followers
#define IR_LF_I A0 //OUT1 Prev A10, A11, A12, A13, A14
#define IR_LF_A A1 //OUT2 
#define IR_LF_B A2 //OUT3
#define IR_LF_C A3 //OUT4
#define IR_LF_D A4 //OUT5


void setup() {
    Serial.begin(9600);
    pinMode(IR_LF_A, INPUT);
    pinMode(IR_LF_B, INPUT);
    pinMode(IR_LF_C, INPUT);
    pinMode(IR_LF_D, INPUT);
    pinMode(IR_LF_I, INPUT);
}

void loop() {
    Serial.print(digitalRead(IR_LF_I));
    Serial.print("\t");
    Serial.print(digitalRead(IR_LF_A));
    Serial.print("\t");
    Serial.print(digitalRead(IR_LF_B));
    Serial.print("\t");
    Serial.print(digitalRead(IR_LF_C));
    Serial.print("\t");
    Serial.println(digitalRead(IR_LF_D));
}
