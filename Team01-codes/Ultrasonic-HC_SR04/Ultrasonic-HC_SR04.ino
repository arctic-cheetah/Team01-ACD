//Code for the Ultrasonic HC SR04 sensor
//Jeremy Mang
//25-03-2021

#define ECHO_PIN A9
#define TRIGGER_PIN A8

void setup() {
    Serial.begin(9600);
    //Initialist ECHO to be input and trigger to be output
    pinMode(TRIGGER_PIN, OUTPUT);
    digitalWrite(TRIGGER_PIN, LOW);
    pinMode(ECHO_PIN, INPUT);
}

void loop () {
    unsigned long TimeOut = 10000;
    
    //Send 8 pulses at 40Khz for approx 10ms
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    //Make Echo_pin high and  Trigger pin Low


    //Wait for the echo pin to become high
    unsigned long endTime = micros() + TimeOut; //ENSURE THIS IS a LONG
    while (micros() < endTime && !digitalRead(ECHO_PIN));

    //Begin timing for the echo pin
    unsigned long timeBegin = micros();
    endTime = timeBegin + TimeOut;
    while (micros() < endTime && digitalRead(ECHO_PIN));
    
    //Calculate the distance
    double dist = double (micros() - timeBegin) * 34000.0 * (0.000001) / 2.0;
  
    // Print the results
    Serial.print(dist);
    Serial.print("cm \n");
    
    // Pause and do it again
    delay(200);
}
