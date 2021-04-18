//Code for the Ultrasonic HC SR04 sensor
//Shorter code-length
//Jeremy Mang
//25-03-2021

#define ECHO_PIN A7
#define TRIGGER_PIN A6

void setup() {
    Serial.begin(9600);
    //Initialist ECHO to be input and trigger to be output
    pinMode(TRIGGER_PIN, OUTPUT);
    digitalWrite(TRIGGER_PIN, LOW);
    pinMode(ECHO_PIN, INPUT);

    delay(2000);
}

void loop () {
    
    //Send 8 pulses at 40Khz for approx 10ms
    //Make Echo_pin high and  Trigger pin Low
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    
    //Wait for the echo pin to become high
    //Begin timing for the echo pin
    unsigned long duration = pulseIn(ECHO_PIN, HIGH);
    
    //Calculate the distance, using the speed of sound (340m/s) in cm/s
    double dist = double (duration) * 34000 * (0.000001) / 2.0;
  
    // Print the results
    Serial.print(dist);
    Serial.print("cm \n");
    
    // Pause and do it again
    delay(300);
}
