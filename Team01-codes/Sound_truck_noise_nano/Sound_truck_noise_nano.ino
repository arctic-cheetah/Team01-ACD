//A sound code for a reversing truck for the Arduino Nano
//Modified to enable ISR's in the ENGG1000 code

//Pins declared here
#define SOUND_PIN 3
#define MEGA_PIN 4

//Settings for reversing truck sound
#define INTERVAL 1000 //ms
#define SOUND_DURATION_ON 18000//ms
#define FREQUENCY 30 //Hz

unsigned long duration = 0; //Duration for off time arduino
unsigned long lastTime = 0; //Keep track of time for arduino reversing truck sound


void setup() {
    pinMode(SOUND_PIN, OUTPUT);
    pinMode(MEGA_PIN, INPUT);
}

void loop() {
    //Invoke the sound if the Mega requires the reversing truck sound
    duration = millis();
    if (digitalRead(MEGA_PIN)) {
        if (duration - lastTime > INTERVAL) {
            lastTime = duration;
            tone(SOUND_PIN, FREQUENCY, SOUND_DURATION_ON);
        }
    }
    
}
