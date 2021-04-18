//A sound code for a reversing truck

//Pins declared here
#define SOUND_PIN 34

//Settings for reversing truck sound
#define INTERVAL 1000 //ms
#define SOUND_DURATION_ON 18000//ms
#define FREQUENCY 30 //Hz

int duration = 0; //Duration for off time arduino
int lastTime = 0; //Keep track of time for arduino reversing truck sound


void setup() {
    pinMode(SOUND_PIN, OUTPUT);
}

void loop() {
    duration = millis();
    if (duration - lastTime > INTERVAL) {
        lastTime = duration;
        tone(SOUND_PIN, FREQUENCY, SOUND_DURATION_ON);
    }
}
