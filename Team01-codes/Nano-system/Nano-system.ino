//The code for the arduino nnano


//Pins declared here
#define SOUND_PIN 3
#define MEGA_PIN 4
//RGB LED headlight
#define RED_LED 11
#define GREEN_LED 10
#define BLUE_LED 9
#define MAX_PWM 255
#define DELAY 5
#define DELAY_STAY_ON_ONE_COLOUR 100

//Settings for reversing truck sound
#define INTERVAL 1000 //ms
#define SOUND_DURATION_ON 18000//ms
#define FREQUENCY 30 //Hz

unsigned long duration = 0; //Duration for off time arduino
unsigned long lastTime = 0; //Keep track of time for arduino reversing truck sound
char c = 0;


void command(char instruct);
void reversing_truck_sound();

void setup() {
    Serial.begin(9600);
    pinMode(SOUND_PIN, OUTPUT);
    pinMode(MEGA_PIN, INPUT);
}

void loop() {
    if (Serial.available()) {
        c = Serial.read();
    }
    command(c);
    //Invoke the sound if the Mega requires the reversing truck sound
    reversing_truck_sound();

    //Allow time for serial to read
    delay(10);
}

////////////////////////////////////////////////////////////////////
//Function declarations
void reversing_truck_sound() {
    duration = millis();
    if (digitalRead(MEGA_PIN)) {
        if (duration - lastTime > INTERVAL) {
            lastTime = duration;
            tone(SOUND_PIN, FREQUENCY, SOUND_DURATION_ON);
        }
    }
}

void command(char instruct) {
    //Fade with rainbow colours
    if (instruct == 'r') {
        Serial.print("Rainbow on!\n");
        RGB_fade();
    }
    //White LED lights
    else if (instruct == 'w') {
        Serial.print("White light on!\n");
        white_LED();
    }
}

//Only white LED light
void white_LED(){
    analogWrite(BLUE_LED, MAX_PWM);
    analogWrite(RED_LED, MAX_PWM);
    analogWrite(GREEN_LED, MAX_PWM);
}

//Fades the RGB light
void RGB_fade() {
    //Begin with white

  
  //Fade red and green
    for (int i = 0; i < MAX_PWM; i +=1) {
        analogWrite(BLUE_LED, MAX_PWM - i);
        analogWrite(RED_LED, MAX_PWM);
        analogWrite(GREEN_LED, MAX_PWM);
        reversing_truck_sound();
        delay(DELAY);
    }
    for (int i = 0; i < MAX_PWM; i +=1) {
        analogWrite(BLUE_LED, i);
        analogWrite(RED_LED, MAX_PWM - i);
        analogWrite(GREEN_LED, MAX_PWM);
        reversing_truck_sound();
        delay(DELAY);
    }
    
    //Go to blue
    for (int i = 0; i < MAX_PWM; i +=1) {
        analogWrite(BLUE_LED, MAX_PWM);
        analogWrite(RED_LED, 0);
        analogWrite(GREEN_LED, MAX_PWM - i);
        reversing_truck_sound();
        delay(DELAY);
    }
    delay(DELAY_STAY_ON_ONE_COLOUR);
    //Go to Green
        for (int i = 0; i < MAX_PWM; i +=1) {
        analogWrite(BLUE_LED, MAX_PWM - i);
        analogWrite(RED_LED, 0);
        analogWrite(GREEN_LED, i);
        reversing_truck_sound();
        delay(DELAY);
    }
    delay(DELAY_STAY_ON_ONE_COLOUR);
    //Go to red
        for (int i = 0; i < MAX_PWM; i +=1) {
        analogWrite(BLUE_LED, 0);
        analogWrite(RED_LED, i);
        analogWrite(GREEN_LED, MAX_PWM - i);
        reversing_truck_sound();
        delay(DELAY);
    }
    delay(DELAY_STAY_ON_ONE_COLOUR);
        for (int i = 0; i < MAX_PWM; i +=1) {
        analogWrite(BLUE_LED, i);
        analogWrite(RED_LED, MAX_PWM);
        analogWrite(GREEN_LED, 0);
        reversing_truck_sound();
        delay(DELAY);
    }
    
    //Go to white
    for (int i = 0; i < MAX_PWM; i +=1) {
        analogWrite(BLUE_LED, MAX_PWM);
        analogWrite(RED_LED, MAX_PWM);
        analogWrite(GREEN_LED, i);
        reversing_truck_sound();
        delay(DELAY);
    }
}
