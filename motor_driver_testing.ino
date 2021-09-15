#include <AccelStepper.h>
#include <MultiStepper.h>
#include <ezButton.h>


// Testing buttons
#define leftButton 42
#define rightButton 44  

// Pins: Stepper Motor Driver
#define dirPin 29
#define stepPin 27

// Other Pins
#define potentiometer A5

#define record_LED 36
#define play_LED 34 
#define record_button 22
#define play_button 24

// Motor Interface
#define interface 1

#define MAX_STORAGE 1000

// Initializing motors
// AccelStepper motor_x1 = AccelStepper(interface, stepPin, dirPin);
// AccelStepper motor_x2 = AccelStepper(interface, stepPin, dirPin);
// AccelStepper motor_x3 = AccelStepper(interface, stepPin, dirPin);
// AccelStepper motor_z = AccelStepper(interface, stepPin, dirPin);

AccelStepper motor_x= AccelStepper(interface, stepPin, dirPin);
// 500 was chosen arbitrarily
short memory[MAX_STORAGE];



// Button Initialization
ezButton left(leftButton);
ezButton right(rightButton);
ezButton play(play_button);
ezButton record(record_button);

// LED flags
bool record_flag = false;
bool play_flag = false;

// Potentiometer Initialization
int pot_val = 0;
int speed = -1;
int i = 0;
int recording_size = 0;
int intial_home = 0;

// Moves the motor to the specified position=pos
void translate (AccelStepper *motor, int pos){
  motor->moveTo(pos);
  motor->runToPosition();
  return;
}

void playback_recording(AccelStepper *motor, short recording [], int size){
  for(int i = 0; i < size; i++){
    Serial.println(recording[i]);
  }
}
// Moves the motor back to its home position
// Note: Home position must be set before this function is called
void return_home(AccelStepper *motor){
  translate(motor, home_position);
}

void setup() {

  // Open the serial port:
  Serial.begin(9600);

  pinMode(record_LED, OUTPUT);
  pinMode(play_LED, OUTPUT);
  
  // Configure software debouncing on the buttons at 50 ms
  left.setDebounceTime(50);
  right.setDebounceTime(50);
  record.setDebounceTime(50);
  play.setDebounceTime(50);

  // Configuring the motor
  int num = 100000;
  motor_x.setMaxSpeed(num);
  motor_x.setAcceleration(num);
  motor_x.setCurrentPosition(0);
}
// Uses the ezButton library
void loop() {

  // Polling each button
  left.loop();
  right.loop();
  play.loop();
  record.loop();
  
  // Reads the current state of the button
  if (record.getState() == 1){
    Serial.println("Record pressed");
    // Start recording positions
    record_flag = true;
    play_flag = false;
  }
  if (play.getState() == 1){
    // Stop recording and play it back
    Serial.println("Play pressed");
    play_flag = true;
    record_flag = false;

    // playback_recording(&motor_x, memory, recording_size);
  }

  // Handle LEDs
  if (record_flag){
    digitalWrite(record_LED, HIGH);
    digitalWrite(play_LED, LOW);
  }else{
    digitalWrite(record_LED, LOW);
  }

  if (play_flag){
    digitalWrite(play_LED, HIGH);
    digitalWrite(record_LED, LOW);
  }else{
    digitalWrite(play_LED, LOW);
  }

  int curr_pos = motor_x.currentPosition();


  if (left.getState() == 1){
      Serial.println("Left");
      curr_pos = curr_pos - 25;
      translate(&motor_x, curr_pos);
  }
  else if(right.getState() == 1){
      Serial.println("Right");
      curr_pos = curr_pos + 25;
      translate(&motor_x, curr_pos);
  }

  // Record the movement
  if (record_flag){
    memory[i++] = (short)curr_pos;
    recording_size++;
    Serial.println("Position: " + (short)curr_pos);
    delay(500);
  }

}

/*
void setup() {

  // open the serial port:
  Serial.begin(9600);
  pinMode(leftButton, INPUT);
  pinMode(rightButton, INPUT);

}

void loop() {

  // check for incoming serial data:
  Serial.println("Left: " + String(digitalRead(leftButton)));
  Serial.println("Right: " + String(digitalRead(rightButton)));
  delay(100);
  // if (digitalRead(leftButton)){
  //   Serial.println("Left")
  // }else if(digitalRead(rightButton)){
  //   Serial.println("Right")
  // }
}
*/
