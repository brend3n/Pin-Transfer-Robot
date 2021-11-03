/*Libraries used*/

/*Interfacing with motor drivers*/
#include <AccelStepper.h>
#include <MultiStepper.h>

/*Button libraries for manual control*/
#include <ezButton.h>

/*Time measurement*/
#include <PollingTimer.h>

/*Pin definitions*/
/*Buttons*/
#define leftButton 42
#define rightButton 44  
#define play_button 24
#define record_button 22

/*Motor Driver*/
#define dirPin 29
#define stepPin 27

/*LEDs*/
#define record_LED 36
#define play_LED 34

/*Motor Interface*/
#define interface 1

// MAX_STORAGE * 2 = Number of Bytes used
#define MAX_STORAGE 500

// Initializing motors
// AccelStepper motor_x1 = AccelStepper(interface, stepPin, dirPin);
// AccelStepper motor_x2 = AccelStepper(interface, stepPin, dirPin);
// AccelStepper motor_x3 = AccelStepper(interface, stepPin, dirPin);
// AccelStepper motor_z = AccelStepper(interface, stepPin, dirPin);
// AccelStepper motor_y = AccelStepper(interface, stepPin, dirPin);
AccelStepper motor_x = AccelStepper(interface, stepPin, dirPin);

// Initializing Timer
PollingTimer timer;

// Button Initialization
ezButton left(leftButton);
ezButton right(rightButton);
ezButton play(play_button);
ezButton record(record_button);

short position_memory[MAX_STORAGE];
int pos_index = 0;

float time_memory[MAX_STORAGE];
int time_index = 0;

#define MAX_RECORDING_SIZE 500
short last_recording = -1;

// LED flags
bool record_flag = false;
bool play_flag = false;

// Misc. variables
int speed = -1;
int recording_size = 0;
int pos_at_start_of_recording = 0;
unsigned long start, end =0;
float time_elapsed = -1;

int last_pos = 0;
int curr_pos = -1;
int curr_dir;
int last_dir = 0;

boolean positionChanged(int last_pos, int new_pos){
  return (new_pos == last_pos) ? false:true;
}

/*
  prev_dir < 0, left
  prev_dir > 0, right
  prev_dir = 0, static
*/

// Returns true if the direction of the motor has changed, otherwise false.
boolean directionChanged(int last_pos, int new_pos, int prev_dir){
  return (( ((new_pos < last_pos) && (prev_dir > 0)) || ( (new_pos > last_pos) && (prev_dir < 0) ) )?true:false);
}

/*
Right -> new_pos > last_pos
Left -> new_pos < last_pos
Not Moving -> new_pos = last_pos
*/
int curr_direction(int last_pos, int new_pos){
  if (new_pos > last_pos){
    return 1;
  }else if(new_pos < last_pos){
    return -1;
  }else{
    return 0;
  }
}

// Moves the motor to the specified position=pos
void translate (AccelStepper *motor, int pos){
  motor->moveTo(pos);
  motor->runToPosition();
  return;
}


void print_recording_data(AccelStepper *motor, float time_mem [], short pos_mem [], int size){

  // Only positive values
  if (size < 0){
    size*=-1;
  }
  Serial.print("Size: " + String(size));
  for(int i  = 0;i < size; i++){
    Serial.println("Position: " +String(pos_mem[i]) +"\nTime at position: " + String(time_mem[i])+"\n");
  }
}
void playback_recording(AccelStepper *motor, float time_mem [], short pos_mem [], int size){
  Serial.println("Playback Recording:\n");
  if (size < 0){
    size *= -1;
  }
  // Play recording
  for(int i = 0; i < size; i++){
    delay(time_mem[i]);
    translate(motor, pos_mem[i]);
  }
}

// Moves the motor back to its home position
// Note: Home position must be set before this function is called
void return_home(AccelStepper *motor){
  translate(motor, 0);
}

void setup() {

  // Open the serial port:
  Serial.begin(38400);

  // Configuring the LEDs
  pinMode(record_LED, OUTPUT);
  pinMode(play_LED, OUTPUT);
  
  // Configure software debouncing on the buttons at 50 ms
  left.setDebounceTime(25);
  right.setDebounceTime(25);
  record.setDebounceTime(25);
  play.setDebounceTime(25);

  // Configuring the motor
  int num = 5000;
  motor_x.setMaxSpeed(num);
  motor_x.setAcceleration(num);
  motor_x.setCurrentPosition(0);

  // Timer setup
  timer.start();
}
// Uses the ezButton library
void loop() {

  // Wait for button event
	left.loop();
	right.loop();
	play.loop();
	record.loop();

  // Reads the current state of the button
	if(record.isPressed()){

	  // Start recording positions
		record_flag = true;
		play_flag = false;

    // Resert the size of the current recording
		recording_size = 0;

    // Turning on indicator LEDs
		digitalWrite(record_LED, HIGH);
		digitalWrite(play_LED, LOW);

    // Get the position at the start of the recording
		pos_at_start_of_recording = motor_x.currentPosition();
	}
	if (play.isPressed()){
    // Stop recording and play it back
		play_flag = true;
		record_flag = false;
		digitalWrite(play_LED, HIGH);
		digitalWrite(record_LED, LOW);

	  // Recording available then play it
		if (recording_size != 0){
			Serial.println("Recording found.");
			translate(&motor_x, pos_at_start_of_recording);
			// playback_recording(&motor_x, time_memory, position_memory, recording_size);
			print_recording_data(&motor_x, time_memory, position_memory, recording_size);
		}else{
			Serial.println("No recording available.");
		}
	}

  // Retrieve the current position of the motor
	curr_pos = motor_x.currentPosition();
  curr_dir = curr_direction(last_pos, curr_pos);


  /*THIS IS PROBABLY WRONG*/
	if ( ((!motor_x.isRunning() && (last_pos != curr_pos)) || (curr_pos != last_pos)) ){
	  Serial.println("Pos: " + String(curr_pos));
  }

  // Check to see if motor is to move
	if (left.getState() == 1){
		curr_pos = curr_pos - 25;
		translate(&motor_x, curr_pos);
	}
	else if(right.getState() == 1){
    curr_pos = curr_pos + 25;
    translate(&motor_x, curr_pos);
	}
  // If the motor isnt moving (speed == 0) or the direction has changed
	if (record_flag && (!motor_x.isRunning() || directionChanged(last_pos, curr_pos, last_dir))){

		if (directionChanged(last_pos, curr_pos, last_dir)){
	    // Save the position
			position_memory[pos_index++] = curr_pos;
			time_memory[time_index++] = 0;
		}
		else{
			if (timer.isRunning()){
				time_elapsed = timer.msec();	
				timer.stop();	

        // Record the movement
				position_memory[pos_index++] = curr_pos;
				time_memory[time_index++] = time_elapsed;

				recording_size++;
	
        // Stop Recording
				if (recording_size >= MAX_RECORDING_SIZE){
					record_flag = false;
				}
        timer.clear();
			  timer.restart();

			}
		  else{
        // Same position as before start recording time until position changes
        timer.start();
		  }
	  }

  }

  // Update the last position and last direction
	last_pos = curr_pos;
  last_dir = curr_dir;
}
