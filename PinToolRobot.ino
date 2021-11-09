// Motors
#include <AccelStepper.h>
#include <MultiStepper.h>

// Servos
#include <Servo.h>


// ! NOTICE
// Potential class implementation for a linear actuator
// There is no urge to incorporate OOP but could be cleaner in grouping stuff together
// Might work on this later, while waiting for parts for robot but done with calibration and such
// Changing to this might result in full refactor of code though.
/*
class LinearActuator{

  public: 
    int curr_dir;
    int last_pos;
    int last_dir;

    AccelStepper motor;

    int max_speed;
    int acceleration;

    int lower_bound;
    int upper_bound;
    
    LinearActuator(int m_speed, int m_acceleration){
      max_speed = m_speed;
      acceleration = m_acceleration;

      motor = AccelStepper(interface, step);
    }
}
*/

/*###########################################################################################*/
/* Motor Pin Definitions*/

// Motor driver type
#define interface  1

// Direction and Step pins for all motors
#define dirPinx1  -1
#define stepPinx1 -1

#define dirPinx2  -1
#define stepPinx2 -1

#define dirPinx3  27
#define stepPinx3 26

#define dirPiny   -1
#define stepPiny  -1

#define dirPinz   -1
#define stepPinz  -1

/*CONTSTANTS*/
// Number of steps in a single step of the motor
#define NUM_STEP 100

// Speed of all the motors
#define MAX_SPEED        500
#define MAX_ACCELERATION 1000
#define MAX_RUN_DISTANCE 100

#define DELAY            500 // Half a second

#define plate_height_in_steps 1024

#define steps_per_mm 1024

#define heat_and_fan_delay 5000


// 0 -> Middle
// 1 -> Right/Upper
//-1 -> Left/Lower
#define REFERENCE_POS 0

// Change to false to do polling
#define INTERRUPTS_ENABLED false


// Limit switch pins definitions
#define x1_limit_switch -1
#define x2_limit_switch -1
#define x3_limit_switch  3
#define y_limit_switch  -1
#define z_limit_switch  -1

// Emergency button interrupt
#define big_red_button  -1

// Pins for fan and heater N-Channel MOSFET gate pin
#define fan_pin         -1
#define heater_pin      -1
#define fan_heater_pin -1


/*###########################################################################################*/
/* LCD Pin Definitions*/

//#include <Adafruit_TFTLCD.h>
//#include <TouchScreen.h>

//#define LCD_CS A3
//#define LCD_CD A2
//#define LCD_WR A1
//#define LCD_RD A0
//#define LCD_RESET A4
//#define YP A2  
//#define XM A3 
//#define YM 8   
//#define XP 9 
//#define BLACK 0x0000
//#define WHITE 0xFFFF
//
//Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
//TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);


/*###########################################################################################*/

// Position Variables

// ! Can be optimized further to save space
short curr_dirx1;
int   last_posx1;
short last_dirx1;

short curr_dirx2;
int   last_posx2;
short last_dirx2;

short curr_dirx3;
int   last_posx3;
short last_dirx3;

short curr_diry;
int   last_posy;
short last_diry;

short curr_dirz;
int   last_posz;
short last_dirz;


// Bounds of each linear actuator
long x1_right = -1;
long x1_left  = -1;

long x2_left  = -1;
long x2_right = -1;

long x3_left  = -1;
long x3_right = -1;

long y_left   = -1;
long y_right  = -1;

long z_lower  = -1;
long z_upper  = -1;

/*###########################################################################################*/

// Instantiating motor driver objects
AccelStepper motor_x1 = AccelStepper(interface, stepPinx1, dirPinx1);
AccelStepper motor_x2 = AccelStepper(interface, stepPinx2, dirPinx2);
AccelStepper motor_x3 = AccelStepper(interface, stepPinx3, dirPinx3);
AccelStepper motor_y  = AccelStepper(interface, stepPiny, dirPiny);
AccelStepper motor_z  = AccelStepper(interface, stepPinz, dirPinz);

// Instantiating class that encapsulates a list of motor driver objects
MultiStepper stepperBabies;

/*###########################################################################################*/


// WORKS
// ! Will not be used in the final version of the code
// this function can be replaced with AccelStepper function to do the same thing -> Need to ask Yousef which function it was
// Moves the motor to the specified position=pos
void translate (AccelStepper *motor, int pos){
  motor->moveTo(pos);
  motor->runToPosition();
  return;
}

// ! TEST different motor movements when with linear actuator
// WORKING but STILL TESTING
// Takes a steps in a given direction
/*

 dir:      
     1, move right
    -1, move left

*/
boolean take_step_until_bound(AccelStepper *motor, short dir, long *bound){

  if (dir > 0){
    // Move clockwise
    motor->move(10);
  }else{
    // Move counter clockwise
    motor->move(-10);
  }
  // Run movement
  motor->runSpeedToPosition();

  if ((
      digitalRead(x1_limit_switch) ||
      digitalRead(x2_limit_switch) ||
      digitalRead(x3_limit_switch) ||
      digitalRead(x3_limit_switch) ||
      digitalRead(y_limit_switch)  ||
      digitalRead(z_limit_switch)
      ) == LOW){
    
    Serial.println("Bound: " + String(motor->currentPosition()));
    *bound = motor->currentPosition();
    return false;
  }
  return true;
}

// TEST 
void find_bound(AccelStepper *motor, short dir, long *bound){

  motor->moveTo(dir*MAX_RUN_DISTANCE);
  motor->runSpeedToPosition();

  // Checks if the motor stopped
  // In the ISR, interrupt stops the motor.
  // This can be used to see if the motor hit the limit switch
  if (!motor->isRunning()){
    *bound = motor->currentPosition();
    return;
  }
}

// Converts a distance in mm to steps
long convert_mm_to_steps(float mm){
  long steps = mm * steps_per_mm;
  return steps;
}

// Computes the direction the motor is moving in
/* 
  Returns:
     1: Motor moving right (clockwise)
    -1: Motor moving left  (counter clockwise)
     0: Motor not moving   (static)


  State                     Condition

  Moving Right returns 1:   new_pos > last_pos
  Moving Left returns -1:   new_pos < last_pos  
  Not Moving returns 0:     new_pos = last_pos
*/
// ? Might not be necessary
// TEST
// Should be working but need to test
int curr_direction(int last_pos, int new_pos){
  if (new_pos > last_pos){

    return 1;
  }else if(new_pos < last_pos){
    return -1;
  }else{
    return 0;
  }
}

// WORKING
void set_pins(){

  // Emergency Stop button
  // attachInterrupt(digitalPinToInterrupt(big_red_button), emergency_shut_off, RISING);
  // attachInterrupt(digitalPinToInterrupt(d_limit_switch), DominicsISR, CHANGE);

  if (INTERRUPTS_ENABLED){
    // Limit Switches
    attachInterrupt(digitalPinToInterrupt(x1_limit_switch), x1_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(x2_limit_switch), x2_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(x3_limit_switch), x3_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(y_limit_switch), y_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(z_limit_switch), z_ISR, RISING);
  }else{
    // Polling inputs
    pinMode(x1_limit_switch, INPUT_PULLUP);
    pinMode(x2_limit_switch, INPUT_PULLUP);
    pinMode(x3_limit_switch, INPUT_PULLUP);
    pinMode(y_limit_switch,  INPUT_PULLUP);
    pinMode(z_limit_switch,  INPUT_PULLUP);
  }
  
}

// WORKING
// Configure each motor
void configure_motors(){

  // ! Might want to set different speeds for different linear actuators

  // Set maxmium speeds
  motor_x1.setMaxSpeed(MAX_SPEED);
  motor_x2.setMaxSpeed(MAX_SPEED);
  motor_x3.setMaxSpeed(MAX_SPEED);
  motor_y.setMaxSpeed(MAX_SPEED);
  motor_z.setMaxSpeed(MAX_SPEED);

  // Set acceleration
  motor_x1.setAcceleration(MAX_ACCELERATION);
  motor_x2.setAcceleration(MAX_ACCELERATION);
  motor_x3.setAcceleration(MAX_ACCELERATION);
  motor_y.setAcceleration(MAX_ACCELERATION);
  motor_z.setAcceleration(MAX_ACCELERATION);
}

// WORKING
// Add all motors to MultiStepper object
void add_all_steppers_to_manager(){
  stepperBabies.addStepper(motor_x1);
  stepperBabies.addStepper(motor_x2);
  stepperBabies.addStepper(motor_x3);
  stepperBabies.addStepper(motor_y);
  stepperBabies.addStepper(motor_z);
}

// TEST
// Updates all of the current directions that each motor was previously moving in
void update_last_directions(){
  last_dirx1 = curr_direction(last_posx1, motor_x1.currentPosition());
  last_dirx2 = curr_direction(last_posx2, motor_x2.currentPosition());
  last_dirx3 = curr_direction(last_posx3, motor_x3.currentPosition());
  last_diry  = curr_direction(last_posy,  motor_y.currentPosition());
  last_dirz  = curr_direction(last_posz,  motor_z.currentPosition());
}

// TEST
// Updates all of the last positions that each motor was previously at
void update_last_positions(){
  last_posx1 = motor_x1.currentPosition();
  last_posx2 = motor_x2.currentPosition();
  last_posx3 = motor_x3.currentPosition();
  last_posy  = motor_y.currentPosition();
  last_posz  = motor_z.currentPosition();
}

// TEST
// Updates all motor states
void update_motor_states(){
  update_last_directions();
  update_last_positions();
}

// ! Used for testing only
// WORKS
void test_run_individual(){
// Move counter-clockwise

  long target_posx1;
  long target_posx2;
  long target_posx3;
  long target_posy;
  long target_posz;

  // X
  target_posx1 = motor_x1.currentPosition() - NUM_STEP;
  translate(&motor_x1, target_posx1);

  target_posx2 = motor_x2.currentPosition() - NUM_STEP;
  translate(&motor_x2, target_posx2);

  target_posx3 = motor_x3.currentPosition() - NUM_STEP;
  translate(&motor_x3, target_posx3);

  // Y
  target_posy = motor_y.currentPosition() - NUM_STEP;
  translate(&motor_y, target_posy);

  // Z
  target_posz = motor_z.currentPosition() - NUM_STEP;
  translate(&motor_z, target_posz);

  
  // Move clockwise
  // X
  target_posx1 = motor_x1.currentPosition() + NUM_STEP;
  translate(&motor_x1, target_posx1);

  target_posx2 = motor_x2.currentPosition() + NUM_STEP;
  translate(&motor_x2, target_posx2);

  target_posx3 = motor_x3.currentPosition() + NUM_STEP;
  translate(&motor_x3, target_posx3);

  // Y
  target_posy = motor_y.currentPosition() + NUM_STEP;
  translate(&motor_y, target_posy);

  // Z
  target_posz = motor_z.currentPosition() + NUM_STEP;
  translate(&motor_z, target_posz);
}

// ! Used for testing only
// WORKS
void test_run_group(){
  // Set all positions for the motors to move in

  // positions: {X1, X2, X3, Y, Z}
  //             0   1   2   3  4
  
  long positions[5];

  positions[0] = 500;
  positions[1] = 500;
  positions[2] = 500;
  positions[3] = 500;
  positions[4] = 200;

  // Set target positions
  stepperBabies.moveTo(positions);

  // Run motors to target positions
  stepperBabies.runSpeedToPosition();

  // Small delay
  delay(1010);

 // Same as before but other direction
  positions[0] = -500;
  positions[1] = -500;
  positions[2] = -500;
  positions[3] = -500;
  positions[4] = -200;
  stepperBabies.moveTo(positions);
  stepperBabies.runSpeedToPosition();
  delay(1010);
}

// WORKS
// Oscillates between the two bounds of a linear actuator
void oscillate(AccelStepper *motor, long left, long right){
  while(true){
    motor->moveTo(left);
    motor->runToPosition();
    delay(DELAY);
    motor->moveTo(right);
    motor->runToPosition();
  }
}

// WORKING
// Returns the mid point between two points
long mid_point(long lower_bound, long upper_bound){
  return ((upper_bound + lower_bound)/2);
}

// TEST
// Moves all motors to their respective reference points
void go_to_reference(){
  long positions[5];

  if(REFERENCE_POS == 0){
    // Send all motors to their midpoint on linear actuator
    positions[0] = mid_point(x1_left, x1_right);
    positions[1] = mid_point(x2_left, x2_right);
    positions[2] = mid_point(x3_left, x3_right);
    positions[3] = mid_point(y_left,  y_right);
    positions[4] = mid_point(z_lower, z_upper);

    // Set target positions
    stepperBabies.moveTo(positions);

    // Run motors to target positions
    stepperBabies.runSpeedToPosition();

  }else if(REFERENCE_POS == 1){
    // Send all motors to their midpoint on linear actuator
    positions[0] = x1_right;
    positions[1] = x2_right;
    positions[2] = x3_right;
    positions[3] = y_right;
    positions[4] = z_upper;

    // Set target positions
    stepperBabies.moveTo(positions);

    // Run motors to target positions
    stepperBabies.runSpeedToPosition();

  }else if(REFERENCE_POS == -1){
    // Send all motors to their midpoint on linear actuator
    positions[0] = x1_left;
    positions[1] = x2_left;
    positions[2] = x3_left;
    positions[3] = y_left;
    positions[4] = z_lower;

    // Set target positions
    stepperBabies.moveTo(positions);

    // Run motors to target positions
    stepperBabies.runSpeedToPosition();

  }
}

// WORKING but needs further TESTING
// Get all the bounds of each linear actuator
void calibrate_motors_polling(){

  // Get right bound
  while (take_step_until_bound(&motor_x1, 1, &x1_left)){;}
  delay(1000);
  // Get left bound
  while (take_step_until_bound(&motor_x1, -1, &x1_right)){;}
  

  while (take_step_until_bound(&motor_x2, 1, &x2_left)){;}
  delay(1000);
  while (take_step_until_bound(&motor_x2, -1, &x2_right)){;}

  // Serial.println("Bounds:\nLeft:" + String(x2_left));
  // Serial.println("Right:" + String(x2_right));
  // oscillate(&motor_x2, x2_left, x2_right);

  while (take_step_until_bound(&motor_x3, 1, &x3_left)){;}
  delay(1000);
  while (take_step_until_bound(&motor_x3, -1, &x3_right)){;}

  while (take_step_until_bound(&motor_y, 1, &y_left)){;}
  delay(1000);
  while (take_step_until_bound(&motor_y, -1, &y_right)){;}

  while (take_step_until_bound(&motor_z, 1, &z_lower)){;}
  delay(1000);
  while (take_step_until_bound(&motor_z, -1, &z_upper)){;}
}

// TEST
// Get all the bounds of each linear actuator
void calibrate_motors_interrupts(){
  find_bound(&motor_x1,1,&x1_left);
  find_bound(&motor_x1,1,&x1_right);

  delay(500);

  find_bound(&motor_x2,1,&x2_right);
  find_bound(&motor_x2,1,&x2_right);

  delay(500);

  find_bound(&motor_x3,1,&x3_left);
  find_bound(&motor_x3,1,&x3_right);

  delay(500);

  find_bound(&motor_y,1,&y_left);
  find_bound(&motor_y,1,&y_right);

  delay(500);

  find_bound(&motor_z,1,&z_lower);
  find_bound(&motor_z,1,&z_upper);

}

// TEST
// Runs the calibration sequence depending on whether interrupts are enabled.
void calibrate_motors(){
  if (INTERRUPTS_ENABLED){
    Serial.println("INTERRUPTS in calibrate_motors()");
    calibrate_motors_interrupts();
  }else{
    Serial.println("POLLING in calibrate_motors()");
    calibrate_motors_polling();
  }

  // go_to_reference();
}


/*###########################################################################################*/
/*Servo Functions*/
// TODO
// Yousef
// Open gripper arms enough to grab one plate
void open_gripper(){

}

// TODO
// Yousef
// Close gripper arms enough to secure one plate
void close_gripper(){

}

/*###########################################################################################*/
/*Robot Functions*/
// TODO
// Take plate from a stack
void take_from_stack(AccelStepper *motor, int stack, int height_to_pick_from){

}

// TODO
// Put a plate onto a stack
void push_onto_stack(AccelStepper *motor, int stack, int height_to_put_on){

}

// TODO
void do_wash(){

}

// TEST
// Allows fan to draw from power supply
void fan_on(){

  digitalWrite(fan_pin, HIGH);
}

// TEST
// Allows heater to draw from power supply
void heat_on(){
  digitalWrite(heater_pin, HIGH);
}

// TEST
// Turns off the fan.
void fan_off(){
  digitalWrite(fan_pin, LOW);
}

// TEST
// Turns off the heater.
void heat_off(){
  digitalWrite(heater_pin, LOW);
}

// TEST
// Allows fan and hearter to draw from power supply
void do_fan_and_heat(int drying_time_ms){
  fan_on();
  heat_on();
  delay(drying_time_ms);
  head_off();
  fan_off();
}

// TODO
// Perform a single pin transfer and bring the pin back to its starting position.
void do_pin_transfer(){

}

// TODO
// Wash the pin tool.
void wash_pin_tool(){
  // 1. Move pin tool to wash step linear actuator

}

// TODO
// Uses the fan and heater to dry the pin tool. 
void dry_pin_tool(){

  // 1. Move pin tool over to fan and heat
  do_fan_and_heat(heat_and_fan_delay);
  // 2. Move pin tool back to do pin transfer
}

// TODO
// TEST
void do_cycle(int num_wash_steps, int pin_depth, int drying_time, int height_of_next_plate_in_steps){
  take_from_stack();
  do_pin_transfer();
  wash_pin_tool();
  dry_pin_tool();
  push_onto_stack();
}

// TODO
// TEST
void run_all_cycles(short num_plates, short num_wash_steps, int pin_depth, int drying_time){
   
  double height_of_stack = (num_plates * plate_height_in_mm);
  double height_of_next_plate_to_grab = convert_mm_to_steps(height_of_stack);

  for(int i = 0; i < num_cycles; i++){
    
    do_cycle(num_wash_steps, pin_depth, drying_time, height_of_next_plate_to_grab);

    // Update where the next plate is located at.
    height_of_next_plate_to_grab -= plate_height_in_steps;
  }
}

/*###########################################################################################*/
/*LCD Functions*/


/*###########################################################################################*/
/*LCD functions*/
// TODO
// DOMINIC
// Initialize the LCD.
void configure_LCD(){

}

// TODO
// DOMINIC
/*
  screen_num:
    1 -> greeting
    2 -> input_1
    3 -> input_2
*/
void display_screen(int screen_num){
  if(screen_num == 0){

  }else if(screen_num == 1){

  }else if(screen_num == 2){
    
  }else if(screen_num == 3){
    
  }else{

  }
}

// TODO
// DOMINIC
// Get user input from the LCD
void get_user_input(int screen_num){

  // 1. Display screen for user input
  // 2. Cycle through screens for input and get all the necessary user input
  // 3. Store the user input as global variables

}


/*###########################################################################################*/
/* Interrupt Service Routines (ISRs) */

void x1_ISR(){
  motor_x1.stop();
  if (x1_left == -1){
    x1_left = motor_x1.currentPosition();
  }else{
    x1_right = motor_x1.currentPosition();
  }
}

void x2_ISR(){
 motor_x2.stop();
 if (x2_left == -1){
    x2_left = motor_x2.currentPosition();
  }else{
    x2_right = motor_x2.currentPosition();
  }
}

void x3_ISR(){
  motor_x3.stop();
  if (x3_left == -1){
    x3_left = motor_x3.currentPosition();
  }else{
    x3_right = motor_x3.currentPosition();
  }
}

void y_ISR(){
  motor_y.stop();
  if (y_left == -1){
    y_left = motor_y.currentPosition();
  }else{
    y_right = motor_y.currentPosition();
  }
}

void z_ISR(){
  motor_z.stop();
  if (z_lower == -1){
    z_lower = motor_z.currentPosition();
  }else{
    z_upper = motor_z.currentPosition();
  }
}
/*###########################################################################################*/


void test(){
  take_from_stack();
  do_pin_transfer();
  wash_pin_tool();
  dry_pin_tool();
  push_onto_stack();
}

void setup() {

  // TESTING: Begin serial connection for debugging
  Serial.begin(9600);

  // Set the state of any pins used as inputs
  set_pins();

  // Display startup screen
  // 0 -> startup screen
  display_screen(0);

  // Set the max speed and acceleration values for each motor
  configure_motors();

  // Initialize LCD
  configure_LCD();
  
  // Add stepper motor objects to MultiStepper object
  add_all_steppers_to_manager();

  // Determine the bounds of each actuator
  calibrate_motors();

  // Calls whatever things we are testing in the test() function call
  test();
}

void loop() {
  get_user_input();
  run_all_cycles();
}
