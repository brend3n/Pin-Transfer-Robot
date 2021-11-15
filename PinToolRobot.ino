// Motors
#include <AccelStepper.h>
#include <MultiStepper.h>

// Servos
#include <Servo.h>

/*  
  A Phallic Haiku

  PENIS PENIS PE
  PENUS PENSUS PENYASSS BITCH 
  PENUS SNIP SNIP SNIP
*/

/*###########################################################################################*/
/* Motor Pin Definitions*/

// Motor driver type
#define interface  1

// Direction and Step pins for all motors
/*Two separate motor drivers for each x-axis linear actuator*/
#define dirPinx1      -1
#define stepPinx1     -1

#define dirPinx2      -1
#define stepPinx2     -1

/*
  One motor driver for both x-axis linear actuators

  Need to wire the motors inversely such that they step together
*/
#define dirPinGantry  -1
#define stepPinGantry -1

#define dirPiny       -1
#define stepPiny      -1

#define dirPinz1      -1
#define stepPinz1     -1

/* Other Z-axis linear actuator*/
#define dirPinz2      -1
#define stepPinz2     -1

/*###########################################################################################*/
/*Servo Pins*/

#define SERVO_PIN 9

// Servo constants
#define CLOSE 50
#define OPEN  0

/*###########################################################################################*/
/* LCD Pin Definitions*/



/*###########################################################################################*/
/*Other Pin Definitions*/

// Limit switch pins definitions
#define x1_limit_switch -1
#define x2_limit_switch -1
#define gantry_limit_switch -1
#define y_limit_switch  -1
#define z1_limit_switch  -1
#define z2_limit_switch  -1

// Pins for fan and heater N-Channel MOSFET gate pin
#define fan_pin         4
#define heater_pin      4
#define fan_heater_pin -1 // If using one pin to control both MOSFETS

/*###########################################################################################*/
/*CONTSTANTS*/

// Number of steps in a single step of the motor
#define NUM_STEP   100

// Speed of all the motors
#define MAX_SPEED             500
#define MAX_ACCELERATION      1000
#define MAX_RUN_DISTANCE      100

#define DELAY                 500 // Half a second

#define plate_height_in_steps 1024

#define plate_height_in_mm    1024

#define steps_per_mm          1024

#define heat_and_fan_delay    5000

// Change to false to do polling
#define INTERRUPTS_ENABLED false

/*###########################################################################################*/
/*Position Variables*/

// ! Can be optimized further to save space
short curr_dirx1;
int   last_posx1;
short last_dirx1;

short curr_dirx2;
int   last_posx2;
short last_dirx2;

// Gantry
// Used for the gantry (2 x-axis linear actuators together)
short curr_dir_gantry;
int   last_pos_gantry;
short last_dir_gantry;

short curr_diry;
int   last_posy;
short last_diry;

short curr_dirz1;
int   last_posz1;
short last_dirz1;

short curr_dirz2;
int   last_posz2;
short last_dirz2;

// Bounds of each linear actuator
long x1_right = -1;
long x1_left  = -1;

long x2_left  = -1;
long x2_right = -1;

long gantry_left  = -1;
long gantry_right = -1;

long y_left   = -1;
long y_right  = -1;

long z1_lower  = -1;
long z1_upper  = -1;

long z2_lower  = -1;
long z2_upper  = -1;

/*###########################################################################################*/

// Instantiating motor driver objects
AccelStepper motor_x1     =     AccelStepper(interface, stepPinx1, dirPinx1);
AccelStepper motor_x2     =     AccelStepper(interface, stepPinx2, dirPinx2);
AccelStepper motor_gantry =     AccelStepper(interface, stepPinGantry, dirPinGantry);
AccelStepper motor_y      =     AccelStepper(interface, stepPiny, dirPiny);
AccelStepper motor_z      =     AccelStepper(interface, stepPinz, dirPinz);

Servo servo = Servo();

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
      digitalRead(gantry_limit_switch) ||
      digitalRead(y_limit_switch)  ||
      digitalRead(z1_limit_switch) ||
      digitalRead(z2_limit_switch)
      ) == LOW){
    
    Serial.println("Bound: " + String(motor->currentPosition()));
    *bound = motor->currentPosition();
    return false;
  }
  return true;
}

// ! CHANGE THIS TO IMPLEMENT NEW INTERRUPT THING
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


// ! NEED
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


  if (INTERRUPTS_ENABLED){
    // Limit Switches
    attachInterrupt(digitalPinToInterrupt(x1_limit_switch), x1_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(x2_limit_switch), x2_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(gantry_limit_switch), x2_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(y_limit_switch), y_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(z1_limit_switch), z_ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(z2_limit_switch), z_ISR, RISING);

  }else{
    // Polling inputs
    pinMode(x1_limit_switch, INPUT_PULLUP);
    pinMode(x2_limit_switch, INPUT_PULLUP);
    pinMode(gantry_limit_switch, INPUT_PULLUP);
    pinMode(y_limit_switch,  INPUT_PULLUP);
    pinMode(z1_limit_switch,  INPUT_PULLUP);
    pinMode(z2_limit_switch,  INPUT_PULLUP);
  }

    // Set up servo
    servo.attach(SERVO_PIN);
  
}

// WORKING
// Configure each motor
void configure_motors(){

  // ! Might want to set different speeds for different linear actuators

  // Set maxmium speeds
  motor_x1.setMaxSpeed(MAX_SPEED);
  motor_x2.setMaxSpeed(MAX_SPEED);
  motor_gantry.setMaxSpeed(MAX_SPEED);
  motor_y.setMaxSpeed(MAX_SPEED);
  motor_z1.setMaxSpeed(MAX_SPEED);
  motor_z2.setMaxSpeed(MAX_SPEED);

  // Set acceleration
  motor_x1.setAcceleration(MAX_ACCELERATION);
  motor_x2.setAcceleration(MAX_ACCELERATION);
  motor_gantry.setMaxSpeed(MAX_SPEED);
  motor_y.setAcceleration(MAX_ACCELERATION);
  motor_z1.setAcceleration(MAX_ACCELERATION);
  motor_z2.setAcceleration(MAX_ACCELERATION);
}

// WORKING
// Add all motors to MultiStepper object
void add_all_steppers_to_manager(){
  stepperBabies.addStepper(motor_x1);
  stepperBabies.addStepper(motor_x2);
  stepperBabies.addStepper(motor_gantry);
  stepperBabies.addStepper(motor_y);
  stepperBabies.addStepper(motor_z1);
  stepperBabies.addStepper(motor_z2);
}

// TEST
// Updates all of the current directions that each motor was previously moving in
void update_last_directions(){
  last_dirx1 = curr_direction(last_posx1, motor_x1.currentPosition());
  last_dirx2 = curr_direction(last_posx2, motor_x2.currentPosition());
  last_dir_gantry = curr_direction(last_posx3, motor_gantry.currentPosition());
  last_diry  = curr_direction(last_posy,  motor_y.currentPosition());
  last_dirz1  = curr_direction(last_posz,  motor_z1.currentPosition());
  last_dirz2  = curr_direction(last_posz,  motor_z2.currentPosition());
}

// TEST
// Updates all of the last positions that each motor was previously at
void update_last_positions(){
  last_posx1 = motor_x1.currentPosition();
  last_posx2 = motor_x2.currentPosition();
  last_gantry = motor_gantry.currentPosition();
  last_posy  = motor_y.currentPosition();
  last_posz1  = motor_z1.currentPosition();
  last_posz2  = motor_z2.currentPosition();
}

// TEST
// Updates all motor states
void update_motor_states(){
  update_last_directions();
  update_last_positions();
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


  while(take_step_until_bound(&motor_gantry,1,&gantry_left);
  while(take_step_until_bound(&motor_gantry,-1,&gantry_right);)

  while (take_step_until_bound(&motor_y, 1, &y_left)){;}
  delay(1000);
  while (take_step_until_bound(&motor_y, -1, &y_right)){;}

  while (take_step_until_bound(&motor_z1, 1, &z1_lower)){;}
  delay(1000);
  while (take_step_until_bound(&motor_z1, -1, &z1_upper)){;}

  while (take_step_until_bound(&motor_z2, 1, &z2_lower)){;}
  delay(1000);
  while (take_step_until_bound(&motor_z2, -1, &z2_upper)){;}
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

  find_bound(&motor_gantry,1,&gantry_left);
  find_bound(&motor_gantry,1,&gantry_right);

  delay(500);

  find_bound(&motor_y,1,&y_left);
  find_bound(&motor_y,1,&y_right);

  delay(500);

  find_bound(&motor_z1,1,&z1_lower);
  find_bound(&motor_z1,1,&z1_upper);


  find_bound(&motor_z2,1,&z2_lower);
  find_bound(&motor_z2,1,&z2_upper);

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

void gripper(int a, Servo x)
{
  int currentPos =  x.read();
  if (currentPos > a)
  {
    for (int angle = currentPos; angle >= a; angle--) {
      x.write(angle);
      delay(15);
    }
  }
  else if (currentPos < a)
  {
    for (int angle = currentPos; angle <= a; angle++) {
      x.write(angle);
      delay(15);
    }
  }
}

// TODO
// Open gripper arms enough to grab one plate
void open_gripper(){
  gripper(OPEN,servo);
}

// TODO
// Yousef
// Close gripper arms enough to secure one plate
void close_gripper(){
  gripper(CLOSE, servo);
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

// WORKS
// Allows fan to draw from power supply
void fan_on(){
  digitalWrite(fan_pin, HIGH);
}

// WORKS
// Allows heater to draw from power supply
void heat_on(){
  digitalWrite(heater_pin, HIGH);
}

// WORKS
// Turns off the fan.
void fan_off(){
  digitalWrite(fan_pin, LOW);
}

// WORKS
// Turns off the heater.
void heat_off(){
  digitalWrite(heater_pin, LOW);
}

// WORKS
// Allows fan and hearter to draw from power supply
void do_fan_and_heat(int drying_time_ms){
  fan_on();
  heat_on();
  delay(drying_time_ms);
  heat_off();
  fan_off();
  delay(drying_time_ms);
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
  // take_from_stack();
  // do_pin_transfer();
  // wash_pin_tool();
  // dry_pin_tool();
  // push_onto_stack();
}

// TODO
// TEST
void run_all_cycles(short num_plates, short num_wash_steps, int pin_depth, int drying_time){
   
  double height_of_stack = (num_plates * plate_height_in_mm);
  double height_of_next_plate_to_grab = convert_mm_to_steps(height_of_stack);

  for(int i = 0; i < num_plates; i++){
    
    do_cycle(num_wash_steps, pin_depth, drying_time, height_of_next_plate_to_grab);

    // Update where the next plate is located at.
    height_of_next_plate_to_grab -= plate_height_in_steps;
  }
}

/*###########################################################################################*/
/*LCD Functions*/


/*###########################################################################################*/

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

void gantry_ISR(){
 motor_gantry.stop();
 if (gantry_left == -1){
    gantry_left = motor_gantry.currentPosition();
  }else{
    gantry_right = motor_gantry.currentPosition();
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
  motor_z1.stop();
  if (z1_lower == -1){
    z1_lower = motor_z1.currentPosition();
  }else{
    z1_upper = motor_z1.currentPosition();
  }
}
void z_ISR(){
  motor_z2.stop();
  if (z2_lower == -1){
    z2_lower = motor_z2.currentPosition();
  }else{
    z2_upper = motor_z2.currentPosition();
  }

/*###########################################################################################*/


// ! Not sure if will be used... Will probably just poll
void motor_ISR(){
  if (gantry_limit_L == 0){
    Serial.println("gantry_limit_L");
    gantry_left_bound = motor_gantry.currentPosition();
    gantry_limit_L = 1;
  }else if(gantry_limit_R == 0){
    Serial.println("gantry_limit_R");
    gantry_right_bound = motor_gantry.currentPosition();
    gantry_limit_R = 1;
  }else if(z_lower_s == 0){
    z_lower_s = 1;
    z_lower_bound = motor_z.currentPosition();
    Serial.println("z_lower");
  }else if(z_upper_s == 0){
    z_upper_s = 1;
    z_upper_bound = motor_z.currentPosition();
    Serial.println("z_upper");
  }else if(y_L == 0){
    y_L = 1;
    y_left_bound = motor_y.currentPosition();
    Serial.println("y_L");
  }else if(y_R == 0){
    y_R = 1;
    y_right_bound = motor_y.currentPosition();
    Serial.println("y_R");
  }else{
    gantry_limit_L = 0;
    gantry_limit_R = 0;

    z_lower_s = 0;
    z_upper_s = 0;

    y_L = 0;
    y_R = 0;
  }
}


void run_startup(){
  // TESTING: Begin serial connection for debugging
  Serial.begin(9600);


  // Set the state of any pins used as inputs
  set_pins();

  // Set the max speed and acceleration values for each motor
  configure_motors();

  // Initialize LCD
  configure_LCD();

  // Display startup screen
  // 0 -> startup screen
  display_screen(0);

  // Add stepper motor objects to MultiStepper object
  add_all_steppers_to_manager();

  // Find reference positions
  calibrate_motors();

}

void setup() {
  run_startup();

}

void loop() {
  // get_user_input();
  // run_all_cycles();
}
