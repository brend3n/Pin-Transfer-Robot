#include <AccelStepper.h>
#include <MultiStepper.h>


// ! NOTICE
// Potential class implementation for a linear actuator
// There is no urge to incorporate OOP but could be cleaner in grouping stuff together
// Might work on this later, while waiting for parts for robot but done with calibration and such
// Changing to this might result in full refactor of code though.


/*
class LinearActuator{

  public: 
    int curr_dir;
    int curr_pos;
    int last_pos;
    int last_dir;
    AccelStepper motor;

    int max_speed;
    int acceleration;

    int lower_bound;
    int upper_bound;
    
    LinearActuator(){
    }
}
*/


/* Pin Definitions*/

// Motor driver type
#define interface 1

// Direction and Step pins for all motors
#define dirPinx1  29
#define stepPinx1 27

#define dirPinx2  43
#define stepPinx2 45

#define dirPinx3  51
#define stepPinx3 49

#define dirPiny   42
#define stepPiny  44

#define dirPinz   36
#define stepPinz  37

// Number of steps in a single step of the motor
#define STEP 100

// Speed of all the motors
#define MAX_SPEED        500
#define MAX_ACCELERATION 500

#define MAX_RUN_DISTANCE 100


#define x1_limit_switch -1
#define x2_limit_switch -1
#define x3_limit_switch -1
#define y_limit_switch  -1
#define z_limit_switch  -1


// Instantiating motor driver objects
AccelStepper motor_x1 = AccelStepper(interface, stepPinx1, dirPinx1);
AccelStepper motor_x2 = AccelStepper(interface, stepPinx2, dirPinx2);
AccelStepper motor_x3 = AccelStepper(interface, stepPinx3, dirPinx3);
AccelStepper motor_y  = AccelStepper(interface, stepPiny, dirPiny);
AccelStepper motor_z  = AccelStepper(interface, stepPinz, dirPinz);

// Instantiating class that encapsulates a list of motor driver objects
MultiStepper stepperBabies;

// Position Variables
int curr_posx1;
short curr_dirx1;
int last_posx1;
short last_dirx1;

int curr_posx2;
short curr_dirx2;
int last_posx2;
short last_dirx2;

int curr_posx3;
short curr_dirx3;
int last_posx3;
short last_dirx3;

int curr_posy;
short curr_diry;
int last_posy;
short last_diry;

int curr_posz;
short curr_dirz;
int last_posz;
short last_dirz;


// Bounds of each linear actuator
long x1_left;
long x1_right;

long x2_left;
long x2_right;

long x3_left;
long x3_right;

long y_left;
long y_right;

long z_lower;
long z_upper;


// ! Replace
// this function can be replaced with AccelStepper function to do the same thing -> Need to ask Yousef which function it was
// Moves the motor to the specified position=pos
void translate (AccelStepper *motor, int pos){
  motor->moveTo(pos);
  motor->runToPosition();
  return;
}


// Takes a single step in a given direction
/*

 dir:      
     1, move right
    -1, move left

*/
boolean take_step_until_bound(AccelStepper *motor, short dir, long *bound){

   /* Reset the current position to 0
      Parameters should be the position in steps of where the motor is currently... not sure if should be motor->currentPosition()
   */
  // motor->setCurrentPosition(motor->currentPosition());
  motor->setCurrentPosition(0);
  if (dir > 0){
    // Move clockwise
    motor->move(1);
  }else{
    // Move counter clockwise
    motor->move(-1);
  }
  // Run movement
  motor->runSpeedToPosition();

  if ((
      digitalRead(x1_limit_switch) ||
      digitalRead(x2_limit_switch) ||
      digitalRead(x3_limit_switch) ||
      digitalRead(y_limit_switch)  ||
      digitalRead(z_limit_switch)
      ) == HIGH){

    *bound = motor->currentPosition();
    return false;
  }
  return true;
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
int curr_direction(int last_pos, int new_pos){
  if (new_pos > last_pos){
    return 1;
  }else if(new_pos < last_pos){
    return -1;
  }else{
    return 0;
  }
}


// Configure each motor
void configure_motors(){

  // Might want to set different speeds for different linear actuators

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

// Add all motors to MultiStepper object
void add_all_steppers_to_manager(){
  stepperBabies.addStepper(motor_x1);
  stepperBabies.addStepper(motor_x2);
  stepperBabies.addStepper(motor_x3);
  stepperBabies.addStepper(motor_y);
  stepperBabies.addStepper(motor_z);
}

// Updates all of the current directions that each motor was previously moving in
void update_last_directions(){
  last_dirx1 = curr_direction(last_posx1, motor_x1.currentPosition());
  last_dirx2 = curr_direction(last_posx2, motor_x2.currentPosition());
  last_dirx3 = curr_direction(last_posx3, motor_x3.currentPosition());
  last_diry  = curr_direction(last_posy,  motor_y.currentPosition());
  last_dirz  = curr_direction(last_posz,  motor_z.currentPosition());
}

// Updates all of the last positions that each motor was previously at
void update_last_positions(){
  last_posx1 = motor_x1.currentPosition();
  last_posx2 = motor_x2.currentPosition();
  last_posx3 = motor_x3.currentPosition();
  last_posy = motor_y.currentPosition();
  last_posz = motor_z.currentPosition();
}

// Updates all motor states
void update_motor_states(){
  update_last_directions();
  update_last_positions();
}

void test_run_individual(int curr_posx1, int curr_posx2, int curr_posx3,  int curr_posy, int curr_posz){
// Move counter-clockwise

  // X linear actuators
  curr_posx1 = curr_posx1 - STEP;
  translate(&motor_x1, curr_posx1);

  curr_posx2 = curr_posx2 - STEP;
  translate(&motor_x2, curr_posx2);

  curr_posx3 = curr_posx3 - STEP;
  translate(&motor_x3, curr_posx3);

  // Y
  curr_posy = curr_posy - STEP;
  translate(&motor_y, curr_posy);

  // Z
  curr_posz = curr_posz - STEP;
  translate(&motor_z, curr_posz);

  
  // Move clockwise
  // X
  curr_posx1 = curr_posx1 + STEP;
  translate(&motor_x1, curr_posx1);

  curr_posx2 = curr_posx2 + STEP;
  translate(&motor_x2, curr_posx2);

  curr_posx3 = curr_posx3 + STEP;
  translate(&motor_x3, curr_posx3);

  // Y
  curr_posy = curr_posy + STEP;
  translate(&motor_y, curr_posy);

  // Z
  curr_posz = curr_posz + STEP;
  translate(&motor_z, curr_posz);
}

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


// Get all the bounds of each linear actuator
void calibrate_motors(){


  // boolean take_step_until_bound(AccelStepper *motor, short dir, long *bound){

  // Get right bound
  while (take_step_until_bound(&motor_x1, 1, &x1_left)){;}
  // Get left bound
  while (take_step_until_bound(&motor_x1, -1, &x1_right)){;}


  while (take_step_until_bound(&motor_x2, 1, &x2_left )){;}
  while (take_step_until_bound(&motor_x2, -1, &x2_right)){;}

  while (take_step_until_bound(&motor_x3, 1, &x3_left)){;}
  while (take_step_until_bound(&motor_x3, -1, &x3_right)){;}

  while (take_step_until_bound(&motor_y, 1, &y_left)){;}
  while (take_step_until_bound(&motor_y, -1, &y_right)){;}

  while (take_step_until_bound(&motor_z, 1, &z_lower)){;}
  while (take_step_until_bound(&motor_z, -1, &z_upper)){;}
}

void do_cycle(int num_wash_steps, int pin_depth, int drying_time){
  // Beginning Stage
  // etc ...
  // Final Stage
}

void run_all_cycles(short num_cycles, short num_wash_steps, int pin_depth, int drying_time){
  for(int i = 0; i < num_cycles; i++){
    do_cycle(num_wash_steps, pin_depth, drying_time);
  }
}


void setup() {

  // TESTING: Begin serial connection for debugging
  Serial.begin(9600);

  // Set the max speed and acceleration values for each motor
  configure_motors();
  
  // Add stepper motor objects to MultiStepper object
  add_all_steppers_to_manager();

  // Determine the bounds of each actuator
  // calibrate_motors();
}

void loop() {

  // Run test code
  // test_run_individual(curr_posx,curr_posy,curr_posz);
  test_run_group();

  // run_all_cycles();

  // Update the last position and last direction
  update_motor_states();
}