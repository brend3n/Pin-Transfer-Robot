// Motors
#include <AccelStepper.h>
#include <MultiStepper.h>

// Servos
#include <Servo.h>

/*###########################################################################################*/
/* Motor Pin Definitions*/

// Motor driver type
#define interface  1

// Direction and Step pins for all motors
#define dirPinx   40
#define stepPinx  41

#define dirPiny   38
#define stepPiny  39

#define dirPinz1  46
#define stepPinz1 47

#define dirPinz2  44
#define stepPinz2 45

/*###########################################################################################*/
/*Position Constants*/

#define Z_HIGH -100

#define drying_time_ms 10000
#define time_in_solution_ms 1000

#define time_for_full_absorption_of_chemicals_in_ms 1000
#define time_for_full_transfer_of_chemicals_in_ms 1000

#define STEPS_TO_MM  25.455
// #define Plate_Offset 290
#define Plate_Offset 319

#define BASE_PLATE_STEPS -1881

#define PINS_ABOVE_WELL_PLATE_OPENING_STEPS -1897
#define PINS_AT_BOTTOM_OF_WELL_PLATE_STEPS -2128

#define Cell_Input_Stack_X 2063
#define Cell_Input_Stack_Y 3423
#define Cell_Input_Stack_Z1 Z_HIGH
#define Cell_Input_Stack_Z2 BASE_PLATE_STEPS

#define Cell_Output_Stack_X 873
#define Cell_Output_Stack_Y 3351
#define Cell_Output_Stack_Z1 Z_HIGH
#define Cell_Output_Stack_Z2 BASE_PLATE_STEPS

#define Chemical_Input_Stack_X 2063
#define Chemical_Input_Stack_Y 6091
#define Chemical_Output_Stack_X 899
#define Chemical_Output_Stack_Y 6016

#define Cell_Transfer_Area_Gripper_X 1463
#define Cell_Transfer_Area_Gripper_Y 3358
#define Cell_Transfer_Area_Gripper_Z1 Z_HIGH
#define Cell_Transfer_Area_Gripper_Z2 BASE_PLATE_STEPS

#define Chemical_Transfer_Area_Gripper_X 1467
#define Chemical_Transfer_Area_Gripper_Y 6051
#define Chemical_Transfer_Area_Gripper_Z1 Z_HIGH
#define Chemical_Transfer_Area_Gripper_Z2 BASE_PLATE_STEPS

#define Cell_Transfer_Area_Pin_Tool_X 843.5
#define Cell_Transfer_Area_Pin_Tool_Y 3281

#define Chemical_Transfer_Area_Pin_Tool_X 836
#define Chemical_Transfer_Area_Pin_Tool_Y 5949

#define Solution_1_X 1451
#define Solution_1_Y 486
#define Solution_1_Z1 -1680
#define Solution_1_Z2 Z_HIGH

#define Solution_2_X 855
#define Solution_2_Y 417
#define Solution_2_Z1 -1656
#define Solution_2_Z2 Z_HIGH

#define Solution_3_X 249
#define Solution_3_Y 334
#define Solution_3_Z1 -1657
#define Solution_3_Z2 Z_HIGH

#define Fan_Heater_X 1908
#define Fan_Heater_Y 3318
#define Fan_Heater_Z1 -154
#define Fan_Heater_Z2 Z_HIGH


/*###########################################################################################*/
/*Servo Pins*/
#define SERVO_PIN 10

// Servo constants
#define CLOSE 50
#define OPEN  0

/*###########################################################################################*/
/* LCD Pin Definitions*/



/*###########################################################################################*/
/*Bluetooth*/

/*###########################################################################################*/
/*Other Pin Definitions*/

// Limit switch pins definitions
#define y_switch 26
#define x_switch 29
#define z1_switch 25
#define z2_switch 24

// Pins for fan and heater N-Channel MOSFET gate pin
#define fan_pin         34
#define heater_pin      36

/*###########################################################################################*/
/*CONTSTANTS*/

// Motor speeds
#define SPEED_GANTRY 100
#define SPEED_Y      200
#define SPEED_Z      100

#define SPEED_Z1     100
#define SPEED_Z2     100

// Speed of all the motors
#define MAX_SPEED             150
#define MAX_ACCELERATION      100

#define DELAY                 500 // Half a second

#define plate_height_in_steps 1024

#define plate_height_in_mm    1024

#define steps_per_mm          1024

#define heat_and_fan_delay    5000

// Change to false to do polling
#define INTERRUPTS_ENABLED false

/*###########################################################################################*/

// Instantiating motor driver objects
AccelStepper gantry   = AccelStepper(interface, stepPinx, dirPinx);
AccelStepper motor_y  = AccelStepper(interface, stepPiny, dirPiny);
AccelStepper motor_z1 = AccelStepper(interface, stepPinz1, dirPinz1);
AccelStepper motor_z2 = AccelStepper(interface, stepPinz2, dirPinz2);

Servo servo = Servo();

/*###########################################################################################*/

// Converts a distance in mm to steps
long convert_mm_to_steps(float mm){
  long steps = mm * STEPS_TO_MM;
  return steps;
}

// Prints the current position of each motor to the serial for testing and getting position values for hard coding.
void print_current_position(){
    Serial.println("X Position: " + String(gantry.currentPosition()) +"\nY Position: " + String(motor_y.currentPosition()) + "\nZ1 Position: " + String(motor_z1.currentPosition()) +"\nZ2 Position: " + String(motor_z2.currentPosition()) + "\n"); 
}

// WORKING
void set_pins(){

  // Polling inputs
  pinMode(x_switch, INPUT);
  pinMode(y_switch, INPUT);
  pinMode(z1_switch, INPUT);
  pinMode(z2_switch, INPUT);

  pinMode(heater_pin, INPUT);
  pinMode(fan_pin, INPUT);

  // Set up servo
  servo.attach(SERVO_PIN);  
}

// Configure each motor
void configure_motors(){

    // Set maxmium speeds
    gantry.setMaxSpeed(MAX_SPEED);
    motor_y.setMaxSpeed(MAX_SPEED);
    motor_z1.setMaxSpeed(MAX_SPEED);
    motor_z2.setMaxSpeed(MAX_SPEED);

    // Set acceleration
    gantry.setAcceleration(MAX_ACCELERATION);
    motor_y.setAcceleration(MAX_ACCELERATION);
    motor_z1.setAcceleration(MAX_ACCELERATION);
    motor_z2.setAcceleration(MAX_ACCELERATION);
}

// Calibrates a single motor given by &motor and a limit switch
long calibrate_motor(AccelStepper *motor, int limit_switch, short dir){

    long steps;
    Serial.println("Start Position:");

    // For testing
    print_current_position();
    
    // Set the current speed and run until the limit switch is hit
    motor->setSpeed(100 * dir);
    while (digitalRead(limit_switch) != LOW){
        motor->runSpeed();
    }

    // ERASE THIS LATER OR ELSE I WILL KILL A MANATEE WITH MY BARE TEETH AND GIVE THE MANATEE TAIL TO MY FRIEND ADOMININC AND THEN I WILL SAY WPAJ THERE BUDYDDYN WHY ARENT YPU LOOKING AT ME 
    delay(0);
    // Distance from starting position to limit switch
    // Negative because reference 0 is at limit switch and all of other distances are negative relative to the limit switch
    steps = -1*motor->currentPosition();

    // For testing
    Serial.println("Steps taken to reach limit switch: " + String(steps));
    // Stop the motor and store the current position
    motor->stop();
    motor->setCurrentPosition(0);

    // For testing
    Serial.println("End Position:");
    print_current_position();

    // Move the motor off the limit switch
    
    motor->move(50 * -1 * dir);
    motor->runToPosition();

    return steps;
}

// Shows the motor's (x,y,z1,z2) coordinates from where the motor starts before calibration

void calibrate_motors(){ 
  int val;

  Serial.println("Calibrating Z1");
  long z1_start = calibrate_motor(&motor_z1, z1_switch, 1); 

  Serial.println("Calibrating Z2");
  long z2_start = calibrate_motor(&motor_z2, z2_switch, 1); 

  Serial.println("Calibrating gantry");
  long x_start = calibrate_motor(&gantry , x_switch, -1);

  Serial.println("Calibrating Y");
  long y_start = calibrate_motor(&motor_y, y_switch, -1);
 
  motor_z1.setSpeed(SPEED_Z);
  motor_z2.setSpeed(SPEED_Z);
//  gantry.setSpeed(SPEED_GANTRY);
  motor_y.setSpeed(SPEED_Y);
  

  // gantry.runToNewPosition(x_start);
  // motor_y.runToNewPosition(y_start);
  // motor_z1.runToNewPosition(z1_start);
  // motor_z2.runToNewPosition(z2_start);
}

// Test code for verifying limit switches are working
void test_limit_switches(){
   Serial.println("X: " + String(digitalRead(x_switch)));
   Serial.println("Y: " + String(digitalRead(y_switch)));
   Serial.println("Z1: " + String(digitalRead(z1_switch)));
   Serial.println("Z2: " + String(digitalRead(z2_switch)));
   Serial.println();
   delay(1000);
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

// Open gripper arms enough to grab one plate
void open_gripper(){
  gripper(OPEN,servo);
}

// Close gripper arms enough to secure one plate
void close_gripper(){
  gripper(CLOSE, servo);
}

/*###########################################################################################*/
/*Robot Functions*/

// Moves each motor to a given position starting with the x-axis
void move_to_coordinate_x_first(long x, long y, long z1, long z2){
// void move_to_coordinate_x_first(long *coordinates){

  motor_z1.setSpeed(SPEED_Z);
  motor_z2.setSpeed(SPEED_Z);
  gantry.setSpeed(SPEED_GANTRY);
  motor_y.setSpeed(SPEED_Y);

  gantry.runToNewPosition(x);
  motor_y.runToNewPosition(y);
  motor_z1.runToNewPosition(z1);
  motor_z2.runToNewPosition(z2);
  
}
// Moves each motor to a given position starting with the z-axis to prevent the pintool/gripper from hitting anything on the workspace
void move_to_coordinate_z_first(long x, long y, long z1, long z2){
// void move_to_coordinate_z_first(long *coordinates){

  motor_z1.setSpeed(SPEED_Z);
  motor_z2.setSpeed(SPEED_Z);
  gantry.setSpeed(SPEED_GANTRY);
  motor_y.setSpeed(SPEED_Y);

  motor_z1.runToNewPosition(z1);
  motor_z2.runToNewPosition(z2);
  gantry.runToNewPosition(x);
  motor_y.runToNewPosition(y);
}

// Take plate from a stack
// stack ==  
//    false for cell
//    true for chemical
void take_from_stack(boolean stack, int height_to_pick_from){

  // chemical stack
  if (stack){
    
    // Position gripper over stack
    // z1 is set to 0 because we dont know height yet of first plate to grab so bring it all the way up
    move_to_coordinate_x_first(Chemical_Input_Stack_X, Chemical_Input_Stack_Y, Z_HIGH, height_to_pick_from);
    // move_to_coordinate_x_first(chemical_stack_1);
    close_gripper();
    // z2 set to 0 to bring plate up ... might change to something else later but 0 for now
    // Moving up gripper
    move_to_coordinate_z_first(Chemical_Input_Stack_X, Chemical_Input_Stack_Y, Z_HIGH, Z_HIGH);
    // Move to pin transfer area
    move_to_coordinate_x_first(Chemical_Transfer_Area_Gripper_X, Chemical_Transfer_Area_Gripper_Y, Z_HIGH, BASE_PLATE_STEPS);
    open_gripper();
    move_to_coordinate_x_first(Chemical_Transfer_Area_Gripper_X, Chemical_Transfer_Area_Gripper_Y, Z_HIGH, Z_HIGH);
  }
  // cell stack
  else{

    // Position gripper over stack
    // z1 is set to 0 because we dont know height yet of first plate to grab so bring it all the way up
    move_to_coordinate_x_first(Cell_Input_Stack_X, Cell_Input_Stack_Y, Z_HIGH, height_to_pick_from);
    close_gripper();
    // z2 set to 0 to bring plate up ... might change to something else later but 0 for now
    // Moving up gripper
    move_to_coordinate_z_first(Cell_Input_Stack_X, Cell_Input_Stack_Y, Z_HIGH, Z_HIGH);
    // Move to pin transfer area
    move_to_coordinate_x_first(Cell_Transfer_Area_Gripper_X, Cell_Transfer_Area_Gripper_Y, Z_HIGH, BASE_PLATE_STEPS);
    open_gripper();
    move_to_coordinate_x_first(Cell_Transfer_Area_Gripper_X, Cell_Transfer_Area_Gripper_Y, Z_HIGH, Z_HIGH);
  }
}

// TODO
// Put a plate onto a stack
void push_onto_stack(int stack, int height_to_put_on){

  // chemical stack
  if (stack){
    // Position gripper over stack
    // z1 is set to 0 because we dont know height yet of first plate to grab so bring it all the way up
    move_to_coordinate_x_first(Chemical_Transfer_Area_Gripper_X, Chemical_Transfer_Area_Gripper_Y, Z_HIGH, BASE_PLATE_STEPS);
    close_gripper();
    // z2 set to 0 to bring plate up ... might change to something else later but 0 for now
    // Moving up gripper
    move_to_coordinate_z_first(Chemical_Transfer_Area_Gripper_X, Chemical_Transfer_Area_Gripper_Y, Z_HIGH, Z_HIGH);
    // Move to pin transfer area
    move_to_coordinate_x_first(Chemical_Output_Stack_X, Chemical_Output_Stack_Y, Z_HIGH, height_to_put_on);
    open_gripper();
    move_to_coordinate_x_first(Chemical_Output_Stack_X, Chemical_Output_Stack_Y, Z_HIGH, Z_HIGH);
  }
  // cell stack
  else{
    // Position gripper over stack
    // z1 is set to 0 because we dont know height yet of first plate to grab so bring it all the way up
    move_to_coordinate_x_first(Cell_Transfer_Area_Gripper_X, Cell_Transfer_Area_Gripper_Y, Z_HIGH, BASE_PLATE_STEPS);
    close_gripper();
    // z2 set to 0 to bring plate up ... might change to something else later but 0 for now
    // Moving up gripper
    move_to_coordinate_z_first(Cell_Transfer_Area_Gripper_X, Cell_Transfer_Area_Gripper_Y, Z_HIGH, Z_HIGH);
    // Move to pin transfer area
    move_to_coordinate_x_first(Cell_Output_Stack_X, Cell_Output_Stack_Y, Z_HIGH, height_to_put_on);
    open_gripper();
    move_to_coordinate_x_first(Cell_Output_Stack_X, Cell_Output_Stack_Y, Z_HIGH, Z_HIGH);
  }
}

// TODO
void do_wash(int wash_step){
  
  // Solution 1
  if(wash_step == 0){
    // Move to Solution 1
    for(int i = 0; i < 1; i++){
      move_to_coordinate_x_first(Solution_1_X, Solution_1_Y, Solution_1_Z1, Z_HIGH);
      delay(time_in_solution_ms);
      move_to_coordinate_x_first(Solution_1_X, Solution_1_Y, Z_HIGH, Z_HIGH);  
    }
  }
  // Solution 2
  else if(wash_step == 1){
    // Move to Solution 2
    for(int i = 0; i < 3;i++){
      move_to_coordinate_x_first(Solution_2_X, Solution_2_Y, Solution_2_Z1, Z_HIGH);
      delay(time_in_solution_ms);
      move_to_coordinate_x_first(Solution_2_X, Solution_2_Y, Z_HIGH, Z_HIGH);     
    }
  }
  // Solution 3
  else if(wash_step == 2){
    // Move to Solution 3
    for(int i = 0;i < 3; i++){
      move_to_coordinate_x_first(Solution_3_X, Solution_3_Y, Solution_3_Z1, Z_HIGH);
      delay(time_in_solution_ms);
      move_to_coordinate_x_first(Solution_3_X, Solution_3_Y, Z_HIGH, Z_HIGH);   
    }
  }
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
void do_fan_and_heat(){
  
  // Move pin tool over the fan
  move_to_coordinate_x_first(Fan_Heater_X, Fan_Heater_Y, Fan_Heater_Z1, Z_HIGH);
  fan_on();
  heat_on();
  delay(drying_time_ms);
  heat_off();
  fan_off();
  delay(drying_time_ms);
  move_to_coordinate_x_first(Fan_Heater_X , Fan_Heater_Y, Z_HIGH, Z_HIGH);
}

// TODO
// Perform a single pin transfer and bring the pin back to its starting position.
void do_pin_transfer(int pin_depth){

  long depth =  PINS_ABOVE_WELL_PLATE_OPENING_STEPS - convert_mm_to_steps(pin_depth);
  

  // Move to chemical plate to absorb chemicals in pin tool also dipping pin tool in chemical plate
  move_to_coordinate_x_first(Chemical_Transfer_Area_Pin_Tool_X,Chemical_Transfer_Area_Pin_Tool_Y, depth, Z_HIGH);
  delay(time_for_full_absorption_of_chemicals_in_ms);

  // Moving pin tool out of chemical plate
  move_to_coordinate_z_first(Chemical_Transfer_Area_Pin_Tool_X, Chemical_Transfer_Area_Pin_Tool_Y, Z_HIGH,Z_HIGH);
  // Moving pin tool to cell plate and transfer chemicals to cells
  move_to_coordinate_x_first(Cell_Transfer_Area_Pin_Tool_X, Cell_Transfer_Area_Pin_Tool_Y, depth, Z_HIGH);
  delay(time_for_full_transfer_of_chemicals_in_ms);
  // Moving pin tool out of cell plate
  move_to_coordinate_z_first(Cell_Transfer_Area_Pin_Tool_X, Cell_Transfer_Area_Pin_Tool_Y, Z_HIGH, Z_HIGH);

}

// Wash the pin tool.
void wash_pin_tool(boolean * wash_steps){
  // 1. Move pin tool to wash step linear actuator
  for(int wash_step = 0; wash_step < 3; wash_step++){
    if(wash_steps[wash_step]){
      do_wash(wash_step);
    }
  }
}


void do_cycle(boolean *wash_steps, int pin_depth, int grab_height, int stack_height, int plateNum){
  
  take_from_stack(true, grab_height);
  take_from_stack(false, grab_height);
  do_pin_transfer(pin_depth);
   wash_pin_tool(wash_steps);
  do_fan_and_heat();
  push_onto_stack(true, stack_height);
  push_onto_stack(false, stack_height);
}

void run_all_cycles(boolean * wash_steps, short num_plates, int pin_depth ){

  int grab_height = BASE_PLATE_STEPS + (Plate_Offset * (num_plates - 1));
  int stack_height = BASE_PLATE_STEPS;
  
  for(int i = 0; i < num_plates; i++){
    
    do_cycle(wash_steps, pin_depth, grab_height, stack_height , i+1);

    // Update where the next plate is located at.
    grab_height -= Plate_Offset;
    stack_height += Plate_Offset;
  }
}

/*###########################################################################################*/
/*LCD Functions*/


/*###########################################################################################*/

void run_startup(){
  // TESTING: Begin serial connection for debugging
  Serial.begin(9600);

  // Set the state of any pins used as inputs
  set_pins();
  

  // Set the max speed and acceleration values for each motor
  configure_motors();


  while(true){
    servo.write(0);
    if(digitalRead(x_switch)== LOW){
      Serial.println("Starting calibration.");
      break;
    }
  }
  // Find reference positions
  calibrate_motors();

}


void setup() {
  Serial.begin(9600);
  run_startup();
}

void loop() {

  
  boolean steps[3] =  {true, false, false };
  int num_plates = 2;
  int depth = 10;

  run_all_cycles(steps, num_plates, depth);
}
