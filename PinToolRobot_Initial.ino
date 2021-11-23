// Motors
#include <AccelStepper.h>
#include <MultiStepper.h>

// Servos
#include <Servo.h>

// LCD
#include <Adafruit_TFTLCD.h>
#include <Adafruit_GFX.h>
#include <TouchScreen.h>

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
#define dirPinx   46
#define stepPinx  47 

#define dirPiny   40
#define stepPiny  41

#define dirPinz1  42
#define stepPinz1 43

#define dirPinz2  44
#define stepPinz2 45

/*###########################################################################################*/
/*Servo Pins*/

#define SERVO_PIN 10

// Servo constants
#define CLOSE 50
#define OPEN  0

/*###########################################################################################*/
/* LCD Pin Definitions*/

// Touch Screen Pins
#define YP A3  
#define XM A2  
#define YM 9   
#define XP 8 

// LCD Pins
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET A4

// HEX color defines
#define BLACK 0x0000
#define WHITE 0xFFFF


// 13 on the UNO myabe something different on the mega2560
#define LCD_CLOCK 13

// Instantiate touch screen and lcd objects
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);

/*###########################################################################################*/
/*Other Pin Definitions*/

// Limit switch pins definitions
#define y_switch 26
#define x_switch 25
#define z1_switch 24
#define z2_switch 29

// Pins for fan and heater N-Channel MOSFET gate pin
#define fan_pin         4
#define heater_pin      4

/*###########################################################################################*/
/*CONTSTANTS*/

// Motor speeds
#define SPEED_GANTRY 100
#define SPEED_Y      100
#define SPEED_Z      100
#define SPEED_Z1     100
#define SPEED_Z2     100


// Speed of all the motors
#define MAX_SPEED             500
#define MAX_ACCELERATION      1000
#define MAX_RUN_DISTANCE      100

#define DELAY                 500 // Half a second

#define plate_height_in_steps 1024

#define plate_height_in_mm    1024

#define steps_per_mm          1024

#define heat_and_fan_delay    5000


// Pin tool only
long solution_1 {};
long solution_2 {};
long solution_3 {};

long fan {};

// Gripper only
// Input
long cell_stack_1 {};
// Output
long cell_stack_2 {};

// Input
long chemical_stack_1 {};
// Output
long chemical_stack_2 {};

// Pin tool and gripper
long cell_area_pintool {};
long cell_area_gripper {};

long chemical_area_pintool {};
long chemical_area_gripper {};


/*###########################################################################################*/

// Instantiating motor driver objects
AccelStepper gantry   = AccelStepper(interface, stepPinx, dirPinx);
AccelStepper motor_y  = AccelStepper(interface, stepPiny, dirPiny);
AccelStepper motor_z1 = AccelStepper(interface, stepPinz1, dirPinz1);
AccelStepper motor_z2 = AccelStepper(interface, stepPinz2, dirPinz2);

Servo servo = Servo();

/*###########################################################################################*/

// ! NEED
// Converts a distance in mm to steps
long convert_mm_to_steps(float mm){
  long steps = mm * steps_per_mm;
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

// WORKING
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
// void gripper_movement_test(){ 
void calibrate_motors(){ 

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
  gantry.setSpeed(SPEED_GANTRY);
  motor_y.setSpeed(SPEED_Y);
  

  gantry.runToNewPosition(x_start);
  motor_y.runToNewPosition(y_start);
  motor_z1.runToNewPosition(z1_start);
  motor_z2.runToNewPosition(z2_start);
}

// Used for returning the absolute coordinates of a certain position
void get_absolute_positions(){ 
  int val;

  gripper(OPEN, servo);
  while(true){
    if(digitalRead(x_switch) == LOW){
      if(val == CLOSE){
        val = OPEN;
      }else{
        val = CLOSE;
      }    
      gripper(val, servo);
    }else if(digitalRead(y_switch) == LOW){
      break;
    }
  }

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
  gantry.setSpeed(SPEED_GANTRY);
  motor_y.setSpeed(SPEED_Y);
  
  
  // Print the coordinate from where the motor started
  Serial.println("X: " + String(x_start) + "\nY: " + String(y_start) + "\nZ1: " + String(z1_start) + "\nZ2: " + String(z2_start));

  gantry.runToNewPosition(x_start);
  motor_y.runToNewPosition(y_start);
  motor_z1.runToNewPosition(z1_start);
  motor_z2.runToNewPosition(z2_start);
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


void move_to_xy(long *xy_coordinate){

}

void raise_pintool(){

}

void raise_gripper(){

}

// TODO
// Take plate from a stack
// stack ==  
//    false for cell
//    true for chemical
void take_from_stack(boolean stack, int height_to_pick_from){

  #define x_over_chemical_stack -1
  #define y_over_chemical_stack -1
  #define x_over_pin_transfer_area -1
  #define y_over_pin_transfer_for_chemical -1
  #define z2_on_chemical_area_on_workspace -1

  #define x_over_cell_stack -1
  #define y_over_cell_stack -1
  #define x_over_pin_transfer_area -1
  #define y_over_pin_transfer_for_cell -1
  #define z2_on_cell_area_on_workspace -1
  
  // chemical stack
  if (stack){

    // ! Might have to move the z2 motor up to -100 before calling the function below
    // Position gripper over stack
    // z1 is set to 0 because we dont know height yet of first plate to grab so bring it all the way up
    move_to_coordinate_x_first(x_over_chemical_stack, y_over_chemical_stack, -100, height_to_pick_from);
    // move_to_coordinate_x_first(chemical_stack_1);
    close_gripper();
    // z2 set to 0 to bring plate up ... might change to something else later but 0 for now
    // Moving up gripper
    move_to_coordinate_z_first(x_over_chemical_stack, y_over_chemical_stack, -100, -100);
    // Move to pin transfer area
    move_to_coordinate_x_first(x_over_pin_transfer_area, y_over_pin_transfer_for_chemical, -100, z2_on_chemical_area_on_workspace);
    open_gripper();
    move_to_coordinate_x_first(x_over_pin_transfer_area, y_over_pin_transfer_for_chemical, -100, -100);
  }
  // cell stack
  else{

    // ! Might have to move the z2 motor up to -100 before calling the function below 
    // Position gripper over stack
    // z1 is set to 0 because we dont know height yet of first plate to grab so bring it all the way up
    move_to_coordinate_x_first(x_over_cell_stack, y_over_cell_stack, -100, height_to_pick_from);
    close_gripper();
    // z2 set to 0 to bring plate up ... might change to something else later but 0 for now
    // Moving up gripper
    move_to_coordinate_z_first(x_over_cell_stack, y_over_cell_stack, -100, -100);
    // Move to pin transfer area
    move_to_coordinate_x_first(x_over_pin_transfer_area, y_over_pin_transfer_for_cell, -100, z2_on_cell_area_on_workspace);
    open_gripper();
    move_to_coordinate_x_first(x_over_pin_transfer_area, y_over_pin_transfer_for_cell, -100, -100);
  }
}

// TODO
// Put a plate onto a stack
void push_onto_stack(int stack, int height_to_put_on){

  #define x_over_chemical_plate_after_transfer -1
  #define y_over_chemical_plate_after_transfer -1
  #define chemical_plate_on_base -1
  #define x_over_pin_chemical_output_stack -1
  #define y_over_chemical_output_stack -1
  #define z2_on_chemical_stack_at_whatever_height -1

  #define x_over_cell_plate_after_transfer -1
  #define y_over_cell_plate_after_transfer -1
  #define cell_plate_on_base -1
  #define x_over_pin_cell_output_stack -1
  #define y_over_cell_output_stack -1
  #define z2_on_cell_stack_at_whatever_height -1
  


  // chemical stack
  if (stack){
    // Position gripper over stack
    // z1 is set to 0 because we dont know height yet of first plate to grab so bring it all the way up
    move_to_coordinate_x_first(x_over_chemical_plate_after_transfer, y_over_chemical_plate_after_transfer, -100, chemical_plate_on_base);
    close_gripper();
    // z2 set to 0 to bring plate up ... might change to something else later but 0 for now
    // Moving up gripper
    move_to_coordinate_z_first(x_over_chemical_plate_after_transfer, y_over_chemical_plate_after_transfer, -100, -100);
    // Move to pin transfer area
    move_to_coordinate_x_first(x_over_pin_chemical_output_stack, y_over_chemical_output_stack, -100, z2_on_chemical_stack_at_whatever_height);
    open_gripper();
    move_to_coordinate_x_first(x_over_pin_chemical_output_stack, y_over_chemical_output_stack, -100, -100);
  }
  // cell stack
  else{
    // Position gripper over stack
    // z1 is set to 0 because we dont know height yet of first plate to grab so bring it all the way up
    move_to_coordinate_x_first(x_over_cell_plate_after_transfer, y_over_cell_plate_after_transfer, -100, cell_plate_on_base);
    close_gripper();
    // z2 set to 0 to bring plate up ... might change to something else later but 0 for now
    // Moving up gripper
    move_to_coordinate_z_first(x_over_cell_plate_after_transfer, y_over_cell_plate_after_transfer, -100, -100);
    // Move to pin transfer area
    move_to_coordinate_x_first(x_over_pin_cell_output_stack, y_over_cell_output_stack, -100, z2_on_cell_stack_at_whatever_height);
    open_gripper();
    move_to_coordinate_x_first(x_over_pin_cell_output_stack, y_over_cell_output_stack, -100, -100);
  }
}

// TODO
void do_wash(short wash_step){

  #define pintool_into_solution -1
  
  #define x_over_solution_1 -1
  #define y_over_solution_1 -1

  #define x_over_solution_2 -1
  #define y_over_solution_2 -1

  #define x_over_solution_3 -1
  #define y_over_solution_3 -1


  #define time_in_solution_ms -1
  
  // Solution 1
  if(wash_step == 1){
    // Move to Solution 1
    move_to_coordinate_x_first(x_over_solution_1, y_over_solution_1, pintool_into_solution, -100);
    delay(time_in_solution_ms);
    move_to_coordinate_x_first(x_over_solution_1, y_over_solution_1, -100, -100);
  }
  // Solution 2
  else if(wash_step == 2){
    // Move to Solution 2
    move_to_coordinate_x_first(x_over_solution_2, y_over_solution_2, pintool_into_solution, -100);
    delay(time_in_solution_ms);
    move_to_coordinate_x_first(x_over_solution_2, y_over_solution_2, -100, -100);   
  }
  // Solution 3
  else if(wash_step == 3){
    // Move to Solution 3
    move_to_coordinate_x_first(x_over_solution_3, y_over_solution_3, pintool_into_solution, -100);
    delay(time_in_solution_ms);
    move_to_coordinate_x_first(x_over_solution_3, y_over_solution_3, -100, -100); 
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
void do_fan_and_heat(int drying_time_ms){

  #define x_over_fan -1
  #define y_over_fan -1
  #define pin_tool_over_fan -1
  
  // Move pin tool over the fan
  move_to_coordinate_x_first(x_over_fan, y_over_fan, pin_tool_over_fan, -100);
  fan_on();
  heat_on();
  delay(drying_time_ms);
  heat_off();
  fan_off();
  delay(drying_time_ms);
  move_to_coordinate_x_first(x_over_fan, y_over_fan, -100, -100);
}

// TODO
// Perform a single pin transfer and bring the pin back to its starting position.
void do_pin_transfer(long pin_depth){

  #define x_over_pin_transfer_chemical_area -1
  #define y_over_pin_transfer_chemical_area -1

  #define time_for_full_absorption_of_chemicals_in_ms -1
  #define time_for_full_transfer_of_chemicals_in_ms -1

  #define y_over_pin_transfer_cell_area -1
  

  long depth = convert_mm_to_steps(pin_depth);

  // Move to chemical plate to absorb chemicals in pin tool also dipping pin tool in chemical plate
  move_to_coordinate_x_first(x_over_pin_transfer_chemical_area,y_over_pin_transfer_chemical_area, depth, -100);
  delay(time_for_full_absorption_of_chemicals_in_ms);

  // Moving pin tool out of chemical plate
  move_to_coordinate_z_first(x_over_pin_transfer_chemical_area, y_over_pin_transfer_chemical_area, -100, -100);
  // Moving pin tool to cell plate and transfer chemicals to cells
  move_to_coordinate_x_first(x_over_pin_transfer_cell_area, y_over_pin_transfer_cell_area, depth, -100);
  delay(time_for_full_transfer_of_chemicals_in_ms);
  // Moving pin tool out of cell plate
  move_to_coordinate_z_first(x_over_pin_transfer_cell_area, y_over_pin_transfer_cell_area, -100, -100);
}

// TODO
// Wash the pin tool.
void wash_pin_tool(boolean * wash_steps){
  // 1. Move pin tool to wash step linear actuator
  for(int wash_step = 0; wash_step < 3; wash_step++){
    if(wash_steps[wash_step]){
      do_wash(wash_step);
    }
  }
}

// TODO
// TEST
void do_cycle(boolean *wash_steps, int pin_depth, int drying_time, int height_of_next_plate_in_steps_input_stack, int height_of_next_plate_in_steps_output_stack){
  // Chemical stack
   take_from_stack(true, height_of_next_plate_in_steps_input_stack);
   // Cell stack
   take_from_stack(false, height_of_next_plate_in_steps_input_stack);
   do_pin_transfer(pin_depth);
   wash_pin_tool(wash_steps);
   do_fan_and_heat(drying_time);
   push_onto_stack(true, height_of_next_plate_in_steps_output_stack);
   push_onto_stack(false, height_of_next_plate_in_steps_output_stack);
}

// TODO
// TEST
//void run_all_cycles(short num_plates, short num_wash_steps, int pin_depth, int drying_time){/
void run_all_cycles(boolean *wash_steps, short num_plates, int pin_depth ){

  #define drying_time -1

   
  double height_of_stack = (num_plates * plate_height_in_mm);
  // NEED TO DETERMINE THESE
  double height_of_next_plate_in_steps_input_stack = convert_mm_to_steps(height_of_stack);
  double height_of_next_plate_in_steps_output_stack;
  

  for(int i = 0; i < num_plates; i++){
    
    do_cycle(wash_steps, pin_depth, drying_time, height_of_next_plate_in_steps_input_stack, height_of_next_plate_in_steps_output_stack);

    // Update where the next plate is located at.
//    height_of_next_plate_to_grab -= plate_height_in_steps;
  }
}

/*###########################################################################################*/
/*LCD Functions*/

// TODO: use the correct pin number
// LCD configurations on startup
void configure_lcd()
{
  tft.reset();
  uint16_t identifier = tft.readID();
  tft.begin(identifier);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  pinMode(13, OUTPUT);
}


// DONE
// Intial greeting at machine start up
void greeting()
{
  tft.fillScreen(BLACK);
  tft.setCursor(40,120);
  tft.println("Welcome to the");
  tft.setCursor(43,140);
  tft.println("automated Pin");
  tft.setCursor(40,160);
  tft.println("Transfer Tool!");
  // delay(5000);
}

// Done
// Screen setup for plate input
void plateNumberSetup()
{
  // Reset screen
  tft.fillScreen(BLACK);
  // Instructions
  tft.setCursor(33,20);
  tft.println("How many plates");
  tft.setCursor(38,40);
  tft.println("would you like");
  tft.setCursor(75,60);
  tft.println("to use?");
  
  // Buttons
  tft.drawRect(70, 135, 30, 30, WHITE);
  tft.setCursor(80,143);
  tft.println("1");
  tft.drawRect(105, 135, 30, 30, WHITE);
  tft.setCursor(115,143);
  tft.println("2");
  tft.drawRect(140, 135, 30, 30, WHITE);
  tft.setCursor(150,143);
  tft.println("3");
  tft.drawRect(70, 170, 30, 30, WHITE);
  tft.setCursor(80,178);
  tft.println("4");
  tft.drawRect(105, 170, 30, 30, WHITE);
  tft.setCursor(115,178);
  tft.println("5");
  tft.drawRect(140, 170, 30, 30, WHITE);
  tft.setCursor(150,178);
  tft.println("6");
  tft.drawRect(70, 205, 30, 30, WHITE);
  tft.setCursor(80,213);
  tft.println("7");
  tft.drawRect(105, 205, 30, 30, WHITE);
  tft.setCursor(115,213);
  tft.println("8");
  tft.drawRect(140, 205, 30, 30, WHITE);
  tft.setCursor(150,213);
  tft.println("9");
  tft.drawRect(105, 240, 30, 30, WHITE);
  tft.setCursor(115,248);
  tft.println("0");
  tft.setCursor(80, 278);
  tft.println("CONFIRM");
  tft.setCursor(90, 300);
  tft.println("CLEAR");
}

// TODO: use correct pins on write
// User inputs number of plates they wish to use
int plateNumberInput()
{
  int plateNum = -1;
  while (true)
  {
    digitalWrite(13, HIGH);
    TSPoint point = ts.getPoint();
    digitalWrite(13, LOW);
    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);
    if (point.z  >= 200 && point.z <= 1500)
    {
      int x = map(point.x, 78, 951, 0, 320);
      int y = map(point.y, 96, 921, 0, 240);

      if (x >= 185 && x <= 220 && y >= 109 && y <= 127)
      {
        tft.fillRect(105, 90, 30, 30, BLACK);
        plateNum = 1;
        tft.setCursor(115, 100);
        tft.print("1");
        delay(1000);
      }
      else if (x >= 140 && x <= 175 && y >= 107 && y <= 129)
      {
        tft.fillRect(105, 90, 30, 30, BLACK);
        plateNum = 2;
        tft.setCursor(115, 100);
        tft.print("2");
        delay(1000);
      }
      else if (x >= 95 && x <= 132 && y >= 107 && y <= 129)
      {
        tft.fillRect(105, 90, 30, 30, BLACK);
        tft.setCursor(115, 100);
        tft.print("3");
        plateNum = 3;
        delay(1000);
      }
      else if (x >= 185 && x <= 220 && y >= 80 && y <= 101)
      {
        tft.fillRect(105, 90, 30, 30, BLACK);
        tft.setCursor(115, 100);
        tft.print("4");
        plateNum = 4;
        delay(1000);
      }
      else if (x >= 140 && x <= 175 && y >= 80 && y <= 101)
      {
        tft.fillRect(105, 90, 30, 30, BLACK);
        tft.setCursor(115, 100);
        tft.print("5");
        plateNum = 5;
        delay(1000);
      }
      else if (x >= 95 && x <= 132 && y >= 80 && y <= 101)
      {
        tft.fillRect(105, 90, 30, 30, BLACK);
        tft.setCursor(115, 100);
        tft.print("6");
        plateNum = 6;
        delay(1000);
      }
      else if (x >= 185 && x <= 220 && y >= 55 && y <= 75)
      {
        tft.fillRect(105, 90, 30, 30, BLACK);
        tft.setCursor(115, 100);
        tft.print("7");
        plateNum = 7;
        delay(1000);
      }
      else if (x >= 140 && x <= 175 && y >= 55 && y <= 75)
      {
        tft.fillRect(105, 90, 30, 30, BLACK);
        tft.setCursor(115, 100);
        tft.print("8");
        plateNum = 8;
        delay(1000);
      }
      else if (x >= 95 && x <= 132 && y >= 55 && y <= 75)
      {
        tft.fillRect(105, 90, 30, 30, BLACK);
        tft.setCursor(115, 100);
        tft.print("9");
        plateNum = 9;
        delay(1000);
      }
      else if (x >= 140 && x <= 175 && y >= 28 && y <= 50)
      {
        tft.fillRect(105, 90, 30, 30, BLACK);
        tft.setCursor(115, 100);
        tft.print("0");
        plateNum = 0;
        delay(1000);
      }
      else if (x >= 103 && x <= 208 && y >= 13 && y <= 22 && plateNum != -1)
        break;
      else if (x >= 120 && x <= 195 && y >= -2 && y <= 6)
      {
        plateNum = -1;
        tft.fillRect(105, 90, 30, 30, BLACK);
      }
    }
  }
  return plateNum;
}

// DONE
// create the screen asking for pin tool depth
void depthSetup()
{
  // Reset screen
  tft.fillScreen(BLACK);
  // Instructions
  tft.setCursor(33,20);
  tft.println("How deep would");
  tft.setCursor(13,40);
  tft.println("you like to insert");
  tft.setCursor(40,60);
  tft.println("the pin tool?");
  
  // Buttons
  tft.drawRect(70, 135, 30, 30, WHITE);
  tft.setCursor(80,143);
  tft.println("1");
  tft.drawRect(105, 135, 30, 30, WHITE);
  tft.setCursor(115,143);
  tft.println("2");
  tft.drawRect(140, 135, 30, 30, WHITE);
  tft.setCursor(150,143);
  tft.println("3");
  tft.drawRect(70, 170, 30, 30, WHITE);
  tft.setCursor(80,178);
  tft.println("4");
  tft.drawRect(105, 170, 30, 30, WHITE);
  tft.setCursor(115,178);
  tft.println("5");
  tft.drawRect(140, 170, 30, 30, WHITE);
  tft.setCursor(150,178);
  tft.println("6");
  tft.drawRect(70, 205, 30, 30, WHITE);
  tft.setCursor(80,213);
  tft.println("7");
  tft.drawRect(105, 205, 30, 30, WHITE);
  tft.setCursor(115,213);
  tft.println("8");
  tft.drawRect(140, 205, 30, 30, WHITE);
  tft.setCursor(150,213);
  tft.println("9");
  tft.drawRect(105, 240, 30, 30, WHITE);
  tft.setCursor(115,248);
  tft.println("0");
  tft.setCursor(80, 278);
  tft.println("CONFIRM");
  tft.setCursor(90, 300);
  tft.println("CLEAR");
}

// TODO: use correct pins on write
// take input for how deep the pin tool needs to go
int depthInput()
{
  String depth = "";
  int x_cursor = 115;
  while (true)
  {
    digitalWrite(13, HIGH);
    TSPoint point = ts.getPoint();
    digitalWrite(13, LOW);
    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);
    if (point.z  >= 200 && point.z <= 1500)
    {     
      int x = map(point.x, 78, 951, 0, 320);
      int y = map(point.y, 96, 921, 0, 240);

      if (x >= 185 && x <= 220 && y >= 109 && y <= 127)
      {
        tft.fillRect(0, 90, 240, 30, BLACK);
        tft.setCursor(x_cursor, 100);
        x_cursor -= 7;
        depth.concat("1");
        tft.print(depth);
        delay(1000);
      }
      else if (x >= 140 && x <= 175 && y >= 107 && y <= 129)
      {
        tft.fillRect(0, 90, 240, 30, BLACK);
        tft.setCursor(x_cursor, 100);
        x_cursor -= 7;
        depth.concat("2");
        tft.print(depth);
        delay(1000);
      }
      else if (x >= 95 && x <= 132 && y >= 107 && y <= 129)
      {
        tft.fillRect(0, 90, 240, 30, BLACK);
        tft.setCursor(x_cursor, 100);
        x_cursor -= 7;
        depth.concat("3");
        tft.print(depth);
        delay(1000);
      }
      else if (x >= 185 && x <= 220 && y >= 80 && y <= 101)
      {
        tft.fillRect(0, 90, 240, 30, BLACK);
        tft.setCursor(x_cursor, 100);
        x_cursor -= 7;
        depth.concat("4");
        tft.print(depth);
        delay(1000);
      }
      else if (x >= 140 && x <= 175 && y >= 80 && y <= 101)
      {
        tft.fillRect(0, 90, 240, 30, BLACK);
        tft.setCursor(x_cursor, 100);
        x_cursor -= 7;
        depth.concat("5");
        tft.print(depth);
        delay(1000);
      }
      else if (x >= 95 && x <= 132 && y >= 80 && y <= 101)
      {
        tft.fillRect(0, 90, 240, 30, BLACK);
        tft.setCursor(x_cursor, 100);
        x_cursor -= 7;
        depth.concat("6");
        tft.print(depth);
        delay(1000);
      }
      else if (x >= 185 && x <= 220 && y >= 55 && y <= 75)
      {
        tft.fillRect(0, 90, 240, 30, BLACK);
        tft.setCursor(x_cursor, 100);
       x_cursor -= 7;
        depth.concat("7");
        tft.print(depth);
        delay(1000);
      }
      else if (x >= 140 && x <= 175 && y >= 55 && y <= 75)
      {
        tft.fillRect(0, 90, 240, 30, BLACK);
        tft.setCursor(x_cursor, 100);
        x_cursor -= 7;
        depth.concat("8");
        tft.print(depth);
        delay(1000);
      }
      else if (x >= 95 && x <= 132 && y >= 55 && y <= 75)
      {
        tft.fillRect(0, 90, 240, 30, BLACK);
        tft.setCursor(x_cursor, 100);
        x_cursor -= 7;
        depth.concat("9");
        tft.print(depth);
        delay(1000);
      }
      else if (x >= 140 && x <= 175 && y >= 28 && y <= 50)
      {
        tft.fillRect(0, 90, 240, 30, BLACK);
        tft.setCursor(x_cursor, 100);
        x_cursor -= 7;
        depth.concat("0");
        tft.print(depth);
        delay(1000);
      }
      else if (x >= 103 && x <= 208 && y >= 13 && y <= 22 && depth != "")
        break;
      else if (x >= 120 && x <= 195 && y >= -2 && y <= 6)
      {
        x_cursor = 115;
        depth = "";
        tft.fillRect(0, 90, 240, 30, BLACK);
      }
    }
  }
  return depth.toInt();
}

// DONE
// Create the screen that asks about wash steps
void washStepSetup()
{
  // Reset screen
  tft.fillScreen(BLACK);
  // Instructions
  tft.setCursor(19, 20);
  tft.println("Select which wash");
  tft.setCursor(30, 40);
  tft.println("steps you would");
  tft.setCursor(47, 60);
  tft.println("like to use.");

  // Check boxes
  tft.drawRect(150, 115, 30, 30, WHITE);
  tft.drawRect(150, 165, 30, 30, WHITE);
  tft.drawRect(150, 215, 30, 30, WHITE);

  // Wash step numbers
  tft.setCursor(55, 123);
  tft.print("1 -----");
  tft.setCursor(55, 173);
  tft.print("2 -----");
  tft.setCursor(55, 223);
  tft.print("3 -----");
  tft.setCursor(80, 280);
  tft.print("CONFIRM");
}

// TODO: use correct pins on write
// take input for the wash steps to use
void washStepInput(bool * steps)
{
  steps[0] = false;
  steps[1] = false;
  steps[2] = false;
      
  while (true)
  {
    digitalWrite(13, HIGH);
    TSPoint point = ts.getPoint();
    digitalWrite(13, LOW);
    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);
    if (point.z  >= 200 && point.z <= 1500)
    {
      int x = map(point.x, 78, 951, 0, 320);
      int y = map(point.y, 96, 921, 0, 240);

      if (x >= 83 && x <= 119 && y >= 122 && y <= 145)
      {
        if (!steps[0])
        {
          tft.drawLine(151, 116, 178, 143, WHITE);
          tft.drawLine(178, 116, 151, 143, WHITE);
          steps[0] = true;
          delay(500);
        }
        else
        {
          tft.fillRect(151, 116, 28, 28, BLACK);
          steps[0] = false;
          delay(500);
        }         
      }
      else if (x >= 83 && x <= 119 && y >= 83 && y <= 108)
      { 
        if (!steps[1])
        {
          tft.drawLine(151, 166, 178, 193, WHITE);
          tft.drawLine(178, 166, 151, 193, WHITE);
          steps[1] = true;
          delay(500);
        }
        else
        {
          tft.fillRect(151, 166, 28, 28, BLACK);
          steps[1] = false;
          delay(500);
        }      
      }
      else if (x >= 83 && x <= 119 && y >= 45 && y <= 70)
      {  
        if (!steps[2])
        {
          tft.drawLine(151, 216, 178, 243, WHITE);
          tft.drawLine(178, 216, 151, 243, WHITE);
          steps[2] = true;
          delay(500);
        }
        else
        {
          tft.fillRect(151, 216, 28, 28, BLACK);
          steps[2] = false;
          delay(500);
        }     
      }
      else if (x >= 105 && x <= 211 && y >= 10 && y <= 22) 
        break;
    }
  }
}

// TODO: use correct pins on write
// confirm with the user that their inputs are correct
bool paramCheck(int plateNum, int depth, bool * steps)
{
  tft.fillScreen(BLACK);
  tft.setCursor(19, 20);
  tft.println("Accept parameters");
  tft.setCursor(50, 40);
  tft.println("and continue");
  tft.setCursor(62, 60);
  tft.println("to the pin");
  tft.setCursor(20, 80);
  tft.println("transfer process?");

  tft.setCursor(0, 140);
  tft.print("Number of Plates: ");
  tft.print(plateNum);
  tft.setCursor(0, 160);
  tft.print("Pin Tool Depth: ");
  tft.print(depth);
  tft.setCursor(0, 180);
  tft.print("Wash Steps: ");
  if (steps[0] == 1)
    tft.print("1 ");
  if (steps[1] == 1)
    tft.print("2 ");
  if (steps[2] == 1)
    tft.print("3 ");

  tft.setCursor(20, 240);
  tft.print("CONFIRM");
  tft.setCursor(173, 240);
  tft.print("REDO");

  while (true)
  {
    digitalWrite(13, HIGH);
    TSPoint point = ts.getPoint();
    digitalWrite(13, LOW);
    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);
    if (point.z  >= 200 && point.z <= 1500)
    {
      int x = map(point.x, 78, 951, 0, 320);
      int y = map(point.y, 96, 921, 0, 240);

      if (x >= 182 && x <= 283 && y >= 37 && y <= 50)
        return true;
      else if (x >= 35 && x <= 93 && y >= 37 && y <= 50)
        return false;
    }
  }
  return false;
}

// TODO: need operation location names and maybe move the names down a line
// Show pin tool operation progress
void progressScreen(int plateNum, String location)
{
  tft.fillRect(134, 115, 30, 30, BLACK);
  tft.fillRect(160, 155, 100, 30, BLACK);
  tft.setCursor(50, 120);
  tft.print("Plate #");
  tft.print(plateNum);
  tft.setCursor(50, 160);
  tft.print("Location: ");
  tft.print(location);
}

// TODO: use the correct pin to write with
// Screen for when the pin tool operation has finished
void redo()
{
  tft.fillScreen(BLACK);
  tft.setCursor(25, 120);
  tft.print("Run more plates?");
  tft.setCursor(80, 170);
  tft.print("CONFIRM");
  tft.drawRect(75, 165, 92, 25, WHITE);

  while (true)
  {
    digitalWrite(13, HIGH);
    TSPoint point = ts.getPoint();
    digitalWrite(13, LOW);
    pinMode(XM, OUTPUT);
    pinMode(YP, OUTPUT);
    if (point.z  >= 200 && point.z <= 1500)
    {
      int x = map(point.x, 78, 951, 0, 320);
      int y = map(point.y, 96, 921, 0, 240);
      if (x >= 106 && x <= 214 && y >= 90 && y <= 105)
        break;
    }
  }
}

/*###########################################################################################*/

void run_startup(){
  // TESTING: Begin serial connection for debugging
  Serial.begin(9600);

  // Initialize LCD
  configure_lcd();

  // Display startup screen
  // 0 -> startup screen
  greeting();

  // Set the state of any pins used as inputs
  set_pins();

  // Set the max speed and acceleration values for each motor
  configure_motors();

  // Find reference positions
  calibrate_motors();

}

void setup() {
  run_startup();

}

void loop() {
  int num_plates;
  int depth;
  bool steps[3];  
  while (true)
  {
    plateNumberSetup();
    num_plates = plateNumberInput();
    depthSetup();
    depth = depthInput();
    washStepSetup();
    washStepInput(steps);
    if (paramCheck(num_plates, depth, steps))
      break;
  }
  tft.fillScreen(BLACK);
  progressScreen(0, "Starting...");

  run_all_cycles(steps,num_plates,depth );

  redo();
}

/*FLOW OF CONTROL FOR A PIN TRANSFER*/
/**
 * 
 * 1. Set up
 * Loop:
     //// 1. LCD input from user
  * Cycle:
  *    2. take a plate from the cell plate stack
  *        - Position gripper over stack
  *            - moving gantry to stack x coordinate
  *            - moving y-axis over stack
  *            - calculate stack height of plate to grab
  *            - move gripper (z2) down to plate
  *            - grab plate
  *            - move up gripper (z2) some distance to not interfere with anything
  *            - move it to pin transfer area, move to correct x-y coordinate
  *            - lower gripper to base
  *            - ungrip
  *            - Raise gripper
  *     3. Repeat step 2 for Chemical stack
  *     4. Set up for pin transfer
  *        - move y axis over chemical plate
  *        - lower pin tool into chemical plate based on pin tool depth given by user
  *        - maybe some delay for absorption
  *        -  raise up pin tool
  *        - move y axis over cell plate
  *        - lower pin tool (z1) with chemicals into cell plate
  *        - maybe some delay for transfer
  *        - move up pin tool to end transfer to get out of the way
  * 
  *      5. Move cell plate with chemicals into output stack
  *          - position gripper over cell plate with chemicals
  *              - move gantry over plate
  *          - lower gripper (z2) to pick up plate
  *          - grip
  *          - raise plate in gripper
  *          - move x and y position to output stack
  *          - calculate height to place plate into stack
  *          - place plate into stack
  *          - raise gripper to get out of the way
  *    
  *      6. Wash pin tool
  *          - move x and y over to first cleaning solution selected by user
  *          - dip pin tool (z1) into cleaning solution .. wait sometime for cleaning
  *          - move pin tool (z1 ) up and to next cleaning solution if user selected more 
  *          - repeat previous steps for each solution selected
  * 
  *      7. Drying stage
  *          - position x and y to be over the fan/heater.
  *          - lower pin tool(z1) some distance over the fan/heater
  *          - turn on fan and heater
  *          - delay for some time for drying
  *          - turn off fan and heater
  *      8. Reset x,y,z1,z2 positions for another cycle
  *          - moving gantry over stack
  * 
  * 
  * 
  *  After all cycles
  * 
  *   Infinite loop
  *   - Ask user if want to do another set of pin transfers
  *     - if so, go back to the top of the loop to get information -> break out of infinite loop
  *     - otherwise, stay in infinite loop waiting
  * 
 */
