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

#define SERVO_PIN 9

// Servo constants
#define CLOSE 50
#define OPEN  0

/*###########################################################################################*/
/* LCD Pin Definitions*/
#define YP A3  
#define XM A2  
#define YM 9   
#define XP 8 
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET A4
Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
#define BLACK 0x0000
#define WHITE 0xFFFF

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
#define SPEED_Y 100
#define SPEED_Z 100

#define SPEED_Z1     100
#define SPEED_Z2     100

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



// ! Probaby not even used so get rid of this shit ... maybe
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

// void gripper_movement_test(){ 
void calibrate_motors(){ 
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
// TODO
// Take plate from a stack
void take_from_stack(int stack, int height_to_pick_from){

}

// TODO
// Put a plate onto a stack
void push_onto_stack(int stack, int height_to_put_on){

}

// TODO
void do_wash(short wash_step){


  // Solution 1
  if(wash_step == 1){
    // Move to Solution 1
    dip_pin_tool();
  }
  // Solution 2
  else if(wash_step == 2){
    // Move to Solution 2
    dip_pin_tool();
  }
  // Solution 3
  else if(wash_step == 3){
    // Move to Solution 3
    dip_pin_tool();
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
void wash_pin_tool(boolean [] wash_steps){
  // 1. Move pin tool to wash step linear actuator
  for(int wash_step = 0; wash_step < 3; wash_step++){
    if(wash_steps[wash_step]){
      do_wash(wash_step);
    }
  }

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
void do_cycle(boolean [] wash_steps, int pin_depth, int drying_time, int height_of_next_plate_in_steps){
   take_from_stack();
   do_pin_transfer();
   wash_pin_tool(wash_steps);
   dry_pin_tool();
   push_onto_stack();
}

// TODO
// TEST
//void run_all_cycles(short num_plates, short num_wash_steps, int pin_depth, int drying_time){/
void run_all_cycles(boolean [] wash_steps, short num_plates, int pin_depth ){

   
  double height_of_stack = (num_plates * plate_height_in_mm);
  double height_of_next_plate_to_grab = convert_mm_to_steps(height_of_stack);

  for(int i = 0; i < num_plates; i++){
    
    do_cycle(wash_steps, pin_depth, drying_time, height_of_next_plate_to_grab);

    // Update where the next plate is located at.
    height_of_next_plate_to_grab -= plate_height_in_steps;
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
  configure_LCD();

  // Display startup screen
  // 0 -> startup screen
  greeting();

  // Set the state of any pins used as inputs
  set_pins();

  // Set the max speed and acceleration values for each motor
  configure_motors();

  
  // Add stepper motor objects to MultiStepper object
  add_all_steppers_to_manager();

  // Find reference positions
  calibrate_motors();

}

void setup() {
  run_startup();

}

void loop() {
  while (true)
  {
    plateNumberSetup();
    int num_plates = plateNumberInput();
    depthSetup();
    int depth = depthInput();
    washStepSetup();
    bool steps[3];
    washStepInput(steps);
    if (paramCheck(num_plates, depth, steps))
      break;
  }
  tft.fillScreen(BLACK);
  progressScreen(0, "Starting...");

  // run_all_cycles();

  redo();
}






/**
 * 
 * 1. Set up
 * Loop:
     1. LCD input from user
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
  *
  * 
  * 
 *           
 *          
 *    
 * 
 * 
 * 
 * 
 * 
 * 
 */