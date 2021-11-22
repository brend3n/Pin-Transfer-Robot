// Motors
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>
#include <EEPROM.h>

// Motor driver type
#define interface  1

// Direction and Step pins for all motors
#define dirPiny   40
#define stepPiny  41

#define dirPinz1  42
#define stepPinz1 43

#define dirPinz2  44
#define stepPinz2 45

#define dirPinx   46
#define stepPinx  47 

#define CLOSE 50
#define OPEN  0

#define UP 500
#define DOWN -500

#define y_switch 26
#define x_switch 25
#define z1_switch 29
#define z2_switch 24

// Joy-stick controller
#define x_dir     A0
#define y_dir     A1
#define switch_s  52
#define buttonPin 22


#define SPEED_Z1     100
#define SPEED_Z2     100
#define SPEED_Z      100
#define SPEED_GANTRY -50
#define SPEED_Y      100
#define NUM_TESTS 10
#define OPTIMAL_BASELINE_Z -2090

// Switch variables
int switch_state;
int switch_count = 1;

// Button variables
int button_state;
bool button_mode = false;

// Joy-stick variables
int x_pos;
int y_pos;
int mapX;
int mapY;

#define MAX_SPEED 100
#define MAX_ACCELERATION 100

// Instantiating motor driver objects
AccelStepper gantry = AccelStepper(interface, stepPinx, dirPinx);
AccelStepper motor_y = AccelStepper(interface, stepPiny, dirPiny);
AccelStepper motor_z1 = AccelStepper(interface, stepPinz1, dirPinz1);
AccelStepper motor_z2 = AccelStepper(interface, stepPinz2, dirPinz2);
AccelStepper motor_z = AccelStepper(interface, stepPinz2, dirPinz2);
Servo servo = Servo();

void gripper(int a, Servo x)
{
  int currentPos =  x.read();
  if (currentPos > a)
  {
    // Open
    for (int angle = currentPos; angle >= a; angle--) {
      x.write(angle);
      delay(30);
    }
  }
  else if (currentPos < a)
  {
    // Close
    for (int angle = currentPos; angle <= a; angle++) {
      x.write(angle);
      delay(15);
    }
  }
}

// Prints the current position of each motor to the serial for testing and getting position values for hard coding.
void print_current_position(){
    Serial.println("X Position: " + String(gantry.currentPosition()) +"\nY Position: " + String(motor_y.currentPosition()) + "\nZ1 Position: " + String(motor_z1.currentPosition()) +"\nZ2 Position: " + String(motor_z2.currentPosition()) + "\n"); 
}

// void get_average_calibration_values() {
//   long avg_steps[4] = {0, 0, 0, 0};

//   for (int i = 0; i < NUM_TESTS; i++) {
//     long *steps = get_absolute_positions();

//     for (int i = 0; i < 4; i++) {
//       avg_steps[i] += steps[i];
//     }
//   }
  
//   Serial.println("Average values(for z1, z2, x1, and x2, respectively)");
//   for (int i = 0; i < NUM_TESTS; i++) {
//     Serial.println(double(avg_steps[i]) / NUM_TESTS);
//   }
// }

void test_optimal_baseline_height() {
  test_baseline_height(OPTIMAL_BASELINE_Z);
}

void test_baseline_height(long baseline) {
  int val;
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

  long z2_start = calibrate_motor(&motor_z2, z2_switch, 1);
  long x_start = calibrate_motor(&gantry, x_switch, -1);

  // Always set the speed
  // before you call runToNewPosition()
  motor_z2.setSpeed(SPEED_Z2);
  gantry.setSpeed(SPEED_GANTRY);

  // baseline position(bottom)
  gantry.runToNewPosition(x_start);
  motor_z2.runToNewPosition(z2_start);

  for (;;) {
    // go up by some distance
    motor_z2.setSpeed(SPEED_Z2);
    // This is an optimum value
    // that was found through
    // extensive testing
    motor_z2.moveTo(baseline);
    motor_z2.runToPosition();
    Serial.print("motor_z2 position: "); Serial.println(motor_z2.currentPosition());
    delay(1000);

    // grab
    gripper(CLOSE, servo);
    delay(2000);

    // go up by some more from when you grabbed
    motor_z2.setSpeed(SPEED_Z2);
    motor_z2.move(2000);
    motor_z2.runToPosition();
    delay(1000);

    // go back
    gantry.setSpeed(SPEED_GANTRY);
    gantry.move(400);
    gantry.runToPosition();
    delay(1000);

    // then forth
    gantry.setSpeed(SPEED_GANTRY);
    gantry.move(-400);
    gantry.runToPosition();
    delay(1000);

    // then go down and drop it
    motor_z2.setSpeed(SPEED_Z2);
    motor_z2.move(-2000);
    motor_z2.runToPosition();
    delay(1000);

    motor_z2.setSpeed(SPEED_Z2);
    motor_z2.moveTo(baseline);
    motor_z2.runToPosition();
    delay(1000);

    gripper(OPEN, servo);
    delay(2000);
  }
}

// void test_baseline_height() {
//   int val;
//   while(true){
//     if(digitalRead(x_switch) == LOW){
//       if(val == CLOSE){
//         val = OPEN;
//       }else{
//         val = CLOSE;
//       }
//       gripper(val, servo);
//     }else if(digitalRead(y_switch) == LOW){
//       break;
//     }
//   }

//   long z2_start = calibrate_motor(&motor_z2, z2_switch, 1);
//   long x_start = calibrate_motor(&gantry, x_switch, -1);

//   // Always set the speed
//   // before you call runToNewPosition()
//   motor_z2.setSpeed(SPEED_Z2);
//   gantry.setSpeed(SPEED_GANTRY);

//   // baseline position(bottom)
//   gantry.runToNewPosition(x_start);
//   motor_z2.runToNewPosition(z2_start);

//   for (int i = 0;;i += 0) {
//     int initital_position = motor_z2.currentPosition();
//     // go up by some distance
//     motor_z2.setSpeed(SPEED_Z2);
//     // This is an optimum value
//     // that was found through
//     // extensive testing
//     motor_z2.moveTo(-2090);
//     motor_z2.runToPosition();
//     Serial.print("motor_z2 position: "); Serial.println(motor_z2.currentPosition());
//     delay(1000);

//     // grab
//     gripper(CLOSE, servo);
//     delay(2000);

//     // go up by some more from when you grabbed
//     motor_z2.setSpeed(SPEED_Z2);
//     motor_z2.move(2000);
//     motor_z2.runToPosition();
//     delay(1000);

//     // go back
//     gantry.setSpeed(SPEED_GANTRY);
//     gantry.move(400);
//     gantry.runToPosition();
//     delay(1000);

//     // then forth
//     gantry.setSpeed(SPEED_GANTRY);
//     gantry.move(-400);
//     gantry.runToPosition();
//     delay(1000);

//     // then go down and drop it

//     // motor_z2.setSpeed(SPEED_Z2);
//     // motor_z2.move(-2000);
//     // motor_z2.runToPosition();
//     // delay(1000);

//     motor_z2.setSpeed(SPEED_Z2);
//     motor_z2.moveTo(initital_position);
//     motor_z2.runToPosition();
//     delay(1000);

//     gripper(OPEN, servo);
//     delay(2000);
//   }
// }

void test_different_heights() {
  int val;
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

  long z2_start = calibrate_motor(&motor_z2, z2_switch, 1);
  long x_start = calibrate_motor(&gantry, x_switch, -1);

  // Always set the speed
  // before you call runToNewPosition()
  motor_z2.setSpeed(SPEED_Z2);
  gantry.setSpeed(SPEED_GANTRY);

  // baseline position(bottom)
  gantry.runToNewPosition(x_start);
  motor_z2.runToNewPosition(z2_start);
  gripper(OPEN, servo);

  for (int i = 0;; i += 25) {
    for (int j = 0; j < 3; j++) {
      // go up by some distance
      motor_z2.setSpeed(SPEED_Z2);
      motor_z2.move(i);
      motor_z2.runToPosition();
      Serial.print("motor_z2 position: "); Serial.println(motor_z2.currentPosition());
      delay(1000);

      // grab
      gripper(CLOSE, servo);
      delay(2000);

      // go up by some more from when you grabbed
      motor_z2.setSpeed(SPEED_Z2);
      motor_z2.move(1000);
      motor_z2.runToPosition();
      delay(1000);

      // go back
      gantry.setSpeed(SPEED_GANTRY);
      gantry.move(400);
      gantry.runToPosition();
      delay(1000);

      // then forth
      gantry.setSpeed(SPEED_GANTRY);
      gantry.move(-400);
      gantry.runToPosition();
      delay(1000);

      // then go down and drop it
      motor_z2.setSpeed(SPEED_Z2);
      motor_z2.move(-1000);
      motor_z2.runToPosition();
      delay(1000);

      motor_z2.setSpeed(SPEED_Z2);
      motor_z2.move(-i);
      motor_z2.runToPosition();
      delay(1000);

      gripper(OPEN, servo);
      delay(2000);
    }
  }
}

void setup(){

  Serial.begin(19200);

  pinMode(x_switch, INPUT);
  pinMode(y_switch, INPUT);
  pinMode(z1_switch, INPUT);
  pinMode(z2_switch, INPUT);

  // Configure servo
  servo.attach(9);
  servo.write(OPEN);

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


  // get_average_calibration_values();

  // Then use this other piece of code
  // to test different heights
  // test_different_heights();

  test_optimal_baseline_height();


  // // Test 1
  // Serial.println("Calibrating Z1");
  // long z1_start = calibrate_motor(&motor_z1, z1_switch, 1);

  // Serial.println("Calibrating Z2");
  // long z2_start = calibrate_motor(&motor_z2, z2_switch, 1); 

  // AccelStepper *motors[2] = {&gantry, &motor_y};

  // int limit_switches[2] = {x_switch, y_switch};
  // short dirs[2] = {-1, -1};
  
  // calibrate_motors_async(motors, limit_switches, dirs, 2);

  // Test 2

  // AccelStepper *motors[4] = {&gantry, &motor_y, &motor_z1, &motor_z2};

  // int limit_switches[4] = {x_switch, y_switch, z1_switch, z2_switch};

  // short dirs[4] =  {-1, -1, 1, 1};
  // calibrate_motors_async(motors, limit_switches, dirs, 4);

  // Test 3
  
  // AccelStepper *motors[1] = {&motor_y};
  // int limit_switches[1] = {y_switch};
  // short dirs[1]  = {-1};
  // calibrate_motors_async(motors, limit_switches, dirs, 1);

  // Serial.println("Calibrating Z1");
  // long z1_start = calibrate_motor(&motor_z1, z1_switch, 1); 

  // Serial.println("Calibrating Z2");
  // long z2_start = calibrate_motor(&motor_z2, z2_switch, 1); 

  // Serial.println("Calibrating gantry");
  // long x_start = calibrate_motor(&gantry, x_switch, -1);

  // Serial.println("Calibrating Y");
  // long y_start = calibrate_motor(&motor_y, y_switch, -1);
}

// Used for returning the absolute coordinates of a certain position
long *get_absolute_positions(){ 
  long steps[4];

  Serial.println("Press x limit switch to toggle gripper");
  Serial.println("Press the y limit switch to begin calibration");

  gripper(OPEN, servo);
  int val;
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

  steps[0] = z1_start;
  steps[1] = z2_start;
  steps[2] = x_start;
  steps[3] = y_start;

  gripper(OPEN, servo);

  return steps;
}

// Update the states from all of the inputs
void get_states(){
    x_pos = analogRead(x_dir);  
    y_pos = analogRead(y_dir);
    switch_state = digitalRead(switch_s);
    button_state = digitalRead(buttonPin);

    mapX = map(x_pos, 363,1023,-512,512);
    mapY = map(y_pos, 0,734,-512,512);

    Serial.println("switch_state: " + String(switch_state));
    Serial.println("x_pos: " + String(mapX));
    Serial.println("y_pos: " + String(mapY));
    Serial.println(""); 
}

void control_motor(){
    get_states();

    // Toggle button mode each time it is clicked
    if (button_state == HIGH){
        // Debounce a bit
        delay(200);
        button_mode = !button_mode;
        Serial.println("Button toggled");
    }
    // Joy-stick is RIGHT
    if (mapX > 400){
        // Serial.println("RIGHT");
        gantry.setSpeed(SPEED_GANTRY);
        while (mapX > 400){         
            gantry.runSpeed();           
            get_states();
        }

    // Joy-stick is LEFT
    }else if (mapX < -400){
        // Serial.println("LEFT");
        gantry.setSpeed(-SPEED_GANTRY);
        while (mapX < -400){
            gantry.runSpeed();            
            get_states();
        }
    // Joy-stick is DOWN
    }else if(mapY > 400){

        if (!button_mode){
            //  Z-axis
            //  Serial.println("DOWN");
            motor_z.setSpeed(SPEED_Z);
            while (mapY > 400){
                motor_z.runSpeed();
                get_states();
          }
        }else{
        //  Y-axis
        //  Serial.println("Y LEFT");
          motor_y.setSpeed(-SPEED_Y);
          while (mapY > 400){
              motor_y.runSpeed();
              get_states();
          }
        }
    }
    // Joy-stick is UP
    else if(mapY < -400){
        if (!button_mode){
          // Z-axis
          motor_z.setSpeed(SPEED_Z);
          while (mapY < -400){
                motor_z.runSpeed();
                get_states();
          }
        }else{
          //Y-axis
        //   Serial.println("Y RIGHT");
          motor_y.setSpeed(SPEED_Y);
          while (mapY < -400){
                motor_y.runSpeed();
                get_states();
          }
        }
    }else{
        motor_z.stop();
        gantry.stop();
        motor_y.stop();
    }

    // Could change this to a boolean flag
    if (switch_state == LOW){
        switch_count++;
    }

    if((switch_count % 2) == 0){
        gripper(CLOSE, servo);
    }else{
        gripper(OPEN, servo);
    }
}

/*Test the x-axis motors*/
void test(){

  gantry.moveTo(100);
  gantry.setSpeed(100);

  while (gantry.distanceToGo() != 0){
    gantry.runSpeed();
  }

  gantry.moveTo(-100);
  gantry.setSpeed(-100);

  while (gantry.distanceToGo() != 0){
    gantry.runSpeed();
  }  
}

void calibrate_z(){
  motor_z.setSpeed(100);

  while(true){
    if(digitalRead(z1_switch) == LOW){
      motor_z.stop();
      motor_z.move(-50);
      motor_z.runToPosition();
      break;
    }
    motor_z.runSpeed();
  }
  motor_z.setCurrentPosition(0);
}

bool check_limit_switches(int *limit_switches, int num_motors) {
  // The initial value here should be
  // an idempotent value so as not to
  // affect the final result.
  bool result = true;

  // All limit_switches need to read low for runSpeed() to be called
  for (int i = 0; i < num_motors; i++) {
    result = result && (digitalRead(limit_switches[i]) != LOW);
  }

  return result;
}

bool check_distance_to_go(AccelStepper **motors, int num_motors) {
  bool result = false;

  // Checks if at least one of the motors needs a runSpeed() call
  for (int i = 0; i < num_motors; i++) {
    result = result || (motors[i]->distanceToGo() != 0);
  }

  return result;
}

// General algorithm for simultaneous calibration of motors
// Have yet to upload and try this out.
long *calibrate_motors_async(AccelStepper **motors, int *limit_switches, short *dirs, int num_motors) {
  long steps[num_motors];
  bool to_move_back[num_motors];
  bool calibrated[num_motors];
  int num_calibrated = 0;
  
  for (int i = 0; i < num_motors; i++) {
    to_move_back[i] = false;
    calibrated[i] = false;
  }

  Serial.println("Starting Position: ");

  // For testing
  print_current_position();

  // Fixed a bug here. Super important!!!!
  for (int i = 0; i < num_motors; i++) {
    // Adjusted speed here for safer testing.
    // The original speed was 100
    motors[i]->setSpeed(20 * dirs[i]);
  }

  while (num_calibrated < num_motors)  {
    while (check_limit_switches(limit_switches, num_motors)) {
      for (int i = 0; i < num_motors; i++) {
        if (!calibrated[i])
          motors[i]->runSpeed();
      }
    }

    // Some limit switch(es) got hit
    for (int i = 0; i < num_motors; i++) {
      // Test unconditional stopping method
      // refactor back when done

      // On second thought, it should stay here
      // because there's no way to concurrently
      // check for other motors hitting limit switches
      // simultaneously
      motors[i]->stop();
      if (digitalRead(limit_switches[i]) == LOW) {
        to_move_back[i] = true;
        calibrated[i] = true;

        steps[i] = -motors[i]->currentPosition();
        // motors[i]->stop();

        // For testing
        // Serial.println("Steps taken to reach limit switch: " + String(steps[i]));

        motors[i]->setCurrentPosition(0);
        num_calibrated++;

        // For testing
        // Serial.println("End position:");
        // print_current_position();
      }
    }

    for (int i = 0; i < num_motors; i++) {
      if (to_move_back[i]) {
        // Should be 50
        // refactor back when done
        motors[i]->move(100 * -dirs[i]);
      }
    }

    // Check which motors need to move back.
    while (check_distance_to_go(motors, num_motors)) {
      for (int i = 0; i < num_motors; i++) {
        if (to_move_back[i] && motors[i]->distanceToGo() != 0) {
          motors[i]->runSpeed();
        }
      }
    }

    // Reset toMoveBack
    for (int i = 0; i < num_motors; i++) {
      to_move_back[i] = false;
    }
  }

  Serial.println("Async calibration complete!");
  return steps;
}

// Calibrates a single motor given by &motor and a limit switch
long calibrate_motor(AccelStepper *motor, int limit_switch, short dir) {

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

void loop() {}