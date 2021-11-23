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
#define SPEED_Y      200


#define STACKED_PLATE_GRIP_HEIGHT_OFFSET 319

#define STEPS_UNTIL_BOTTOM_OF_STACK -2090



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

#define MAX_SPEED 200
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
    for (int angle = currentPos; angle >= a; angle--) {
      x.write(angle);
      delay(45);
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

// Prints the current position of each motor to the serial for testing and getting position values for hard coding.
void print_current_position(){
    Serial.println("X Position: " + String(gantry.currentPosition()) +"\nY Position: " + String(motor_y.currentPosition()) + "\nZ1 Position: " + String(motor_z1.currentPosition()) +"\nZ2 Position: " + String(motor_z2.currentPosition()) + "\n"); 
}

void setup(){

    Serial.begin(19200);

    pinMode(x_switch, INPUT);
    pinMode(y_switch, INPUT);
    pinMode(z1_switch, INPUT);
    pinMode(z2_switch, INPUT);

    // Configure servo
    servo.attach(10);

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


//   Serial.println("Press x-switch to begin");
//   while(digitalRead(x_switch) == HIGH){
//   }
//   Serial.println("Starting calibration");
//   delay(3000);
   
  //  gripper_movement_test();
  
    servo.write(OPEN);
    gripper(OPEN, servo);
    
    
    // 3 is the number of plates that we are unstacking
//    unstack(3, STACKED_PLATE_GRIP_HEIGHT_OFFSET, STEPS_UNTIL_BOTTOM_OF_STACK);
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
void gripper_movement_test(){ 
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

//void test_different_heights_featuring_justin_timberlake(){
//  int val;
//  while(true){
//    if(digitalRead(x_switch) == LOW){
//      if(val == CLOSE){
//        val = OPEN;
//      }else{
//        val = CLOSE;
//      }
//      gripper(val, servo);
//    }else if(digitalRead(y_switch) == LOW){
//      break;
//    }
//  }
//
//  long z2_start = calibrate_motor(&motor_z2, z2_switch, 1);
//  long x_start = calibrate_motor(&gantry, x_switch, -1);
//
//  // Always set the speed
//  // before you call runToNewPosition()
//  motor_z2.setSpeed(SPEED_Z2);
//  gantry.setSpeed(SPEED_GANTRY);
//
//  // baseline position(bottom)
//  gantry.runToNewPosition(x_start);
//  motor_z2.runToNewPosition(z2_start);
//  gripper(OPEN, servo);
//
//  for(int i = 0;;i+=25){
//    long test_height = z2.currentPosition() + i;
//    motor_z2.setSpeed(SPEED_Z2);
//    motor_z2.runToNewPosition(z2_start);
//    
//    
//  }
//
//  
//}

// Used for returning the absolute coordinates of a certain position
void get_absolute_positions(){ 

  int val;

  Serial.println("Press x limit switch to toggle gripper");
  Serial.println("Press the y limit switch to begin calibration");

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


  long z1_start;

//  Serial.println("Calibrating Z1");
//  long z1_start = calibrate_motor(&motor_z1, z1_switch, 1); 

  Serial.println("Calibrating Z2");
  long z2_start = calibrate_motor(&motor_z2, z2_switch, 1); 

  Serial.println("Calibrating gantry");
  long x_start = calibrate_motor(&gantry , x_switch, -1);

  Serial.println("Calibrating Y");
  long y_start = calibrate_motor(&motor_y, y_switch, -1);
 
//  motor_z1.setSpeed(SPEED_Z);
  motor_z2.setSpeed(SPEED_Z);
  gantry.setSpeed(SPEED_GANTRY);
  motor_y.setSpeed(SPEED_Y);
  
  // Print the coordinate from where the motor started
  Serial.println("X: " + String(x_start) + "\nY: " + String(y_start) + "\nZ1: " + String(z1_start) + "\nZ2: " + String(z2_start));

  gantry.runToNewPosition(x_start);
  motor_y.runToNewPosition(y_start);
//  motor_z1.runToNewPosition(z1_start);
  motor_z2.runToNewPosition(z2_start);
    
  
}


void test_different_heights() {
  int val;
  Serial.println("Here");
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

// Used for returning the absolute coordinates of a certain position
void get_absolute_positions_loop(){ 

  String area [10] = {"A", "B", "C", "D", "E", "F", "G", "H", "I", "J"};
  int i = 0;
  while (true){

    int val;

    Serial.println("Press x limit switch to toggle gripper");
    Serial.println("Press the y limit switch to begin calibration");

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

    delay(3000);

    long z1_start;
//    long z2_start;
//    long x_start;
//    long y_start;

//    Serial.println("Calibrating Z1");
//    long z1_start = calibrate_motor(&motor_z1, z1_switch, 1); 

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

    Serial.println("Done calibrating all motors");
    Serial.println("\nResults of " + String(area[i++]));
    
    // Print the coordinate from where the motor started
    Serial.println("Pre-calibration starting position: ");
    Serial.println("X: " + String(x_start) + "\nY: " + String(y_start) + "\nZ1: " + String(z1_start) + "\nZ2: " + String(z2_start) +"\n");

    Serial.println("Running to starting position");
    gantry.runToNewPosition(x_start);
    motor_y.runToNewPosition(y_start);
    motor_z1.runToNewPosition(z1_start);
    motor_z2.runToNewPosition(z2_start);

    // Print the current coordinates of the motor
    Serial.println("Current Positions:");
    Serial.println("X: " + String(gantry.currentPosition()) + "\nY: " + String(motor_y.currentPosition()) + "\nZ1: " + String(motor_z1.currentPosition()) + "\nZ2: " + String(motor_z2.currentPosition()));

    Serial.println("Press x limit switch to find another value\nPress y limit switch to end.");
    delay(3000);
    while (true){
        if(digitalRead(x_switch) == LOW){
            Serial.println("Turn off power supply to motor drivers to re-position shit.");
             break;
        }else if(digitalRead(y_switch) == LOW){
            Serial.println("Done getting values.");
            return;
        }
    }
  }
}

void unstack(int num_plates, int offset,int steps_until_last_plate){

    int grab_height = steps_until_last_plate + (offset * (num_plates - 1));

    long z2_start = calibrate_motor(&motor_z2, z2_switch, 1); 
    long x_start = calibrate_motor(&gantry , x_switch, -1);


    gantry.setSpeed(SPEED_GANTRY);
    gantry.runToNewPosition(x_start);

    for(int i = 0 ; i < num_plates; i++){

        motor_z2.setSpeed(SPEED_Z);
        motor_z2.runToNewPosition(grab_height);
        gripper(CLOSE, servo);

        motor_z2.setSpeed(SPEED_Z2);
        motor_z2.move(500);
        motor_z2.runToPosition();

        gantry.setSpeed(SPEED_GANTRY);
        gantry.runToNewPosition(gantry.currentPosition() + 500);
        gripper(OPEN, servo);

        gantry.setSpeed(SPEED_GANTRY);
        gantry.runToNewPosition(x_start);

        grab_height -= offset;
    }
}


void stacking_test(int num_plates, int offset,int steps_until_last_plate){
    int grab_height = steps_until_last_plate + (offset * (num_plates - 1));
    int stack_height = steps_until_last_plate;

    int val;
    Serial.println("Press x limit switch to toggle gripper");
    Serial.println("Press the y limit switch to begin calibration");

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

    delay(3000);

    long z2_start = calibrate_motor(&motor_z2, z2_switch, 1); 
    long z1_start = calibrate_motor(&motor_z1, z1_switch, 1); 
    long x_start = calibrate_motor(&gantry , x_switch, -1);
    long y_start = calibrate_motor(&motor_y, y_switch ,-1);


    for(int i = 0 ; i < num_plates; i++){

        move_to_coordinate_x_first(1463, 3358, -400, grab_height);
        gripper(CLOSE, servo);
        move_to_coordinate_z_first(1463, 3358, -400, -400);
        
        move_to_coordinate_x_first(2063,3358,-400, stack_height);
        gripper(OPEN, servo);
        move_to_coordinate_z_first(1463, 3358, -400, -400);

        grab_height -= offset;
        stack_height += offset;
    }
}

void move_y(){
  motor_y.setSpeed(SPEED_Y);
  while(true){
    motor_y.runSpeed();
  }
}


void test_limit_switches(){
   Serial.println("X: " + String(digitalRead(x_switch)));
   Serial.println("Y: " + String(digitalRead(y_switch)));
   Serial.println("Z1: " + String(digitalRead(z1_switch)));
   Serial.println("Z2: " + String(digitalRead(z2_switch)));
   Serial.println();
   delay(1000);
}



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
void position_test(){
    int val;

    Serial.println("Press x limit switch to toggle gripper");
    Serial.println("Press the y limit switch to begin calibration");

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

    delay(3000);

    long z1_start;
//    long z2_start;
//    long x_start;
//    long y_start;

//    Serial.println("Calibrating Z1");
//    long z1_start = calibrate_motor(&motor_z1, z1_switch, 1); 

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

    Serial.println("Done calibrating all motors");

    /*
     * Cell transfer -> input cell stack
     * 
    // Move to cell transfer area
    move_to_coordinate_x_first(1463,3358,-400,-1892);
    gripper(CLOSE, servo);
    move_to_coordinate_z_first(1463,3358,-400, -1000);

    // Move to input stack
    move_to_coordinate_x_first(2063,3423, -400, -1772);
    gripper(OPEN, servo);
    move_to_coordinate_z_first(2063,3423, -400, -1000);

    */

    // Move to cell transfer area
    move_to_coordinate_x_first(1467,6051,-400,-1892);
    gripper(CLOSE, servo);
    move_to_coordinate_z_first(1467,6051,-400, -1000);

    // Move to input stack
    move_to_coordinate_x_first(2063,6091, -400, -1892);
    gripper(OPEN, servo);
    move_to_coordinate_z_first(2063,6091, -400, -1000);
    
}
void loop(){

//  test_limit_switches();
// get_absolute_positions();
// move_y();
//position_test();

// -1881 base 
stacking_test(3, 319, -1881);


}
  
