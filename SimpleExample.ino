// Motors
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>
#include <EEPROM.h>

// Motor driver type
#define interface  1

// Direction and Step pins for all motors
#define dirPiny  40
#define stepPiny 41

#define dirPinz  42
#define stepPinz 43

#define dirPinx1 46
#define stepPinx1 47 

#define CLOSE 50
#define OPEN  0

#define UP 500
#define DOWN -500

#define y_switch 27
#define x_switch 29
#define z_switch 26

#define gantry_speed 100
#define y_speed 100
#define z_speed 100


// Joy-stick controller
#define x_dir A0
#define y_dir A1
#define switch_s 52
#define buttonPin 22


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
AccelStepper gantry = AccelStepper(interface, stepPinx1, dirPinx1);
AccelStepper motor_y = AccelStepper(interface, stepPiny, dirPiny);
AccelStepper motor_z = AccelStepper(interface, stepPinz, dirPinz);
Servo servo = Servo();

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

// Prints the current position of each motor to the serial for testing and getting position values for hard coding.
void print_current_position(){
    Serial.println("X Position: " + String(gantry.currentPosition()) +"\nY Position: " + String(motor_y.currentPosition()) + "\nZ Position: " + String(motor_z.currentPosition()) + "\n"); 
}

void setup(){

    Serial.begin(19200);

    // Setting pins
    pinMode(x_dir, INPUT);
    pinMode(y_dir, INPUT);
    pinMode(buttonPin, INPUT);
    pinMode(switch_s, INPUT_PULLUP);

    pinMode(x_switch, INPUT);
    pinMode(y_switch, INPUT);
    pinMode(z_switch, INPUT);

    // Configure servo
    servo.attach(9);

    // Set maxmium speeds
    gantry.setMaxSpeed(MAX_SPEED);
    motor_y.setMaxSpeed(MAX_SPEED);
    motor_z.setMaxSpeed(MAX_SPEED);

    // Set acceleration
    gantry.setAcceleration(MAX_ACCELERATION);
    motor_y.setAcceleration(MAX_ACCELERATION);
    motor_z.setAcceleration(MAX_ACCELERATION);


    while(digitalRead(x_switch) == HIGH){
    }
    delay(3000);
    gripper_movement_test();
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
        gantry.setSpeed(gantry_speed);
        while (mapX > 400){         
            gantry.runSpeed();           
            get_states();
        }

    // Joy-stick is LEFT
    }else if (mapX < -400){
        // Serial.println("LEFT");
        gantry.setSpeed(-gantry_speed);
        while (mapX < -400){
            gantry.runSpeed();            
            get_states();
        }
    // Joy-stick is DOWN
    }else if(mapY > 400){

        if (!button_mode){
            //  Z-axis
            //  Serial.println("DOWN");
            motor_z.setSpeed(z_speed);
            while (mapY > 400){
                motor_z.runSpeed();
                get_states();
          }
        }else{
        //  Y-axis
        //  Serial.println("Y LEFT");
          motor_y.setSpeed(-y_speed);
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
          motor_z.setSpeed(z_speed);
          while (mapY < -400){
                motor_z.runSpeed();
                get_states();
          }
        }else{
          //Y-axis
        //   Serial.println("Y RIGHT");
          motor_y.setSpeed(y_speed);
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
    if(digitalRead(z_switch) == LOW){
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
long calibrate_motor(AccelStepper *motor, int limit_switch){

    long steps = 0;
    Serial.println("Start Position:");
    print_current_position();
    
    // Set the current speed and run until the limit switch is hit
    motor->setSpeed(100);
    while (digitalRead(limit_switch) != LOW){
        motor->runSpeed();
    }

    // Distance from starting position to limit switch
    // Negative because reference 0 is at limit switch and all of other distances are negative relative to the limit switch
    steps = -1*motor->currentPosition();
    Serial.println("Steps taken to reach limit switch: " + String(steps));
    // Stop the motor and store the current position
    motor->stop();
    motor->setCurrentPosition(0);

    Serial.println("End Position:");
    print_current_position();

    // Move the motor off the limit switch
    motor->move(-50);
    motor->runToPosition();

    return steps;
}

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

  Serial.println("Calibrating Z");
  long z_start = calibrate_motor(&motor_z, z_switch); 
  Serial.println("Calibrating gantry");
  long x_start = calibrate_motor(&gantry , x_switch);
  Serial.println("Calibrating Y");
  long y_start = calibrate_motor(&motor_y, y_switch);
 
  motor_z.setSpeed(100);
  gantry.setSpeed(100);
  motor_y.setSpeed(100);
  

  gantry.runToNewPosition(x_start);
  motor_y.runToNewPosition(y_start);
  motor_z.runToNewPosition(z_start);
}

void loop(){

//  Serial.println("X: " + String(digitalRead(x_switch)));
//  Serial.println("Y: " + String(digitalRead(y_switch)));
//  Serial.println("Z: " + String(digitalRead(z_switch)));
//  delay(1000);
}
  