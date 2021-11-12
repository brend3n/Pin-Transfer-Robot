// Motors
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>

// Motor driver type
#define interface  1

// Direction and Step pins for all motors
#define dirPinx1  45
#define stepPinx1 44

#define dirPinx2  33
#define stepPinx2 32

#define dirPinz  48
#define stepPinz 49

#define dirPiny  -1
#define stepPiny -1

#define CLOSE 50
#define OPEN  0

// if one stepper for gantry
// #define dirPinGantry -1
// #define stepPinGantry -1


#define x_dir A0
#define y_dir A1
#define switch_s 13
#define buttonPin 12 

// Switch variables
bool isOpen = false;
int prev_state = 1;
int switch_state;

// Button variables
int button_state;
int button_count = 1;


int x_pos;
int y_pos;
int mapX;
int mapY;

long pos[2];

bool gantryIsSet = false;

#define MAX_SPEED 100
#define MAX_ACCELERATION 100

// Instantiating motor driver objects
AccelStepper motor_x1 = AccelStepper(interface, stepPinx1, dirPinx1);
AccelStepper motor_x2 = AccelStepper(interface, stepPinx2, dirPinx2);
AccelStepper motor_y = AccelStepper(interface, stepPiny, dirPiny);
AccelStepper motor_z = AccelStepper(interface, stepPinz, dirPinz);
Servo servo = Servo();

// if one stepper for gantry
// AccelStepper gantry = AccelStepper(interface, stepPinGantry, dirPinGantry);

AccelStepper* current_motor;


MultiStepper gantry;


#define steps 100

// ! Used for testing only
// WORKS
void test_run_group(){
    // Set all positions for the motors to move in

    // positions: {X1, X2, X3, Y, Z}
    //             0   1   2   3  4

    long positions[2];

    positions[0] = steps;
    positions[1] = (-1*steps);

    // Set target positions
    gantry.moveTo(positions);

    // Run motors to target positions
    gantry.runSpeedToPosition();

    // Small delay
    delay(1010);

    // Same as before but other direction
    positions[0] = (-1*steps);
    positions[1] = steps;

    gantry.moveTo(positions);
    gantry.runSpeedToPosition();
    delay(1010);
}

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
#define UP 500
#define DOWN -500
void test(){
  motor_z.move(UP);
  motor_z.runToPosition();
  // gripper(CLOSE, servo);

  // motor_z.move(150);
  // motor_z.runToPosition();
  // gripper(CLOSE, servo);

}
void setup(){

    Serial.begin(9600);

    pinMode(x_dir, INPUT);
    pinMode(y_dir, INPUT);
    pinMode(switch_s, INPUT_PULLUP);
    pinMode(buttonPin, INPUT);

    servo.attach(9);

    // Set maxmium speeds
    motor_x1.setMaxSpeed(MAX_SPEED);
    motor_x2.setMaxSpeed(MAX_SPEED);
    motor_y.setMaxSpeed(MAX_SPEED);
    motor_z.setMaxSpeed(MAX_SPEED);

    // Set acceleration
    motor_x1.setAcceleration(MAX_ACCELERATION);
    motor_x2.setAcceleration(MAX_ACCELERATION);
    motor_y.setAcceleration(MAX_ACCELERATION);
    motor_z.setAcceleration(MAX_ACCELERATION);

    // Add motors to multistepper object
    gantry.addStepper(motor_x1);
    gantry.addStepper(motor_x2);

    servo.write(0);
    // gripper(CLOSE, servo);
    // current_motor = &motor_y;
    // delay(3000);
    // test();
}


void control_motor(){
    x_pos = analogRead(x_dir);
    y_pos = analogRead(y_dir);
    switch_state = digitalRead(switch_s);
    button_state = digitalRead(buttonPin);

    mapX = map(x_pos, 0,1024,-512,512);
    mapY = map(y_pos, 0,1024,-512,512);
    
    if (mapX > 400){
        Serial.println("RIGHT");

        // motor_x1.move(10);
        // motor_x2.move(-10);
        // motor_x1.runSpeedToPosition();
        // motor_x2.runSpeedToPosition();

    

    }else if (mapX < -400){
        Serial.println("LEFT");
        
        // motor_x1.move(-10);
        // motor_x2.move(10);
        // motor_x1.runSpeedToPosition();
        // motor_x2.runSpeedToPosition();
    }else if(mapY > 400){
        Serial.println("DOWN");
        motor_z.move(DOWN);
        motor_z.runToPosition();
    }else if(mapY < -400){
        Serial.println("UP");
        motor_z.move(UP);
        motor_z.runToPosition();
    }else{
        motor_z.stop();
        motor_x1.stop();
        motor_x2.stop();
    }

    if (switch_state == LOW){
        button_count++;
    }

    if((button_count % 2) == 0){
        gripper(CLOSE, servo);
    }else{
        gripper(OPEN, servo);
    }



    // mapX = x_pos;
    // mapY = y_pos;
    

    // Determine current motor to operate
    // if (state != prev_state){
        // if(state == 1){
        //     Serial.println("Controlling: Gantry ");
        //       gantryIsSet = true;
        //     // current_motor = &gantry;
        // }else if(state == 2){
        //     Serial.println("Controlling: Y ");
        //     current_motor = &motor_y;
        //     gantryIsSet =false;
        // }else if(state == 3){
        //     Serial.println("Controlling: Z ");
        //     current_motor = &motor_z;
        //     gantryIsSet = false;
        // }else{
        //     gantryIsSet = false;
        // }
    // }
    /*
    // Determine current motor to operate
    if (state != prev_state){
        if(state == 1){
            Serial.println("Controlling: Gantry ");
            gantryIsSet = true;
            // current_motor = &gantry;
        }else if(state == 2){
            Serial.println("Controlling: Y ");
            current_motor = &motor_y;
            gantryIsSet =false;
        }else if(state == 3){
            Serial.println("Controlling: Z ");
            current_motor = &motor_z;
            gantryIsSet = false;
        }else{
            gantryIsSet = false;
        }
    }

    if (button_state == HIGH){
        button_count++;
    }

    if((button_count % 2) == 0){
        gripper(CLOSE, servo);
    }else{
        gripper(OPEN, servo);
    }
    
    // Move the motor based on joystick position
    if((mapX < 0 && mapY < 0) || (mapX < 0 && mapY > 0)){
        Serial.println("LEFT or DOWN");
        if (gantryIsSet){
            
            pos[0] =  10;
            pos[1] = -10;
            gantry.moveTo(pos);
            gantry.runSpeedToPosition();
        }else{
            (*current_motor).move(-10);
            (*current_motor).runSpeedToPosition();
        }
    }else if(((mapX > 0 && mapY > 0) || (mapX > 0 && mapY < 0))){
        Serial.println("RIGHT or UP");
        if (gantryIsSet){
            pos[0] = -10;
            pos[1] =  10;
            gantry.moveTo(pos);
            gantry.runSpeedToPosition();
        }else{
            (*current_motor).move(10);
            (*current_motor).runSpeedToPosition();
        }
    }

    // Update last state of the switch
    if(switch_state == LOW){
        if (state == 3){
            prev_state = 3;
            state = 1;
        }else{
            prev_state = state;
            state++;
        }
    }
    */
}


void loop(){
    // test_run_group();
    control_motor();
    // test();
}