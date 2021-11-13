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

#define dirPiny  27
#define stepPiny 26

#define CLOSE 50
#define OPEN  0

#define UP 500
#define DOWN -500

#define y_switch 22

// if one stepper for gantry
// #define dirPinGantry -1
// #define stepPinGantry -1


#define x_dir A0
#define y_dir A1
#define switch_s 13
#define buttonPin 53 


#define SPEED_FACTOR 1.25

// Switch variables
int switch_state;
int switch_count = 1;

// Button variables
int button_state;
bool button_mode = false;


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

void setup(){

    Serial.begin(9600);

    pinMode(x_dir, INPUT);
    pinMode(y_dir, INPUT);
    pinMode(switch_s, INPUT_PULLUP);
    pinMode(buttonPin, INPUT);

    pinMode(y_switch, INPUT);

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

    servo.write(OPEN);
}

void get_states(){
    x_pos = analogRead(x_dir);  
    y_pos = analogRead(y_dir);
    switch_state = digitalRead(switch_s);
    button_state = digitalRead(buttonPin);

    mapX = map(x_pos, 0,1024,-512,512);
    mapY = map(y_pos, 0,1024,-512,512);
}
void control_motor(){
    get_states();

    // Toggle button mode each time it is clicked
    if (button_state == HIGH){
      delay(200);
      button_mode = !button_mode;
      Serial.println("Button toggled");
    }
    
    if (mapX > 400){
        Serial.println("RIGHT");
        motor_x1.setSpeed(4000);
        motor_x2.setSpeed(-4000);
        while (mapX > 400){
            // motor_x1.setSpeed(motor_x1.speed()*SPEED_FACTOR);
            // motor_x2.setSpeed(motor_x2.speed()*SPEED_FACTOR);
            motor_x1.runSpeed();
            motor_x2.runSpeed();
            get_states();
        }
    }else if (mapX < -400){
        Serial.println("LEFT");
        motor_x1.setSpeed(-4000);
        motor_x2.setSpeed(4000);
        while (mapX < -400){
            // motor_x1.setSpeed(motor_x1.speed()*SPEED_FACTOR);
            // motor_x2.setSpeed(motor_x2.speed()*SPEED_FACTOR);
            motor_x1.runSpeed();
            motor_x2.runSpeed();
            get_states();
        }
    }else if(mapY > 400){

        if (!button_mode){
          // Z-axis
          Serial.println("DOWN");
          motor_z.setSpeed(DOWN * 10);
          while (mapY > 400){
              // motor_z.setSpeed(motor_z.speed()*SPEED_FACTOR);
              motor_z.runSpeed();
              get_states();
          }
        }else{
          //Y-axis
          Serial.println("Y LEFT");
          motor_y.setSpeed(-1000);
          while (mapY > 400){
              // motor_y.setSpeed(motor_y.speed()*SPEED_FACTOR);
              if(digitalRead(y_switch) == LOW){
                motor_y.stop();
                break;
              }
              motor_y.runSpeed();
              get_states();
          }

        }
        
    }else if(mapY < -400){
        if (!button_mode){
          // Z-axis
          motor_z.setSpeed(UP*10);
          while (mapY < -400){
              // motor_z.setSpeed(motor_z.speed()*SPEED_FACTOR);
              motor_z.runSpeed();
              get_states();
          }
        }else{
          //Y-axis
          Serial.println("Y RIGHT");
          motor_y.setSpeed(1000);
          while (mapY < -400){
              if(digitalRead(y_switch) == LOW){
                  motor_y.stop();
                  break;
              }
              // motor_y.setSpeed(motor_y.speed()*SPEED_FACTOR);
              motor_y.runSpeed();
              get_states();
          }
        }
    }else{
        motor_z.stop();
        motor_x1.stop();
        motor_x2.stop();
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

void loop(){
    control_motor();
}