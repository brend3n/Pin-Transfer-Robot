// Motors
#include <AccelStepper.h>
#include <MultiStepper.h>

// Motor driver type
#define interface  1

// Direction and Step pins for all motors
#define dirPinx1  -1
#define stepPinx1 -1

#define dirPinx2  -1
#define stepPinx2 -1

#define dirPinz  -1
#define stepPinz -1

#define dirPiny  -1
#define stepPiny -1

// if one stepper for gantry
// #define dirPinGantry -1
// #define stepPinGantry -1


#define x_dir A0
#define y_dir A1
#define switch_s 13

int button_state;

int state = 1;
int prev_state = 0;

int x_pos;
int y_pos;
int mapX;
int mapY;

bool gantryIsSet = false;

// Instantiating motor driver objects
AccelStepper motor_x1 = AccelStepper(interface, stepPinx1, dirPinx1);
AccelStepper motor_x2 = AccelStepper(interface, stepPinx2, dirPinx2);
AccelStepper motor_y = AccelStepper(interface, stepPiny, dirPiny);
AccelStepper motor_z = AccelStepper(interface, stepPinz, dirPinz);

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
void setup(){

    Serial.begin(9600);

  pinMode(x_dir, INPUT);
  pinMode(y_dir, INPUT);
  pinMode(switch_s, INPUT_PULLUP);

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

}
void control_motor(){
    x_pos = analogRead(x_dir);
    y_pos = analogRead(y_dir);
    button_state = digitalRead(switch_s);

    mapX = map(x_pos, 0,1024,-512,512);
    mapY = map(y_pos, 0,1024,-512,512);

    if (state != prev_state){
        if(state == 1){
            Serial.println("Controlling: Gantry ");
            current_motor = &motor_x;
            // current_motor = &gantry;
        }else if(state == 2){
            Serial.println("Controlling: Y ");
            current_motor = &motor_y;
        }else if{
            Serial.println("Controlling: Z ");
            gantryIsSet = true;
        }else{
            gantryIsSet = false;
        }
    }
    if(mapX < 0 || mapY < 0){
        Serial.println("LEFT or DOWN");
        if (gantryIsSet){
            long pos;
            pos[0] =  1;
            pos[1] = -1;
            gantry.moveTo(pos);
            gantry.runSpeedToPosition();
        }else{
            *current_motor.moveTo(-10);
            *current_motor.runSpeedToPosition();
        }
    }else if(mapX > 0 || mapY > 0){
        Serial.println("RIGHT or UP");
        if (gantryIsSet){
            long pos;
            pos[0] = -1;
            pos[1] =  1;
            gantry.moveTo(pos);
            gantry.runSpeedToPosition();
        }else{
            *current_motor.moveTo(10);
            *current_motor.runSpeedToPosition();

        }
    }

    if( button_state == HIGH){
        if (state == 3){
            prev_state = 3;
            state = 1;
        }else{
            prev_state = state;
            state++;
        }
    }
}

void loop(){
    // test_run_group();
    control_motor();

}
