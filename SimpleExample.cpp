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

// Instantiating motor driver objects
AccelStepper motor_x1 = AccelStepper(interface, stepPinx1, dirPinx1);
AccelStepper motor_x2 = AccelStepper(interface, stepPinx2, dirPinx2);

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
    // Set maxmium speeds
    motor_x1.setMaxSpeed(MAX_SPEED);
    motor_x2.setMaxSpeed(MAX_SPEED);

    // Set acceleration
    motor_x1.setAcceleration(MAX_ACCELERATION);
    motor_x2.setAcceleration(MAX_ACCELERATION);

    // Add motors to multistepper object
    gantry.addStepper(motor_x1);
    gantry.addStepper(motor_x2);

}

void loop(){
    test_run_group();
}
