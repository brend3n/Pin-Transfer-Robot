//  SerialIn_SerialOut_HM-10_01
//
//  Uses hardware serial to talk to the host computer and AltSoftSerial for communication with the bluetooth module
//
//  What ever is entered in the serial monitor is sent to the connected device
//  Anything received from the connected device is copied to the serial monitor
//  Does not send line endings to the HM-10
//
//  Pins
//  BT VCC to Arduino 5V out.
//  BT GND to GND
//  Arduino D8 (SS RX) - BT TX no need voltage divider 
//  Arduino D9 (SS TX) - BT RX through a voltage divider (5v to 3.3v)
 

#include <AltSoftSerial.h>
AltSoftSerial BTserial; 
// https://www.pjrc.com/teensy/td_libs_AltSoftSerial.html
 
// Pins 46 and 48 represent the Arduino Mega's TX and RX pins, respectively.
char c=' ';
boolean NL = true;

// ACTIVE HIGH
// Used to simulate a
// change in state.
#define BUTTON 6

// Debounce time in ms
#define DEBOUNCE 50

// To be defined later by LCD
// prompt
#define NUM_PLATES 8

// Done means no more work needs to be done.
enum State {CALIBRATING, CALIBRATED, SRCUNLOADED, RCPUNLOADED, PROCESSED, WASHED, DRIED, SRCLOADED, RCPLOADED, DONE};

unsigned long lastPressTime = 0;
int buttonState = HIGH;
int lastButtonSate = HIGH;
unsigned long debounce = DEBOUNCE;

byte plate_number = 0, state = CALIBRATING;

void update_state() {
  if (plate_number == NUM_PLATES) {
    if (state == RCPLOADED) {
      state = DONE;
    } else if (state != DONE) {
      state++;
    }

    return;
  }

  if (state == RCPLOADED) {
    if (plate_number == NUM_PLATES) {
      state = DONE;
      return;
    }

    plate_number++;
    state = SRCUNLOADED;
    return;
  }

  state++;
}

void setup() {
  pinMode(BUTTON, INPUT);

  Serial.begin(9600);
  BTserial.begin(9600);  
  print_info();
}

String print_state() {
  switch(state) {
    case CALIBRATING:
    return "CALIBRATING";
    case CALIBRATED:
    return "CALIBRATED";
    case SRCUNLOADED:
    return "SRCUNLOADED";
    case RCPUNLOADED:
    return "RCPUNLOADED";
    case PROCESSED:
    return "PROCESSED";
    case WASHED:
    return "WASHED";
    case DRIED:
    return "DRIED";
    case SRCLOADED:
    return "SRCLOADED";
    case RCPLOADED:
    return "RCPLOADED";
    case DONE:
    return "DONE";

    default:
    return "UNDEFINED STATE";
  }
}

// You might not want to use this 
void print_info() {
  Serial.print("plate_number:"); Serial.print(plate_number); Serial.print(" state:"); Serial.println(print_state());
}

void loop() {
  // Read from the Bluetooth module and send to the Arduino Serial Monitor
  if (BTserial.available()) {
    // Was a useful debugging tool for
    // when I was using the LED to indicate
    // that I received data
    // digitalWrite(RECEIVE_LED, HIGH);
    // delay(250);
    // digitalWrite(RECEIVE_LED, LOW);

    Serial.write(c);
  }

  // Read from the Serial Monitor and send to the Bluetooth module
  if (Serial.available()) {
    c = Serial.read();

    // do not send line end characters to the HM-10

    if (c != '\n' && c != '\r') {
      BTserial.write(c);
    }

    // Echo the user input to the main window. 
    // If there is a new line print the ">" character.
    if (NL) { Serial.print("\r\n>");  NL = false; }
    Serial.write(c);
    if (c==10) { NL = true; }
  }

  int reading = digitalRead(BUTTON);

  if (reading != lastButtonSate) {
    lastPressTime = millis();
  }

  if (millis() - lastPressTime > debounce && reading != buttonState) {
    buttonState = reading;

    if (buttonState == LOW) {
      update_state();
      print_info();
    }
  }

  lastButtonSate = reading;
  print_info();
  
  BTserial.write(byte(plate_number * 10 + state));
  delay(100);
}
