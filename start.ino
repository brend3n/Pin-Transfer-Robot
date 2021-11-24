/*
 * LCD_RD  A0
 * LCD_WR  A1
 * LCD_RS  A2
 * LCD_CS  A3
 * LCD_RST A4
 * LCD_D2  D2
 * LCD_D3  D3
 * LCD_D4  D4
 * LCD_D5  D5
 * LCD_D6  D6
 * LCD_D7  D7
 * LCD_D0  D8
 * LCD_D1  D9
 * SD_SCK  D13
 */

//#include <Adafruit_TFTLCD.h>
#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
#include <TouchScreen.h>

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
MCUFRIEND_kbv tft;
//Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
#define BLACK 0x0000
#define WHITE 0xFFFF


void setup(void) 
{
  Serial.begin(9600);
  tft.reset();
  uint16_t identifier = tft.readID();
  
  tft.begin(identifier);
  tft.setRotation(2);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  pinMode(13, OUTPUT);
}
void loop(void) 
{
  greeting();
  while (true)
  {
    plateNumberSetup();
    int numPlates = plateNumberInput();
    depthSetup();
    int depth = depthInput();
    washStepSetup();
    bool steps[3];
    washStepInput(steps);
    if (paramCheck(numPlates, depth, steps))
      break;
  }
  String loc[] = {"Input",  "Transfer", "Wash", "Dry", "Output"};
  tft.fillScreen(BLACK);
  for(int i = 0; i < 8; i++)
  {
    for (int j = 0; j < 5; j++)
    {
      progressScreen(i, loc[j]);
      delay(2000);
    }
  }
  redo();
}

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
  delay(3000);
}

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

void progressScreen(int plateNum, String location)
{
  tft.fillRect(154, 115, 30, 30, BLACK);
  tft.fillRect(130, 155, 120, 30, BLACK);
  tft.setCursor(70, 120);
  tft.print("Plate #");
  tft.print(plateNum);
  tft.setCursor(20, 160);
  tft.print("Location: ");
  tft.print(location);
}

boolean redo()
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
      Serial.println(x);
      Serial.println(y);
      Serial.println();
      if (x >= 106 && x <= 214 && y >= 90 && y <= 105)
        return true;
    }
  }
  return true;
}
