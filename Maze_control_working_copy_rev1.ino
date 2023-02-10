/*
Feb 2023
This Sketch is a creation by Chaosbuster. 
This combines 4 major elements to controlling an autonomous robot.

The libraries required are Adafruit_SSD1306.h for the OLED Display,
VL53L0X.h created by Polulu for the Lidar sensors and wire.h allowing I2C communications
to the sensors and the display.
The motors are controlled with the Adafruit library.

Using the multiple Lidar code, theoretically, any number of Lidars could be added.
This Sketch initialises 3 Lidar sensors and outputs their ranges in millimeters.
It outputs data to the 128X64 OLED display and the serial monitor if enabled 

The sensor readings are grouped in a function so that it's easy to add/subtract sensors as needed.
The output to the OLED is grouped in 2 places. The initial display is done in the Void Setup function.
The more complicated aspect of the display configuration is done in a function called OLED.
This function contains some basic conditional statements so that 
the OLED display reads "C" if the distance is greater than 800 centimeters

Button use a libray called ezbutton. It simplifies button use and offers improvements to reliability
Any number of button can be used (subject to digital pin avalability). We used four in this example
*/
#include <PID_v1.h>      // this library uses an algorithm that makes small adjustments to motor power in order to achieve more precise and smooth robot movement
#include <Smoothed.h>  // this library helps to smooth the lidar reading through averaging
  Smoothed  <int> mySensor1;
  Smoothed  <int> mySensor2;
  Smoothed  <int> mySensor3;
#include <Adafruit_SSD1306.h> //OLED Library (adafruit)
  Adafruit_SSD1306 display(128, 64, &Wire, 4);
#include <VL53L0X.h>  //Lidar library (Pololu)
  VL53L0X sensor1; VL53L0X sensor2; VL53L0X sensor3; // Give the sensors name ;sensor, sensor2, sensor3
#include <Wire.h>   //I2C communication library
  #define sensorAd1 0x30   // These 3 lines define the I2C address we are assigning to each one
  #define sensorAd2 0x31   // If we are using a single lidar, the adress is 0x29.
  #define sensorAd3 0x32
 
#include <ezButton.h> //
  ezButton button1(7); ezButton button2(6);ezButton button3(5);ezButton button4(4); // create ezButton objects that attach to digital pins;
    int SFront, SLeft, SRight, MisMatch; // These are variables for the smoothed data and error correction
    int FrontLimit = 100;
    int SideLimit = 100;
    int speed = 0; //overall speed
    int turnOffset; // this is how much we subtract from the inside wheel in a turn
    int duration = 100;  //Adjustment to make turns precise
    float Kp=0, Ki=0, Kd=0; // PID variables
    
    bool btn1State = 0;  bool btn2State = 0;  bool btn3State = 0;  bool btn4State = 0; // These variable hold button state
#include <Adafruit_MotorShield.h>  // designed for the "V2" Adaifruit (and most knockoff) shields
  Adafruit_MotorShield AFMS = Adafruit_MotorShield(); // defines motors
  Adafruit_DCMotor *LeftMotor = AFMS.getMotor(4);     // There are 4 (1,2,3,4) motor ports
  Adafruit_DCMotor *RightMotor = AFMS.getMotor(3);   // Adjust these lines for motor names and ports
#include "TimerOne.h" // for use with the wheel spped interupts
const byte MOTOR1 = 2;  // Motor 1 Interrupt Pin - INT 0
const byte MOTOR2 = 3;  // Motor 2 Interrupt Pin - INT 1
const byte Piezo = 11;  // Pin for Sound output
const byte Light = 12;  // Pin for headlight

unsigned int counter1 = 0;
unsigned int counter2 = 0;
float diskslots = 20;  // Change to match value of encoder disk
void setup() {
  Serial.begin(9600); // Sets up the serial output to use the serial monitor
  Wire.begin();      // Initiates the I2C communications protocol
  AFMS.begin(); // Invokes the library functions for motor control
  mySensor1.begin(SMOOTHED_AVERAGE, 5); // The higher the number the smoother the reading
  mySensor2.begin(SMOOTHED_AVERAGE, 5);  // Also the higher the longer it takes
  mySensor3.begin(SMOOTHED_AVERAGE, 5); 
display.clearDisplay();
initializeDisplay(); // Initiates formatting of the OLED Display
  pinMode(8, OUTPUT); // These 3 lines we make pins D0, D9 & D10 as outputs 
  pinMode(9, OUTPUT); // they wire to the X (Xshut terminal of each Lidars)
  pinMode(10, OUTPUT);
  pinMode(Light, OUTPUT); // make sure these pins are outputs
  pinMode(Piezo, OUTPUT);
  pinMode(4, INPUT_PULLUP);  // use internal resistors for switch pullup
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  
  initializeLidars();  // Initiates setup of the Lidar sensors
 
  button1.setDebounceTime(50); // set switch debounce time to 50 milliseconds
  button2.setDebounceTime(50); 
  button3.setDebounceTime(50); 
  button4.setDebounceTime(50);
  digitalWrite(Light, HIGH);  // tests light and piezo
  delay(2000);
  digitalWrite(Light, LOW);  // Verify they are working on boot up
  tone(Piezo, 500, 1000);
   //initializeLidars();  // Initiates setup of the Lidar sensors
 }

void loop(){
 initializeButtons();   // Must be done 1st !!
 if(button1.isPressed()){ loop1();} 
 if(button2.isPressed()){ loop2();}
 if(button3.isPressed()){ loop3();}
 if(button4.isPressed()){ loop4();}
   readSensors();           // This is a function to read sensors
   navigate();        // makes comparisons with front and side limits and follows left side rule
   OLED();           // Updates the display
   
 //serialMonitor();     // Updates the serial monitor commnet out if not needed
}
void loop1() {

  FrontLimit = 100;
  SideLimit = 100;
  speed = 0;             //overall speed
  int turnOffset = speed/2; // this is how much we slow the inside wheel in a turn
  duration = 100;          //Adjustment to make turns precise
  Kp=0, Ki=0, Kd=0;         // PID variables
  readSensors();           // This is a function to read sensors
  navigate();
  OLED();              //updates the OLED Display
  }
void loop2() {
  FrontLimit = 100;
  SideLimit = 100;
  speed = 75;             //overall speed
  int turnOffset = speed/2; // this is how much we slow the inside wheel in a turn
  duration = 100;          //Adjustment to make turns precise
  Kp=0, Ki=0, Kd=0;         // PID variables
  readSensors();           // This is a function to read sensors
  navigate();
  OLED();  //updates the OLED Display
}
void loop3() {
  FrontLimit = 100;
  SideLimit = 100;
  speed = 100;             //overall speed
  int turnOffset = speed/2; // this is how much we slow the inside wheel in a turn
  duration = 100;          //Adjustment to make turns precise
  Kp=0, Ki=0, Kd=0;         // PID variables
  readSensors();           // This is a function to read sensors
  navigate();
  OLED();  //updates the OLED Display
}
void loop4() {
 FrontLimit = 100;
  SideLimit = 100;
  speed = 150;             //overall speed
  int turnOffset = speed/2; // this is how much we slow the inside wheel in a turn
  duration = 100;          //Adjustment to make turns precise
  Kp=0, Ki=0, Kd=0;         // PID variables
  readSensors();           // This is a function to read sensors
  navigate();
  OLED();  //updates the OLED Display
}
void navigate(){
     if (SFront >= FrontLimit && SRight >= SideLimit && SLeft >= SideLimit)
{goStraight();}
else if (SFront >= FrontLimit && SRight >= SideLimit && SLeft <= SideLimit)
{leftTurn();}
else if (SFront >= FrontLimit && SRight <= SideLimit && SLeft >= SideLimit)
goStraight();
else if (SFront >= FrontLimit && SRight <= SideLimit && SLeft <= SideLimit) 
{goStraight();}
else if (SFront <= FrontLimit && SRight >= SideLimit && SLeft >= SideLimit)   
{leftTurn();}
else if (SFront <= FrontLimit && SRight >= SideLimit && SLeft <= SideLimit)
{rightTurn();}
else if (SFront <= FrontLimit && SRight <= SideLimit && SLeft >= SideLimit) 
{leftTurn();}
else if (SFront <= FrontLimit && SRight <= SideLimit && SLeft <= SideLimit) 
{leftTurn();}  
}
void Stop() // Stops motors
{
  LeftMotor->run(RELEASE);      //Make the robot roll forward
  LeftMotor->setSpeed(speed); // speed and duration values are passed to the function
  RightMotor->run(RELEASE);     // when the function is called
  RightMotor->setSpeed(speed);
  delay(1); // allows for the sensors to "see" from a stable position after a turn
  
}
void goStraight() // The variable speed and duration are passed from the function call
{
  LeftMotor->run(FORWARD);      //Make the robot roll forward
  LeftMotor->setSpeed(speed); // speed and duration values are passed to the function
  RightMotor->run(FORWARD);     // when the function is called
  RightMotor->setSpeed(speed);
}
void leftTurn()
{ 
  LeftMotor->run(BACKWARD);          // easier, at least for me.
  LeftMotor->setSpeed(turnOffset); // 0 is no turn, 127 is a hard turn
  RightMotor->run(FORWARD);             // however, tighter turns can be made by reversing
  RightMotor->setSpeed(speed); // one of the motors
  delay(duration);
  Stop();
}

void rightTurn()
{
  LeftMotor->run(FORWARD);             // notice the similarities between turning left
  LeftMotor->setSpeed(speed);  // and turning right, I just changed the sign
  RightMotor->run(BACKWARD);             // I know, I'm a genius...
  RightMotor->setSpeed(turnOffset);
  delay(duration);
  Stop();
  
}
void readSensors() { // Function to read data from the Lidars
    
  SFront = sensor1.readRangeContinuousMillimeters(); // These 3 lines read the lidar data
  SLeft  = sensor2.readRangeContinuousMillimeters();
  SRight = sensor3.readRangeContinuousMillimeters();
  mySensor1.add(SFront); // These 3 lines sends the data to the smoothing library
  mySensor2.add(SLeft);
  mySensor3.add(SRight);
  
  SFront = mySensor1.get(); // These 3 lines gets the smoothed data
  SLeft =  mySensor2.get();
  SRight =  mySensor3.get();
  MisMatch = SLeft - SRight;
}

void OLED() { // Function to update the display


  if (SFront > 800) {       // House keeping for the display
    display.setCursor(35, 0); // The first number 0-128 pixels horiontal. 
    display.print (" C ");    //  The second is 0-64 pixels vertical
  }
  else {
    display.setCursor(35, 0);
    display.print ("   ");     // this line erases any digits that we displayed from the last reading
    display.setCursor(35, 0);
    display.print (SFront);
  }
  
  if (SLeft > 800) {        // defines the maximum reading we will show
    display.setCursor(35, 16);  // The lidar default max 8190
    display.print (" C ");      // C stands for "clear"    
  }
  else {
    display.setCursor(35, 16);
    display.print ("   ");
    display.setCursor(35, 16);
    display.print (SLeft);
  }
  
    if (SRight > 800) {       
    display.setCursor(35, 32);
    display.print (" C ");
  }
  else {
    display.setCursor(35, 32);
    display.print ("   ");
    display.setCursor(35, 32);
    display.print (SRight);
  }
  
  
  display.setCursor(116,0);    // Display the button status
  display.print (btn1State);
  display.setCursor(116,16);
  display.print (btn2State);
  display.setCursor(116,32);
  display.print (btn3State);
  display.setCursor(116,50);
  display.print (btn4State);
  display.display();          // writes everything we just did above (the buffer) to the OLED
}


void serialMonitor(){       //
  Serial.print(SFront);
  Serial.println(F(" mm    Lidar 1 "));
  Serial.print(SLeft);
  Serial.println(F(" mm    Lidar 2"));
  Serial.print(SRight);
  Serial.println(F(" mm    Lidar 3"));
  Serial.println(F("___________________________"));

  
  Serial.print(F("button 1 state: "));
  Serial.println(btn1State);
  Serial.print(F("button 2 state: "));
  Serial.println(btn2State);
  Serial.print(F("button 3 state: "));
  Serial.println(btn3State);
  Serial.print(F("button 4 state: "));
  Serial.println(btn4State);
 
if(button1.isPressed()){
    Serial.println(F("The button 1 is pressed"));
    Serial.println(btn1State);}
if(button1.isReleased()){
   Serial.println(F("The button 1 is released"));
   Serial.println(btn1State);}
if(button2.isPressed())
   Serial.println(F("The button 2 is pressed"));

if(button2.isReleased())
    Serial.println(F("The button 2 is released"));

if(button3.isPressed())
   Serial.println(F("The button 3 is pressed"));

if(button3.isReleased())
  Serial.println(F("The button 3 is released"));
  
if(button4.isPressed())
   Serial.println(F("The button 4 is pressed"));

if(button4.isReleased())
    Serial.println(F("The button 4 is released"));
   
   delay (500); 
}
  void initializeDisplay() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);// this identifies the oled and it's I2C communications address
  display.display();// This displays the default memory that is Adafruit's logo
  delay(100); // Pause for 1/10 second for the logo to be displayed//Adafruit_SSD1306 display(128, 64, &Wire, 4); // Set up display configration
  display.clearDisplay();// Clear the buffer
  display.setTextSize(2); // set size 0 to 4
  display.setTextColor(WHITE, BLACK); // first color is the text, the second is the background
  display.setCursor(0, 0); // Initiate display
  display.print("F =");
  display.setCursor(76, 0); // Initiate display
  display.print("B1=");
  display.setCursor(0, 16); // Initiate display
  display.print("L =");
  display.setCursor(76, 16); // Initiate display
  display.print("B2=");
 display.setCursor(0, 33); // Initiate display
  display.print("R =");
  display.setCursor(76, 33); // Initiate display
  display.print("B3=");
  display.setCursor(0, 50); // Initiate display
  display.setCursor(76, 50); // Initiate display
  display.print("B4="); 
}
  void initializeLidars(){
 
  
  digitalWrite(8, LOW); // This disables both Lidars
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  
  delay(500);           // Give it a little time happen
  
  digitalWrite(10, HIGH);  // Re-enable the Lidar wired to p
    delay(150);      //Front
  sensor1.init(true);            // Initialize the sensor
  delay(300);                   // give it a little time to happen
  sensor1.setAddress(sensorAd1);   // Set the address to a new one (defined at the start)
  
  digitalWrite(9, HIGH);        //Re-enable the second Lidar
  delay(150);     //Left
  sensor2.init(true);             // Initialize the second sensor
  delay(300);
  sensor2.setAddress(sensorAd2);  // Set the address to a new one (defined at the start)
  
  digitalWrite(8, HIGH);        //Re-enable the third Lidar  
  delay(150);     //Right
  sensor3.init(true);             // Initialize the second sensor
  delay(300);
  sensor3.setAddress(sensorAd3);  // Set the address to a new one (defined at the start)
  
  sensor1.setTimeout(500);         // Gives the sensors some time to make the reading and send it
  sensor1.startContinuous();
  
  sensor2.setTimeout(500);
  sensor2.startContinuous();
 
  sensor3.setTimeout(500);
  sensor3.startContinuous();
  
} 
  
  void initializeButtons(){
  button1.loop(); // MUST call the loop() function first
  button2.loop(); 
  button3.loop(); 
  button4.loop(); 
  button1.getState();
  button2.getState();
  button3.getState();
  button4.getState();
  btn1State = button1.getState();
  btn2State = button2.getState();
  btn3State = button3.getState();
  btn4State = button4.getState();
  }