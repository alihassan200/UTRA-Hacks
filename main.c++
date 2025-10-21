#include <Servo.h> 
#define MAX_COLORS 10
int servo = 12; 
Servo Servo1; 

const int trig = 10;  
const int echo = 11; 
float distance;
float duration;


int ENA = 7; 
int IN1 = 2; 
int IN2 = 3; 
int IN3 = 4; 
int IN4 = 5; 
int ENB = 6; 
int motor_speed;
int motor_speed1;

// define color sensor pins
#define S0 9
#define S1 8
#define S2 0
#define S3 13
#define sensorOut 1

// Calibration Values
// *Get these from Calibration Sketch
int redMin = 37; // Red minimum value
int redMax = 245; // Red maximum value
int greenMin = 20; // Green minimum value
int greenMax = 255; // Green maximum value
int blueMin = 32; // Blue minimum value
int blueMax = 190; // Blue maximum value

// Variables for Color Pulse Width Measurements
int redPW = 0;
int greenPW = 0;
int bluePW = 0;

// Variables for final Color values
int redValue;
int greenValue;
int blueValue;

struct ColorMapRed {
    String name;
    float values[4];  // Stores 4 values for each red detection
};

struct ColorMapGreen {
    String name;
    float values[4];  // Stores 4 values for each green detection
};

struct ColourMapBlue {
    String name;
    float values[4];  // Stores 4 values for each blue detection
};

ColorMapRed redDistances[MAX_COLORS];
ColorMapGreen greenDistances[MAX_COLORS];
ColourMapBlue blueDistances[MAX_COLORS];

int redIndex = 0;
int greenIndex = 0;
int blueIndex = 0;


void setup() {
  // put your setup code here, to run once:

  //Servo
  Servo1.attach(servo); 

  //Ultrasonic
  pinMode(trig, OUTPUT);  
  pinMode(echo, INPUT);  
  Serial.begin(9600); 

  //Driver Motors
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  //Colour
	// Set S0 - S3 as outputs
	pinMode(S0, OUTPUT);
	pinMode(S1, OUTPUT);
	pinMode(S2, OUTPUT);
	pinMode(S3, OUTPUT);

	// Set Sensor output as input
	pinMode(sensorOut, INPUT);

	// Set Frequency scaling to 20%
	digitalWrite(S0,HIGH);
	digitalWrite(S1,LOW);
  pinMode(A1, OUTPUT);

}

void loop() {
    forward();
  
    if (UltraSonic()) {
        gradualDeceleration();
        delay(300);
        backward();
        delay(220);
        stopMotors();
        delay(500);
        leftTurn();
        delay(1200);
        stopMotors();
        delay(300);
    }

    // Check the color (red, green, or blue)
    int colour = ColourCheck();  // Assuming ColourCheck returns 1 for red, 2 for green, 3 for blue

    if (colour == 1) {  // Red detected

        // Get the distances from the ultrasonic sensor
        float distance1 = getUltrasonicDistance();  
        delay(720);
        float distance2 = getUltrasonicDistance();
        delay(720);
        float distance3 = getUltrasonicDistance();
        delay(720);
        float distance4 = getUltrasonicDistance();

        bool matchFound = false;

        // Check if the current red distances match any stored red distances
       for (int i = 0; i < redIndex; i++) {
              float temp[4] = {distance1, distance2, distance3, distance4};  // Explicitly create an array
              if (compareArrays(redDistances[i].values, temp)) {
                matchFound = true;
                break;
              }
        }
        // If a match is found, move forward
        if (matchFound) {
            forward();
            Serial.println("Match found for red, moving forward.");
        } else {
            blinkLight();
            if (redIndex < MAX_COLORS) {
                redDistances[redIndex].values[0] = distance1;
                redDistances[redIndex].values[1] = distance2;
                redDistances[redIndex].values[2] = distance3;
                redDistances[redIndex].values[3] = distance4;
                redIndex++;
                Serial.println("New red distances stored.");
            } else {
                redIndex = 0;
                Serial.println("Max red detections reached, resetting.");
            }
        }

    } else if (colour == 2) {  // Green detected

        // Get the distances from the ultrasonic sensor for green
        float distance1 = getUltrasonicDistance();
        delay(720);
        float distance2 = getUltrasonicDistance();
        delay(720);
        float distance3 = getUltrasonicDistance();
        delay(720);
        float distance4 = getUltrasonicDistance();

        bool matchFound = false;

        // Check if the current green distances match any stored green distances
        for (int i = 0; i < redIndex; i++) {
              float temp[4] = {distance1, distance2, distance3, distance4};  // Explicitly create an array
              if (compareArrays(greenDistances[i].values, temp)) {
                matchFound = true;
                break;
        
              }
        }
        // If a match is found, move forward
        if (matchFound) {
            forward();
            Serial.println("Match found for green, moving forward.");
        } else {
            blinkLight();
            if (greenIndex < MAX_COLORS) {
                greenDistances[greenIndex].values[0] = distance1;
                greenDistances[greenIndex].values[1] = distance2;
                greenDistances[greenIndex].values[2] = distance3;
                greenDistances[greenIndex].values[3] = distance4;
                greenIndex++;
                Serial.println("New green distances stored.");
            } else {
                greenIndex = 0;
                Serial.println("Max green detections reached, resetting.");
            }
        }

    } else if (colour == 3) { 

        // Get the distances from the ultrasonic sensor for blue
        float distance1 = getUltrasonicDistance();
        delay(720);
        float distance2 = getUltrasonicDistance();
        delay(720);
        float distance3 = getUltrasonicDistance();
        delay(720);
        float distance4 = getUltrasonicDistance();

        bool matchFound = false;

        for (int i = 0; i < redIndex; i++) {
              float temp[4] = {distance1, distance2, distance3, distance4}; 
              if (compareArrays(redDistances[i].values, temp)) {
                matchFound = true;
                break;
        
              }
        }
        // If a match is found, move forward
        if (matchFound) {
            forward();
            Serial.println("Match found for blue, moving forward.");
        } else {
            blinkLight();
            if (blueIndex < MAX_COLORS) {
                blueDistances[blueIndex].values[0] = distance1;
                blueDistances[blueIndex].values[1] = distance2;
                blueDistances[blueIndex].values[2] = distance3;
                blueDistances[blueIndex].values[3] = distance4;
                blueIndex++;
                Serial.println("New blue distances stored.");
            } else {
                blueIndex = 0;
                Serial.println("Max blue detections reached, resetting.");
            }
        }

    }
    delay(1000);
}


bool UltraSonic(){
  digitalWrite(trig, LOW);  
	delayMicroseconds(2);  
	digitalWrite(trig, HIGH);  
	delayMicroseconds(10);  
	digitalWrite(trig, LOW); 

  duration = pulseIn(echo, HIGH);  
  distance = (duration*.0343)/2; 
  delay(500); 
  if(distance < 30){
    return true;
  }
  return false;
}

void blinkLight()   {
    // Blink the light (LED) to indicate a match
    digitalWrite(A1, HIGH);
    delay(500);  // Light stays on for 500ms
    digitalWrite(A1, LOW);
    delay(200);
}

float getUltrasonicDistance() {
    float sum = 0;
    for (int i = 0; i < 5; i++) { // Take 5 readings
        digitalWrite(trig, LOW);
        delayMicroseconds(2);
        digitalWrite(trig, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig, LOW);
        sum += (pulseIn(echo, HIGH) * 0.0343) / 2;
        delay(100);
    }
    return sum / 5; 
}
void forward() {
  // Set both motors to move forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 140);  
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 140);  
}

void leftTurn() {
  // Set left motor to move backward, right motor to move forward for a left turn
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 200);  // Adjust speed as needed
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 200);  // Adjust speed as needed
}

void rightturn() {
  // Set left motor to move backward, right motor to move forward for a left turn
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200);  // Adjust speed as needed
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 200);  // Adjust speed as needed
}

void backward() {
  // Set both motors to move backward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 180);  // Adjust speed as needed
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 180);  // Adjust speed as needed
}

bool compareArrays(float arr1[], float arr2[]) {
    int matchCount = 0;  // Counter for matches

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (abs(arr1[i] - arr2[j]) < 0.1) { // Tolerance for floating-point comparison
                matchCount++;
                break;  // Move to the next element in arr1
            }
        }
    }

    // Return true if 3 or more values match
    return matchCount >= 3;
}

void stopMotors() {
  // Stop both motors
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); 
  analogWrite(ENB, 0);
}


int ColourCheck(){
	// Read Red value
	int redPW = getRedPW();
	// Map to value from 0-255
	redValue = map(redPW, redMin,redMax,255,0);
	// Delay to stabilize sensor
	delay(200);

	// Read Green value
	int greenPW = getGreenPW();
	// Map to value from 0-255
	greenValue = map(greenPW, greenMin,greenMax,255,0);
	// Delay to stabilize sensor
	delay(200);

	// Read Blue value
	int bluePW = getBluePW();
	// Map to value from 0-255
	blueValue = map(bluePW, blueMin,blueMax,255,0);
	// Delay to stabilize sensor
	delay(200);

	// Print output to Serial Monitor
	Serial.print("Red = ");
	Serial.print(redValue);
	Serial.print(" - Green = ");
	Serial.print(greenValue);
	Serial.print(" - Blue = ");
	Serial.println(blueValue);
if(redValue < 60 && greenValue < 60 && blueValue < 60){
  return 0; //black
}
if(redValue > greenValue && redValue > blueValue){
  return 1; //red
}
if(greenValue > redValue && greenValue > blueValue){
  return 2; //green
}
if(blueValue > redValue && blueValue > greenValue){
  return 3; //blue
  
}
}


// Function to read Red Pulse Widths
int getRedPW() {
	// Set sensor to read Red only
	digitalWrite(S2,LOW);
	digitalWrite(S3,LOW);
	// Define integer to represent Pulse Width
	int PW;
	// Read the output Pulse Width
	PW = pulseIn(sensorOut, LOW);
	// Return the value
	return PW;
}

// Function to read Green Pulse Widths
int getGreenPW() {
	// Set sensor to read Green only
	digitalWrite(S2,HIGH);
	digitalWrite(S3,HIGH);
	// Define integer to represent Pulse Width
	int PW;
	// Read the output Pulse Width
	PW = pulseIn(sensorOut, LOW);
	// Return the value
	return PW;
}

// Function to read Blue Pulse Widths
int getBluePW() {
	// Set sensor to read Blue only
	digitalWrite(S2,LOW);
	digitalWrite(S3,HIGH);
	// Define integer to represent Pulse Width
	int PW;
	// Read the output Pulse Width
	PW = pulseIn(sensorOut, LOW);
	// Return the value
	return PW;
}

void gradualDeceleration() {
    // Gradually reduce the motor speed
    for (int speed = 255; speed > 100; speed -= 20) {  // Gradual speed reduction
        analogWrite(ENA, speed);  // Adjust speed of motor A
        analogWrite(ENB, speed);  // Adjust speed of motor B
        delay(500);  // Wait for a bit to let the deceleration happen
    }
    stopMotors();  // Completely stop after deceleration
}
