#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

// Pins for the A9G GPS/GSM module
static const int RXPin = 7, TXPin = 6;
static const uint32_t GPSBaud = 9600;

// Pins for Ultrasonic Sensors and Motors
const int trigPin1 = 2;
const int echoPin1 = 3;
const int motorPin1 = 9;

const int trigPin2 = 4;
const int echoPin2 = 5;
const int motorPin2 = 10;

const int trigPin3 = 8;
const int echoPin3 = 13;
const int motorPin3 = 11;

const int buzzerPin = 12;

// Motor speed settings
const int speedUnder30 = 255;
const int speedUnder40 = 240;
const int speedUnder50 = 220;
const int speedUnder60 = 200;
const int speedUnder70 = 180;
const int speedUnder80 = 160;
const int speedUnder90 = 140;
const int speedUnder100 = 120;
const int speedDefault = 0;

// Distance thresholds in centimeters
const int threshold40 = 40;
const int threshold50 = 50;
const int threshold60 = 60;
const int threshold70 = 70;
const int threshold80 = 80;
const int threshold90 = 90;
const int threshold100 = 100;

// URL for Google Maps location sharing
String s = "www.google.com/maps/dir/";

// Time interval for sending GPS data (10 seconds)
unsigned long interval = 10000;
unsigned long previousMillis = 0;
int data_counter = 0;

TinyGPSPlus gps; // The TinyGPSPlus object
SoftwareSerial ss(RXPin, TXPin); // The serial connection to the GPS device

void setup() {
  // Start serial communication with the computer
  Serial.begin(9600);
  // Start communication with the A9G module
  ss.begin(GPSBaud); 
  Serial.println("Starting...");

  // Initialize the A9G module
  sendATCommand("AT");
  sendATCommand("AT+GPS=1");
  sendATCommand("AT+CREG=2");
  sendATCommand("AT+CGATT=1");
  sendATCommand("AT+CGDCONT=1,\"IP\",\"WWW\"");
  sendATCommand("AT+CGACT=1,1");
  
  // Initialize GPS
  sendATCommand("AT+GPS=1");
  sendATCommand("AT+GPSRD=10"); // Set GPS read interval
  
  // Set SMS mode to text mode
  sendATCommand("AT+CMGF=1");

  // Initialize Ultrasonic Sensors and Motors
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(motorPin1, OUTPUT);

  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(motorPin2, OUTPUT);

  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(motorPin3, OUTPUT);

  pinMode(buzzerPin, OUTPUT);

  // Initialize motors to OFF
  analogWrite(motorPin1, speedDefault);
  analogWrite(motorPin2, speedDefault);
  analogWrite(motorPin3, speedDefault);

  // Initialize buzzer to OFF
  digitalWrite(buzzerPin, LOW);
}

void loop() {
  unsigned long currentMillis = millis();

  // Read distance from each ultrasonic sensor
  long distance1 = readUltrasonicDistance(trigPin1, echoPin1);
  long distance2 = readUltrasonicDistance(trigPin2, echoPin2);
  long distance3 = readUltrasonicDistance(trigPin3, echoPin3);

  // Debugging output
  Serial.print("Distance 1: ");
  Serial.print(distance1);
  Serial.print(" cm\t");

  Serial.print("Distance 2: ");
  Serial.print(distance2);
  Serial.print(" cm\t");

  Serial.print("Distance 3: ");
  Serial.print(distance3);
  Serial.println(" cm");

  // Control motor speed based on distance
  int speed1 = getMotorSpeed(distance1);
  int speed2 = getMotorSpeed(distance2);
  int speed3 = getMotorSpeed(distance3);

  // Set motor speeds
  analogWrite(motorPin1, speed1);
  analogWrite(motorPin2, speed2);
  analogWrite(motorPin3, speed3);

  // Control buzzer
  if (distance1 > 0 && distance1 <= threshold40 ||
      distance2 > 0 && distance2 <= threshold40 ||
      distance3 > 0 && distance3 <= threshold40) {
    digitalWrite(buzzerPin, HIGH); // Turn buzzer ON
  } else {
    digitalWrite(buzzerPin, LOW); // Turn buzzer OFF
  }

  // Wait for GPS data and check for valid data
  smartDelay(2000);
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS data received: check wiring"));
  }

  // Send GPS data at intervals
  if ((unsigned long)(currentMillis - previousMillis) >= interval) {
    send_gps_data();
    previousMillis = currentMillis;
  }

  // Small delay before next loop iteration
  delay(100);
}

// Function to handle incoming GPS data with a delay
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (ss.available()) {
      gps.encode(ss.read());
    }
  } while (millis() - start < ms);
}

// Function to send AT command and print response
void sendATCommand(String command) {
  ss.println(command);
  delay(1000);
  while (ss.available()) {
    Serial.write(ss.read());
  }
}

// Function to send GPS data via SMS
void send_gps_data() {
  // Check if the GPS location is valid
  if (gps.location.lat() == 0 || gps.location.lng() == 0)  {
    Serial.println("Return Executed");
    return;
  }

  // Increment data counter and store GPS data
  data_counter++;
  float latitude = gps.location.lat();
  float longitude = gps.location.lng();
  Serial.print("Latitude (deg): ");
  Serial.println(latitude);
  Serial.print("Longitude (deg): ");
  Serial.println(longitude);
  Serial.println(data_counter);
  Serial.println();

  // Construct the Google Maps URL with the GPS coordinates
  s += String(latitude, 6);
  s += ",";
  s += String(longitude, 6);
  s += "/";
  Serial.println(s);

  // Send the SMS after collecting a few data points
  if (data_counter >= 10)  {
    data_counter = 0;
    Serial.println("Sending Message");
    sendATCommand("AT+CMGF=1");
    sendATCommand("AT+CNMI=2,2,0,0,0");
    ss.print("AT+CMGS=\"+919339858145\"\r"); 
    delay(1000);
    ss.print(s);
    ss.write(0x1A); // Send the SMS
    delay(1000);
    
    // Reset the Google Maps URL for the next message
    s = "www.google.com/maps/dir/";
  }
}

// Function to read distance from ultrasonic sensor
long readUltrasonicDistance(int trigPin, int echoPin) {
  // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Set the trigPin high for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echoPin
  long duration = pulseIn(echoPin, HIGH, 30000); // Timeout after 30ms (max distance ~5 meters)
  
  // Calculate distance in centimeters
  long distance = duration * 0.034 / 2;
  
  // Return -1 if no echo was received
  if (duration == 0) {
    return -1;
  } else {
    return distance;
  }
}

// Function to get motor speed based on distance
int getMotorSpeed(long distance) {
  if (distance <= threshold40) {
    return speedUnder40;
  } else if (distance <= threshold50) {
    return speedUnder50;
  } else if (distance <= threshold60) {
    return speedUnder60;
  } else if (distance <= threshold70) {
    return speedUnder70;
  } else if (distance <= threshold80) {
    return speedUnder80;
  } else if (distance <= threshold90) {
    return speedUnder90;
  } else if (distance <= threshold100) {
    return speedUnder100;
  } else {
    return speedDefault;
  }
}