#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// UART pins for the A9G GPS/GSM module
static const int RXPin = TXD0;  // GPIO35 as RX
static const int TXPin = RXD0;  // GPIO34 as TX
static const uint32_t GPSBaud = 9600;

// Pins for Ultrasonic Sensors
const int trigPin1 = 2;
const int echoPin1 = 4;
const int motorPin1 = 14;   // PWM capable

const int trigPin2 = 18;
const int echoPin2 = 19;
const int motorPin2 = 15;  // PWM capable

const int trigPin3 = 12;
const int echoPin3 = 13;
const int motorPin3 = 26;  // PWM capable

const int buzzerPin = 27;  // PWM capable

// Motor speed settings
const int speedUnder25 = 255;
const int speedUnder35 = 240;
const int speedUnder45 = 220;
const int speedUnder55 = 200;
const int speedUnder65 = 180;
const int speedUnder75 = 160;
const int speedUnder85 = 140;
const int speedUnder95 = 120;
const int speedDefault = 0;

// Distance thresholds in centimeters (sensor detection range)
const int threshold120 = 120;  // ~30 km/h - Critical alert
const int threshold160 = 160;  // ~40 km/h - Critical alert
const int threshold200 = 200;  // ~50 km/h - Critical alert
const int threshold250 = 250;  // ~60 km/h - Critical alert
const int threshold300 = 300;  // ~70 km/h - Critical alert
const int threshold350 = 350;  // ~80 km/h - Critical alert
const int threshold450 = 450;  // ~90 km/h - Critical alert
const int threshold600 = 600;  // ~100 km/h - Critical alert

const unsigned long loopDelay = 20;  // 20ms loop for faster response

// URL for Google Maps location sharing
String s = "www.google.com/maps/dir/";

// Time interval for sending GPS data (10 seconds)
unsigned long interval = 10000;
unsigned long previousMillis = 0;
int data_counter = 0;

TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

// PWM Configuration (ESP32 LEDC)
const int pwmFrequency = 5000;
const int pwmResolution = 8;
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;
const int pwmChannel3 = 2;
const int pwmBuzzer = 3;

const int sensorSmoothingFactor = 3;  // Average of 3 readings
long distance1_buffer[3] = {0, 0, 0};
long distance2_buffer[3] = {0, 0, 0};
long distance3_buffer[3] = {0, 0, 0};
int bufferIndex = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  SerialGPS.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
  Serial.println("Starting ESP32 - Optimized Autonomous Braking System");
  Serial.println("Response Time: ~20ms | Sensor Range: Dynamic based on speed");

  sendATCommand("AT");
  sendATCommand("AT+GPS=1");
  sendATCommand("AT+CREG=2");
  sendATCommand("AT+CGATT=1");
  sendATCommand("AT+CGDCONT=1,\"IP\",\"WWW\"");
  sendATCommand("AT+CGACT=1,1");
  
  sendATCommand("AT+GPS=1");
  sendATCommand("AT+GPSRD=10");
  sendATCommand("AT+CMGF=1");

  // Initialize Ultrasonic Sensors
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(buzzerPin, OUTPUT);

  // Configure PWM for motors
  ledcSetup(pwmChannel1, pwmFrequency, pwmResolution);
  ledcSetup(pwmChannel2, pwmFrequency, pwmResolution);
  ledcSetup(pwmChannel3, pwmFrequency, pwmResolution);
  ledcSetup(pwmBuzzer, pwmFrequency, pwmResolution);

  ledcAttachPin(motorPin1, pwmChannel1);
  ledcAttachPin(motorPin2, pwmChannel2);
  ledcAttachPin(motorPin3, pwmChannel3);
  ledcAttachPin(buzzerPin, pwmBuzzer);

  // Initialize motors to OFF
  ledcWrite(pwmChannel1, speedDefault);
  ledcWrite(pwmChannel2, speedDefault);
  ledcWrite(pwmChannel3, speedDefault);
  ledcWrite(pwmBuzzer, 0);

  Serial.println("System Initialized - Ready for operation");
}

void loop() {
  unsigned long loopStartTime = millis();
  unsigned long currentMillis = loopStartTime;

  // FAST SENSOR READING (Non-blocking approach)
  // Read all three sensors with minimal blocking time
  long distance1 = readUltrasonicDistance(trigPin1, echoPin1);
  long distance2 = readUltrasonicDistance(trigPin2, echoPin2);
  long distance3 = readUltrasonicDistance(trigPin3, echoPin3);

  // Apply sensor smoothing to reduce noise
  distance1 = applySensorSmoothing(distance1, distance1_buffer, 0);
  distance2 = applySensorSmoothing(distance2, distance2_buffer, 1);
  distance3 = applySensorSmoothing(distance3, distance3_buffer, 2);

  // Debugging output
  Serial.print("[");
  Serial.print(millis());
  Serial.print("ms] D1: ");
  Serial.print(distance1);
  Serial.print(" cm | D2: ");
  Serial.print(distance2);
  Serial.print(" cm | D3: ");
  Serial.print(distance3);
  Serial.println(" cm");

  // INTELLIGENT SPEED CONTROL BASED ON DISTANCE
  int speed1 = getMotorSpeed(distance1);
  int speed2 = getMotorSpeed(distance2);
  int speed3 = getMotorSpeed(distance3);

  ledcWrite(pwmChannel1, speed1);
  ledcWrite(pwmChannel2, speed2);
  ledcWrite(pwmChannel3, speed3);

  // EMERGENCY ALERT SYSTEM
  // Trigger buzzer at critical distance thresholds
  if ((distance1 > 0 && distance1 <= threshold120) ||
      (distance2 > 0 && distance2 <= threshold120) ||
      (distance3 > 0 && distance3 <= threshold120)) {
    ledcWrite(pwmBuzzer, 255); // Full alert
  } else if ((distance1 > 0 && distance1 <= threshold160) ||
             (distance2 > 0 && distance2 <= threshold160) ||
             (distance3 > 0 && distance3 <= threshold160)) {
    ledcWrite(pwmBuzzer, 180); // Medium alert (PWM)
  } else {
    ledcWrite(pwmBuzzer, 0);   // No alert
  }

  // GPS processing (non-blocking, quick read)
  smartDelay(0);  // Just read available data, don't wait
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("⚠️  No GPS data received: check wiring"));
  }

  // Send GPS data at intervals
  if ((unsigned long)(currentMillis - previousMillis) >= interval) {
    send_gps_data();
    previousMillis = currentMillis;
  }

  // TIGHT LOOP WITH MINIMAL DELAY
  // Calculate actual loop execution time and compensate
  unsigned long loopExecutionTime = millis() - loopStartTime;
  if (loopExecutionTime < loopDelay) {
    delay(loopDelay - loopExecutionTime);
  }
}

// Apply sensor smoothing using a rolling buffer
long applySensorSmoothing(long newValue, long* buffer, int index) {
  // Shift buffer values
  buffer[0] = buffer[1];
  buffer[1] = buffer[2];
  buffer[2] = newValue;
  
  // Ignore invalid readings (-1 means timeout/no object)
  int validCount = 0;
  long sum = 0;
  for (int i = 0; i < 3; i++) {
    if (buffer[i] > 0) {
      sum += buffer[i];
      validCount++;
    }
  }
  
  return (validCount > 0) ? (sum / validCount) : -1;
}

// Handle incoming GPS data
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (SerialGPS.available()) {
      gps.encode(SerialGPS.read());
    }
  } while (millis() - start < ms);
}

// Send AT command
void sendATCommand(String command) {
  SerialGPS.println(command);
  delay(1000);
  while (SerialGPS.available()) {
    Serial.write(SerialGPS.read());
  }
}

// Send GPS data via SMS
void send_gps_data() {
  if (gps.location.lat() == 0 || gps.location.lng() == 0)  {
    Serial.println("Invalid GPS - Return Executed");
    return;
  }

  data_counter++;
  float latitude = gps.location.lat();
  float longitude = gps.location.lng();
  Serial.print("[GPS] Lat: ");
  Serial.print(latitude);
  Serial.print(" | Lon: ");
  Serial.println(longitude);

  s += String(latitude, 6);
  s += ",";
  s += String(longitude, 6);
  s += "/";

  if (data_counter >= 10)  {
    data_counter = 0;
    Serial.println("[SMS] Sending Message...");
    sendATCommand("AT+CMGF=1");
    sendATCommand("AT+CNMI=2,2,0,0,0");
    SerialGPS.print("AT+CMGS=\"+919339858145\"\r"); 
    delay(1000);
    SerialGPS.print(s);
    SerialGPS.write(0x1A);
    delay(1000);
    
    s = "www.google.com/maps/dir/";
  }
}

// OPTIMIZED ULTRASONIC READING WITH REDUCED TIMEOUT
// Reduced pulseIn timeout for faster response (300ms instead of default 1s)
long readUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reduced timeout to 20ms (max distance ~3.4 meters)
  // For higher accuracy in close-range scenarios
  long duration = pulseIn(echoPin, HIGH, 20000);  // 20ms timeout
  
  long distance = duration * 0.034 / 2;
  
  return (duration == 0) ? -1 : distance;
}

// DYNAMIC SPEED CONTROL BASED ON REAL-WORLD BRAKING DATA
// Speed reduces gradually as obstacles approach, preventing sudden stops
int getMotorSpeed(long distance) {
  // Distance in cm -> Speed ranges based on safe braking distances
  
  if (distance <= threshold120) {
    return speedDefault;  // STOP (120cm ~ 30 km/h)
  } else if (distance <= threshold160) {
    return speedUnder25;  // Critical (160cm ~ 40 km/h)
  } else if (distance <= threshold200) {
    return speedUnder35;  // (200cm ~ 50 km/h)
  } else if (distance <= threshold250) {
    return speedUnder45;  // (250cm ~ 60 km/h)
  } else if (distance <= threshold300) {
    return speedUnder55;  // (300cm ~ 70 km/h)
  } else if (distance <= threshold350) {
    return speedUnder65;  // (350cm ~ 80 km/h)
  } else if (distance <= threshold450) {
    return speedUnder75;  // (450cm ~ 90 km/h)
  } else if (distance <= threshold600) {
    return speedUnder85;  // (600cm ~ 100 km/h)
  } else {
    return speedUnder95;  // Full speed
  }
}
