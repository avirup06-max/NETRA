# NETRA
Engineered NETRA – wearable tech guiding the blind with smart sensing.

Netra is a wearable assistive technology designed to support visually impaired individuals by providing real-time obstacle detection and GPS-based location tracking. The system integrates ultrasonic sensors, a GPS+GSM module (A9G), and a buzzer to deliver both spatial awareness and emergency communication.

---

*Features*

Obstacle Detection: Utilizes three ultrasonic sensors to detect nearby objects in multiple directions.

Location Tracking: Sends live GPS coordinates via SMS in a Google Maps-compatible format using the A9G module.

Alert System: Triggers a buzzer alert when an object is detected within a predefined distance threshold.

Motor Feedback : Vibration motors can be used for haptic feedback, adjusting intensity based on proximity.



---

Hardware Components

Microcontroller (Arduino)

A9G GSM + GPS Module

Ultrasonic Sensors (3x HC-SR04)

Buzzer

Vibration Motors (3x)

Power Supply (battery)

Jumper Wires, Breadboard or Custom PCB

Head-mounted wearable (e.g., cap )



---

Pin Configuration

Component	Microcontroller Pin

A9G RX / TX	D7 / D6
Ultrasonic Sensor 1	Trig: D2 / Echo: D3
Ultrasonic Sensor 2	Trig: D4 / Echo: D5
Ultrasonic Sensor 3	Trig: D8 / Echo: D13
Motor Outputs	D9, D10, D11
Buzzer	D12



---

Installation

1. Clone the repository:

git clone https://github.com/yourusername/netra-smart-cap.git


2. Install required Arduino libraries:

TinyGPSPlus



3. Open the code in the Arduino IDE and upload it to your microcontroller.




---

Working Principle

Ultrasonic sensors measure the distance to nearby obstacles.

If an object is within a critical range, the buzzer is activated and motors vibrate to alert the wearer.

The GPS module collects and parses location data at set intervals.

Once a set number of valid GPS readings are collected, an SMS containing a Google Maps link is sent via the GSM module to a predefined mobile number.



---

Usage

Mount the sensors on the cap facing forward, left, and right.

Upload the code to the microcontroller.

Power on the device and wait for GSM/GPS initialization.

The system will start monitoring the environment and periodically transmit the location.

---

![WhatsApp Image 2025-07-19 at 01 17 25_e5ca6987](https://github.com/user-attachments/assets/1d93c2ca-d2ce-4e29-99ca-fb324ffe0a67)


---

Author

Aviup Bhunia
Electronics and Communication Engineering Student
Focus: Embedded Systems, IoT, and Assistive Technologies
