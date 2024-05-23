#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

// Define pins for ultrasonic sensors
#define TRIG_PIN_1 5
#define ECHO_PIN_1 18
#define TRIG_PIN_2 33
#define ECHO_PIN_2 15

unsigned long previousMillis = 0; // For keeping track of time
const long interval = 5000; // Interval of 5 seconds

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(16, 17); // RX, TX

void setup() {
  Serial.begin(115200); // For communication with ESP8266

  pinMode(TRIG_PIN_1, OUTPUT);
  pinMode(ECHO_PIN_1, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);

  ss.begin(9600); // Initialize the GPS module

  Serial.println("Setup complete.");
}

void loop() {
  unsigned long currentMillis = millis();

  while (ss.available() > 0) {
    gps.encode(ss.read());
  }

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Read distances from ultrasonic sensors
    long distance1 = measureDistance(TRIG_PIN_1, ECHO_PIN_1);
    long distance2 = measureDistance(TRIG_PIN_2, ECHO_PIN_2);

    // Calculate the average distance
    long averageDistance = (distance1 + distance2) / 2;

    // Get GPS coordinates
    float latitude = gps.location.isValid() ? gps.location.lat() : 0.0; // Default to 0 if invalid
    float longitude = gps.location.isValid() ? gps.location.lng() : 0.0; // Default to 0 if invalid

    // Create the message in the format "averageDistance,latitude,longitude"
    String message = String(averageDistance) + "," + String(latitude, 6) + "," + String(longitude, 6) + "\n";
    
    // Send the message via Serial to ESP8266
    Serial.print(message);
  }
}

long measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.0344 / 2;

  return distance;
}

