#include <dht11.h>
// Temperature sensor
#define DHT11PIN        4
#define DATA_VALID_PIN  13
#define DATA_PINS_START 5

// Ultrasonic sensors
#define TRIG_PIN   3
#define ECHO_PIN   2
#define TRIG_PIN2 22
#define ECHO_PIN2 23
#define INC_PIN 24
#define DEC_PIN 25

// Gesture detection
const int THRESHOLD_CM = 15;
const unsigned long GESTURE_WINDOW = 350;

// Flame detector
#define FLAME 26
#define FLAME_DET 27

// Globals
dht11 DHT11;
unsigned long lastTriggerTime = 0;
int lastSensor = 0;  // 0 for none, 1 for left, 2 for right

void setup()
{
  Serial.begin(9600);
  // DE10 lite bus
  pinMode(DATA_VALID_PIN, OUTPUT);
  digitalWrite(DATA_VALID_PIN, LOW);

  // 8 bit temperature output pins
  for (int i = 0; i < 8; i++) {
    pinMode(DATA_PINS_START + i, OUTPUT);
    digitalWrite(DATA_PINS_START + i, LOW);
  }

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN2, OUTPUT);
  pinMode(ECHO_PIN2, INPUT);

  pinMode(INC_PIN, OUTPUT);
  pinMode(DEC_PIN, OUTPUT);
  digitalWrite(INC_PIN, LOW);
  digitalWrite(DEC_PIN, LOW);
  pinMode(FLAME, INPUT);
  pinMode (FLAME_DET, OUTPUT);
  digitalWrite(FLAME_DET, LOW);

  Serial.println("setup complete");
}

// ultrasonic distance reading
long readDistanceCm(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000);  // timeout
  if (duration == 0) return -1;

  return duration / 58;
}

// ultrasonic gesture detection
void detectGesture(long d1, long d2)
{
  unsigned long now = millis();

  bool leftActive  = (d1 > 0 && d1 < THRESHOLD_CM);
  bool rightActive = (d2 > 0 && d2 < THRESHOLD_CM);

  if (leftActive && lastSensor == 0) {
    lastSensor = 1;
    lastTriggerTime = now;
  }

  if (rightActive && lastSensor == 0) {
    lastSensor = 2;
    lastTriggerTime = now;
  }

  // from left to right
  if (lastSensor == 1 && rightActive && (now - lastTriggerTime) < GESTURE_WINDOW) {
    Serial.println("increment");
    digitalWrite(INC_PIN, HIGH);
    delayMicroseconds(100);
    digitalWrite(INC_PIN, LOW);

    lastSensor = 0;
  }

  // from right to left
  if (lastSensor == 2 && leftActive && (now - lastTriggerTime) < GESTURE_WINDOW) {
    Serial.println("decrement");
    digitalWrite(DEC_PIN, HIGH);
    delayMicroseconds(100);
    digitalWrite(DEC_PIN, LOW);

    lastSensor = 0;
  }

  // timeout reset
  if (lastSensor != 0 && (now - lastTriggerTime) > GESTURE_WINDOW) {
    lastSensor = 0;
  }
}

void loop()
{
  // read ultrasonic sensors
  long dist1 = readDistanceCm(TRIG_PIN, ECHO_PIN);
  long dist2 = readDistanceCm(TRIG_PIN2, ECHO_PIN2);

  // detect wave direction
  detectGesture(dist1, dist2);

  // flame detector
  if (digitalRead(FLAME) == 1){
      digitalWrite(FLAME_DET, HIGH);
    }
    else {
      digitalWrite(FLAME_DET, LOW);
    }

  // read temperature
  int chk = DHT11.read(DHT11PIN);
  if (chk == 0)
  {
    int temperature = DHT11.temperature;

    // output to DE10 lite
    for (int i = 0; i < 8; i++) {
      digitalWrite(DATA_PINS_START + i, bitRead(temperature, i));
    }

    // strobe DE10 lite latch
    digitalWrite(DATA_VALID_PIN, HIGH);
    delayMicroseconds(50);
    digitalWrite(DATA_VALID_PIN, LOW);
    
    Serial.print("Temp: ");
    Serial.print(temperature);
    Serial.println(" C");
  }

  delay(60);
}
