#include <Servo.h>

// Arduino pin assignment
#define PIN_LED   9   // LED active-low
#define PIN_TRIG  12  // sonar sensor TRIGGER
#define PIN_ECHO  13  // sonar sensor ECHO
#define PIN_SERVO 10  // servo motor

// configurable parameters for sonar
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define DIST_MIN 180.0    // minimum distance to be measured (unit: mm)
#define DIST_MAX 360.0    // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)     // coefficient to convert duration to distance

#define EMA_ALPHA 0.5      // EMA weight of new sample (range: 0 to 1)

// duty duration for myservo.writeMicroseconds()
#define DUTY_MIN 550       // servo full clockwise position (0 degree)
#define DUTY_MAX 2400      // servo full counterclockwise position (180 degree)

// global variables
float dist_ema, dist_prev = DIST_MAX; // unit: mm
unsigned long last_sampling_time;     // unit: ms

Servo myservo;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);    // sonar TRIGGER
  pinMode(PIN_ECHO, INPUT);     // sonar ECHO
  digitalWrite(PIN_TRIG, LOW);  // turn-off Sonar

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(DUTY_MIN); // Initial position (0 degree)

  dist_prev = DIST_MIN; // raw distance output from USS (unit: mm)

  Serial.begin(57600);
}

void loop() {
  float dist_raw;

  if (millis() < (last_sampling_time + INTERVAL))
    return;

  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO); // read distance

  if ((dist_raw == 0.0) || (dist_raw > DIST_MAX)) {
    dist_raw = dist_prev;           
    digitalWrite(PIN_LED, HIGH);    // LED OFF (LED is active-low)
  } else if (dist_raw < DIST_MIN) {
    dist_raw = dist_prev;
    digitalWrite(PIN_LED, HIGH);    // LED OFF
  } else {
    dist_prev = dist_raw;
    digitalWrite(PIN_LED, LOW);     // LED ON
  }

  // Apply EMA filter
  dist_ema = EMA_ALPHA * dist_raw + (1 - EMA_ALPHA) * dist_ema;

  // Control servo based on distance
  int servo_angle;
  if (dist_ema <= 180.0) {
    servo_angle = 0;  // 0 degree
  } else if (dist_ema >= 360.0) {
    servo_angle = 180; // 180 degree
  } else {
    servo_angle = map(dist_ema, 180, 360, 0, 180); // proportional control
  }

  myservo.write(servo_angle); // move servo

  // Output the distance to the serial port
  Serial.print("Distance (mm): ");
  Serial.print(dist_ema);
  Serial.print(", Servo Angle: ");
  Serial.println(servo_angle);

  last_sampling_time = millis();  // update last sampling time
}

float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}
