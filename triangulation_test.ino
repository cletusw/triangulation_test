#include <limits.h>
#include <NewPing.h>
#include <MedianFilter.h>


// For all distance readings
const uint8_t maxDistance = 100; // <=255 so it fits in the uint8_t
const uint8_t minDistance = 10;
const unsigned long distanceSamplePeriodMillis = 75; // need to wait 38ms +/- 10ms between IR readings,
unsigned long nextMillis = 0;

// Ultrasonic sensor
const uint8_t sonarDistanceToCenter = 11;
const uint8_t TRIGGER_PIN = 10;
const uint8_t ECHO_PIN = 11;
NewPing sonar(TRIGGER_PIN, ECHO_PIN);
volatile unsigned long latestSonarResult = 0;

// IR
const uint8_t irDistanceToCenter = 11;
const uint8_t IR0_PIN = A0;
const uint8_t IR1_PIN = A1;

// Angle between ultrasonic and IR sensors in radians
const double sensorAngle = 30 * PI / 180;

// For smoothing
const uint8_t distanceFilterSize = 7;
MedianFilter sonarFilter(distanceFilterSize, 0);
MedianFilter ir0(distanceFilterSize, 0);
MedianFilter ir1(distanceFilterSize, 0);


void setup() {
  Serial.begin(115200);

  pinMode(IR0_PIN, INPUT);
  pinMode(IR1_PIN, INPUT);

  nextMillis = millis();
  initializeSmoothing();
}

void loop() {
  if (millis() > nextMillis) {
    nextMillis += distanceSamplePeriodMillis;

    takeSonarReading();
    takeIrReadings();

    uint8_t ir0Distance = ir0.out() + irDistanceToCenter;
    uint8_t sonarDistance = sonarFilter.out() + sonarDistanceToCenter;
    uint8_t ir1Distance = ir1.out() + irDistanceToCenter;
    
//    Serial.print(ir0Distance);
//    Serial.print(' ');
//    Serial.print(sonarDistance);
//    Serial.print(' ');
//    Serial.print(ir1Distance);
//    Serial.println();

    // Four key angles, from left: A, B, C, D
    // _________
    // \A B|C D/
    //  \  |  /
    // ir0 s ir1
    //     onar
    //
    // (We actually don't care about D since it would be based off the
    // same two distance measurements as A, and we only use A and C to
    // get additional estimates of B)
    double B = getAngle(sensorAngle, ir0Distance, sonarDistance);
    double C = getAngle(sensorAngle, ir1Distance, sonarDistance);
    double B_1 = PI - C;
    double A = getAngle(2 * sensorAngle, ir1Distance, ir0Distance);
    double B_2 = PI - sensorAngle - A;

    Serial.print(A * 180 / PI);
    Serial.print(' ');
    Serial.print(B * 180 / PI);
    Serial.print(' ');
    Serial.print(C * 180 / PI);
    Serial.println();

    // Select the outlier from each triplet (because a single sensor that's reading infinity will mess up two angle estimates)
    // If B_avg <= C_avg, rotate right by B_avg
    // Else, rotate left by C_avg
  }
}

// Solves the ABC triangle using the Law of Cosines and returns angleB
// (all angles in radians)
double getAngle(double angleA, uint8_t distanceB, uint8_t distanceC) {
  uint8_t v;
  uint8_t w;
  boolean flipped;
  if (distanceB < distanceC) {
    v = distanceB;
    w = distanceC;
    flipped = false;
  }
  else {
    v = distanceC;
    w = distanceB;
    flipped = true;
  }
  double interior = (pow(v, 2) * pow(sin(angleA), 2)) / (pow(v, 2) + pow(w, 2) - 2 * v * w * cos(angleA));
  double smallerAngle = asin(sqrt(interior));
  if (flipped) {
    return PI - angleA - smallerAngle;
  }
  else {
    return smallerAngle;
  }
}

void initializeSmoothing() {
  // fill buffer with readings
  for (uint8_t i = 0; i < distanceFilterSize; i++) {
    while (millis() < nextMillis);
    nextMillis += distanceSamplePeriodMillis;
    handleSonarResult(sonar.ping());
    takeIrReadings();
  }
}

void takeSonarReading() {
  sonar.timer_stop();
  unsigned long sonarResult = latestSonarResult; // Not worried about a data race because we stopped the timer interrupt
  latestSonarResult = 0;
  sonar.ping_timer(echoCheck); // Start the next one ASAP
  handleSonarResult(sonarResult ? sonarResult : NO_ECHO);
}

void echoCheck() {
  if (sonar.check_timer()) {
    latestSonarResult = sonar.ping_result;
  }
}

void handleSonarResult(unsigned long pingTimeMicroseconds) {
  uint8_t noisyDistanceInCm =
    (pingTimeMicroseconds == NO_ECHO ||
    pingTimeMicroseconds > UINT_MAX) ?
      maxDistance :
      constrain(
        NewPing::convert_cm(pingTimeMicroseconds) + 1 /* measured error */,
        minDistance,
        maxDistance);
  sonarFilter.in(noisyDistanceInCm);
}

void takeIrReadings() {
  ir0.in(irReadingToDistance(analogRead(IR0_PIN)));
  ir1.in(irReadingToDistance(analogRead(IR1_PIN)));
}

// Returns distance in cm
uint8_t irReadingToDistance(uint16_t value) {
  if (value < 10) value = 10;
  return constrain(
      (6787.0 / (value - 3.0)) - 4.0 - 3.0 /* measured error */, minDistance, maxDistance);
}

