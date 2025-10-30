#include <Servo.h>

#define LEFT_SERVO_PIN 11
#define RIGHT_SERVO_PIN 12

#define LEFT_N_ANGLE 20.0
#define LEFT_W_ANGLE 85.0

#define RIGHT_N_ANGLE 155.0
#define RIGHT_E_ANGLE 90.0

#define DRIVEN_LEG_LENGTH 90.0
#define FREE_LEG_LENGTH 120.0
#define SPREAD 40.0

// Calibration modes - only one works at once
//#define CENTER_SERVOS
//#define WIGGLE_LEFT
//#define LEFT_N
//#define LEFT_W
//#define RIGHT_N
//#define RIGHT_E
//#define OSCLIATE_TEST
//#define CIRCLE_TEST
//#define TRIANGLE_FRACTAL

Servo leftServo;
Servo rightServo;

void setup() {
  leftServo.attach(LEFT_SERVO_PIN);
  rightServo.attach(RIGHT_SERVO_PIN);
}

#if defined(CENTER_SERVOS)
void loop() {
  leftServo.write(90.0);
  rightServo.write(90.0);
  delay(1000);
}
#elif defined(WIGGLE_LEFT)
void loop() {
  double center = 90.0;
  double amplitude = 10.0;

  leftServo.write(center - amplitude);
  delay(600);
  leftServo.write(center + amplitude);
  delay(600);
}
#elif defined(LEFT_N)
void loop() {
  setServosByHeading(0.0, 45.0);
  delay(1000);
}
#elif defined(LEFT_W)
void loop() {
  setServosByHeading(-90.0, 45.0);
  delay(1000);
}
#elif defined(RIGHT_N)
void loop() {
  setServosByHeading(-45.0, 0.0);
  delay(1000);
}
#elif defined(RIGHT_E)
void loop() {
  setServosByHeading(-45.0, 90.0);
  delay(1000);
}
#elif defined(OSCLIATE_TEST)
void loop() {
  for (double heading = 90.0; heading > 0.0; heading -= 1.0) {
    setServosByHeading(-heading, heading);
    delay(20);
  }
}
#elif defined(CIRCLE_TEST)
void loop() {
  double radius = 35.0;
  double centerX = 0.0;
  double centerY = 100.0;
  int steps = 180;

  for (int i = 0; i < steps; i++) {
    double angle = (2.0 * M_PI * i) / steps;  // angle in radians
    double x = centerX + radius * cos(angle);
    double y = centerY + radius * sin(angle);

    setServosByPosition(x, y);
    delay(50);
  }
}
#elif defined(TRIANGLE_FRACTAL)
void loop() {
  float ox = -40;
  float oy = 90;
  bool toggle = false;
  for (float x = 0; x < 100; x+=5) {
    float y = 100-x;
    if (toggle) {
      drawLine(ox, oy+y, ox+x, oy);
    } else {
      drawLine(ox+x, oy, ox, oy+y);
    }
    toggle = !toggle;
    delay(500);
  }
}

// Move servos along a straight line from (x0, y0) to (x1, y1)
void drawLine(double x0, double y0, double x1, double y1) {
  setServosByPosition(x0, y0);
  delay(500);
  int steps = 20;
  for (int i = 0; i <= steps; i++) {
    double t = (double)i / steps;
    double x = x0 + t * (x1 - x0);
    double y = y0 + t * (y1 - y0);
    setServosByPosition(x, y);
    delay(50); // adjust speed
  }
  setServosByPosition(x1, y1);
  delay(500);
}
#else
void loop() {
  setServosByHeading(-45.0, 45.0);
  delay(1000);
}
#endif

// Zero Heading - straight up
// Positive heading - clockwise
void setServosByHeading(double left, double right) {
  double leftAngle = mapf(
    normalizeHeading(left),
    -90.0, 0.0,
    LEFT_W_ANGLE, LEFT_N_ANGLE
  );
  double rightAngle = mapf(
    normalizeHeading(right),
    90.0, 0.0,
    RIGHT_E_ANGLE, RIGHT_N_ANGLE
  );
  leftServo.write(leftAngle);
  rightServo.write(rightAngle);
}

// Positive Y - straight up
// Positive X - right
// Zero - Center between the two joints
void setServosByPosition(double x, double y) {
  // Base positions
  double lx = -SPREAD / 2.0;
  double ly = 0.0;
  double rx =  SPREAD / 2.0;
  double ry = 0.0;

  double leftHeading = headingToPoint(lx, ly, x, y);
  double rightHeading = headingToPoint(rx, ry, x, y);

  double leftDistance = distanceToPoint(lx, ly, x, y);
  double rightDistance = distanceToPoint(rx, ry, x, y);

  double leftDrivenArmOffset = -cosineAngle(FREE_LEG_LENGTH, DRIVEN_LEG_LENGTH, leftDistance);
  double rightDrivenArmOffset = cosineAngle(FREE_LEG_LENGTH, DRIVEN_LEG_LENGTH, rightDistance);

  leftHeading += leftDrivenArmOffset;
  rightHeading += rightDrivenArmOffset;

  setServosByHeading(leftHeading, rightHeading);
}

double headingToPoint(double fromX, double fromY, double toX, double toY) {
  double dx = toX - fromX;
  double dy = toY - fromY;
  double angleRad = atan2(dx, dy);
  return angleRad * 180.0 / M_PI;
}

double distanceToPoint(double fromX, double fromY, double toX, double toY) {
  double dx = toX - fromX;
  double dy = toY - fromY;
  return sqrt(dx*dx + dy*dy);
}

// Returns 0 when b + a = c, and up to 180 when c reduces from there
double cosineAngle(double a, double b, double c) {
  double cosA = (b*b + c*c - a*a) / (2.0 * b * c);
  if (cosA > 1.0) cosA = 1.0;
  if (cosA < -1.0) cosA = -1.0;
  return acos(cosA) * 180.0 / M_PI;
}

double mapf(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double normalizeHeading(double heading) {
  heading = fmod(heading, 360.0);
  if (heading < 0.0) heading += 360.0;
  if (heading > 180.0) heading -= 360.0;
  return heading;
}
