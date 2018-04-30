#include <Arduino.h>
#include "L3G.h"
#include <Wire.h>
#include <algorithm>
#include <Wifi.h>
#include <Math.h>
#include "Coordinate.h"
#include "PID.h"

#define MOTOR1_DIR 16
#define MOTOR1_PWM 17
#define MOTOR2_DIR 25
#define MOTOR2_PWM 26

#define MOTOR1_CHANNEL 0
#define MOTOR2_CHANNEL 1

#define MOTOR1_ENC_YELLOW  4
#define MOTOR1_ENC_WHITE   2
#define MOTOR2_ENC_YELLOW 18
#define MOTOR2_ENC_WHITE  19

#define GYRO_SDA 21
#define GYRO_SCL 22

#define PWM_RES 13
#define PWM_FREQ 1000

#define ALPHA 0.8

#define TICKS_PER_ROT 3200.0 // Pololu website 50:1 37Dx54L mm with 64 CPR encoder
#define WHEEL1_CIRC 10.86    // 3.458" diameter wheel (RIGHT)
#define WHEEL2_CIRC 10.95    // 3.486" diameter wheel (LEFT)
#define WHEEL_RATIO 0.992    // 10.86 / 10.95

//Uncomment for debug information
//#define debugPrints

const char* ssid = "esp32";
const char* pass = "softengiscool";

// IP address to send UDP data to.
// it can be ip address of the server or
// a network broadcast address
// here is broadcast address
const char * udpAddress = "192.168.4.2";
const int udpPort = 44444;

//create UDP instance
WiFiUDP udp;
int received = 0;

Coordinate path[30];
int pathSize = 0;
int pathIndex = 0;

enum states {
    WAITING,
    ROTATE_TO_NODE,
    STRAIGHT_TO_NODE,
    CALCULATE_NEXT_MOVE,
    RETURN_HOME
} state;

void robotDrive(int motor1, int motor2);
void motorDrive(int motor, int spd);

hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE motor1Mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE motor2Mux = portMUX_INITIALIZER_UNLOCKED;

L3G gyro;

int brightness = 0;
int fadeAmount = 5;

volatile long motor1Rot = 0;
volatile long motor2Rot = 0;
volatile SemaphoreHandle_t timerSemaphore;
volatile unsigned long timerCount = 0;
unsigned long prevTime = 0;

double G_Dt = 0.005;  // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

long gyroTimer = 0; //general purpose timer
long gyroTimer1 = 0;

double gyroGain = 0.008699; // gyros gain factor for 250deg/se
double gyroZRawOld = 0.0;
double gyroZAccel = 0.0; //gyro x val
double gyroZVel = 0.0; //gyro cummulative z value
double gyroZPos = 0.0;
double gyroZAccelOld = 0.0;
double gyroZVelOld = 0.0; //gyro cummulative z value
double gyroZPosOld = 0.0;
double gyroErr; // Gyro 7 error
double minErr = 0.005;
double gyroScale = 1.06;

boolean gyroGood;

PID rotatePID;
double Kpr = 10; // Proportional control for rotating
double Kir = 1;
double Kdr = 3;

PID straightPID;
double Kps = 1; // Proportional control for strait moving
double Kis = 0;
double Kds = 0;

double robotAngle = 0.0;      // Angle needed to turn
double currentAngle = 0.0;    // Current angle robot has turned
double distanceToNext = 0.0;  // Distance next node is from node
double currentDistance = 0.0; // Current distance robot has traveled from node
double prevDistance = 0.0;
int destX = 0.0;
int destY = 0.0;
double currX = 0.0;
double currY = 0.0;
double prevX = 0.0;
double prevY = 0.0;

Coordinate curr;
Coordinate next;

int var = 0;

double toDegrees(double rad) {
    return rad * M_PI * 180.0;
}

double toRadians(double deg) {
    return deg / 180.0 / M_PI;
}

void IRAM_ATTR onTimer() {
    timerCount++;
}

void motor1WhiteISR() {
    if (digitalRead(MOTOR1_ENC_YELLOW)) {
        motor1Rot--;
    } else {
        motor1Rot++;
    }
}

void motor1YellowISR() {
    if (digitalRead(MOTOR1_ENC_WHITE)) {
        motor1Rot++;
    } else {
        motor1Rot--;
    }
}

void motor2WhiteISR() {
    if (digitalRead(MOTOR2_ENC_YELLOW)) {
        motor2Rot++;
    } else {
        motor2Rot--;
    }
}

void motor2YellowISR() {
    if (digitalRead(MOTOR2_ENC_WHITE)) {
        motor2Rot--;
    } else {
        motor2Rot++;
    }
}

double calcDist() {
    long localMotor1Rot = motor1Rot;

    long localMotor2Rot = motor2Rot;

    double averageRot = (localMotor1Rot/2.0) * WHEEL1_CIRC + (localMotor2Rot/2.0) * WHEEL2_CIRC;
    // Serial.print(" averageRot: ");
    // Serial.println(averageRot / TICKS_PER_ROT);
    return (averageRot / TICKS_PER_ROT); // might divide by 12
}

double calcDisplacement() {
    return hypot((currX - prevX), (currY - prevY));
}

//initializes the gyro
bool gyroSetup(L3G gyro) {
    delay(50);
    gyro.enableDefault(); // gyro init. default 250/deg/sx
    delay(700);// allow time for gyro to settle
    while (gyro.last_status != 0) {
        gyro.read();
        Serial.println("Waiting for gyro to realize it functions");
    }
    for (int i = 0; i < 100; i++) { // takes 100 samples of the gyro
        gyro.read();
        gyroErr += gyro.g.z;
        delay(25);
    }
    gyroErr = gyroErr / 100;
    Serial.print("GyroErr: ");
    Serial.println(gyroErr);
    minErr = gyroErr*1.45;
    return true;
}

//reads the value of the gyro
void gyroRead(L3G gyro) {
    gyro.init();
    if (!gyroGood) {
        gyroGood = gyroSetup(gyro);
    }
    // if (timerCount % 5) { // reads imu every 5ms
    // Serial.print("gyro.g.z: ");
    // Serial.print(gyro.g.z);
    // Serial.print(" gyroZAccel: ");
    // Serial.print(gyroZAccel);
    // Serial.print(" gyroZVel: ");
    // Serial.print(gyroZVel);
    gyro.read(); // read gyro
    gyroZAccel = (double)((gyro.g.z*ALPHA + (1-ALPHA)*gyroZRawOld) - gyroErr) * gyroGain*gyroScale;
    gyroZVel = gyroZAccel * G_Dt;
    if (abs(gyro.g.z) > minErr) {
        gyroZVel += gyroZPos;
        gyroZPos = gyroZVel;
    }
    gyroZRawOld = gyro.g.z;
    // }
}

void locationUpdate() {
    currentDistance = calcDisplacement();
    double dist = currentDistance - prevDistance;
    currX += dist*cos(toRadians(gyroZPos)) - dist*sin(toRadians(gyroZPos));
    // currY += dist*sin(toRadians(gyroZPos)) + dist*cos(toRadians(gyroZPos));
    prevDistance = currentDistance;
    Serial.print("motor1Rot: ");
    Serial.print(motor1Rot);
    Serial.print(" motor2Rot: ");
    Serial.print(motor2Rot);
    Serial.print(" curDist: ");
    Serial.print(currentDistance);
    Serial.print(" preDist: ");
    Serial.print(prevDistance);
    Serial.print(" dist: ");
    Serial.println(dist);
}

void robotDrive(int motor1, int motor2) {
    motorDrive(0, motor1);
    motorDrive(1, motor2);
}

void pwmOut(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
    // calculate duty, 8191 from 2 ^ 13 - 1
    uint32_t duty = (8191 / valueMax) * std::min(value, valueMax);

    // write duty to LEDC
    ledcWrite(channel, duty);
}

void motorDrive(int motor, int spd) {
    if (spd > 255) spd = 255;
    if (spd < -255) spd = -255;

    boolean dir = true;
    if (spd < 0) {
        dir = false;
        spd = -spd;
    }
    motor = motor % 2;
    switch (motor) {
        case 0:
        digitalWrite(MOTOR1_DIR, dir);
        pwmOut(MOTOR1_CHANNEL, spd);
        break;

        case 1:
        digitalWrite(MOTOR2_DIR, dir);
        pwmOut(MOTOR2_CHANNEL, spd);
        break;
    }
}

void rotateRobot(double speed) {
    robotDrive(speed,speed*WHEEL_RATIO);
}

void straightRobot(double speed) {
    robotDrive(speed,-speed*WHEEL_RATIO);
}

double findNextAngle(Coordinate node1, Coordinate node2) {
    return toDegrees(atan2((node2.y-node1.y),(node2.x-node1.x)));
}

double findNextDistance(Coordinate node1, Coordinate node2) {
    destX = node2.x;
    destY = node2.y;
    return hypot((node1.x-node2.x),(node1.y-node2.y));
}

double driveToNext() {
    distanceToNext = findNextDistance(curr, next);
    prevX = currX;
    prevY = currY;
    double error = straightPID.calc(distanceToNext, currentDistance);
    straightRobot(error);
    // Serial.print("Distance: ");
    // Serial.println(distanceToNext);

    return straightPID.sumError;
}

double rotateToAngle(double toAngle) {
    robotAngle = toAngle;
    currentAngle = gyroZPos;
    while (robotAngle > 180) robotAngle -= 360;
    while (robotAngle < -180) robotAngle += 360;

    while (currentAngle > 180) currentAngle -= 360;
    while (currentAngle < -180) currentAngle += 360;

    double error = rotatePID.calc(robotAngle, currentAngle);
    rotateRobot(error);
    return rotatePID.sumError;
}

void setup() {
    while(millis()<2000);
    Serial.begin(115200);
    Wire.reset();
    Wire.begin();

    rotatePID.setpid(Kpr, Kir, Kdr, 50, 255);
    straightPID.setpid(Kps, Kis, Kds, 200, 255);

    //Connect to the WiFi network
    // WiFi.softAP(ssid, pass);

    curr.init(0,0,0);
    next.init(5,0,0);

    Serial.println("Initialization!");
    // Serial.print("Connected to ");
    // Serial.println(ssid);
    // Serial.print("IP address: ");
    // Serial.println(WiFi.localIP());
    //This initializes udp and transfer buffer
    // udp.begin(udpPort);

    pinMode(MOTOR1_DIR, OUTPUT);
    pinMode(MOTOR2_DIR, OUTPUT);

    pinMode(MOTOR1_ENC_YELLOW, INPUT);
    pinMode(MOTOR1_ENC_WHITE, INPUT);
    pinMode(MOTOR2_ENC_YELLOW, INPUT);
    pinMode(MOTOR2_ENC_WHITE, INPUT);

    attachInterrupt(digitalPinToInterrupt(MOTOR1_ENC_YELLOW), motor1YellowISR, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR1_ENC_WHITE), motor1WhiteISR, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR2_ENC_YELLOW), motor2YellowISR, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR2_ENC_WHITE), motor2WhiteISR, RISING);

    ledcSetup(MOTOR1_CHANNEL, PWM_FREQ, PWM_RES);
    ledcSetup(MOTOR2_CHANNEL, PWM_FREQ, PWM_RES);

    ledcAttachPin(MOTOR1_PWM, MOTOR1_CHANNEL);
    ledcAttachPin(MOTOR2_PWM, MOTOR2_CHANNEL);

    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 5000, true);
    timerAlarmEnable(timer);

    gyro.init();
    gyroGood = gyroSetup(gyro);

    Serial.println("Done initializing!");
    // findNextDistance(curr, next);
}

void loop() {
    unsigned long localTime = timerCount;

    // Every 20ms (50Hz)
    if (localTime % 4) {
        // Serial.println(var);
        double e;
        // switch(var) {
        //     case 0:
        //     e = rotateToAngle(90);
        //     if (e < 0.1) {
        //         var = 1;
        //         delay(100);
        //     }
        //     break;
        //
        //     case 1:
        //     e = driveDistance(5);
        //     if (e < 0.1) {
        //         var = 0;
        //     }
        //     break;
        // }
        // Serial.println("Rotating!");
        // rotateToAngle(90);
        driveToNext();

        // switch (state) {
        //     case WAITING:
        //     uint8_t buffer[50];
        //     memset(buffer, 0, 50);
        //     //processing incoming packet, must be called before reading the buffer
        //     udp.parsePacket();
        //     // udp.read(buffer, 50)
        //     while (udp.available()) {
        //         char next = udp.read();
        //         if (next == 'E') {
        //             state = CALCULATE_NEXT_MOVE;
        //         }
        //     }
        //     break;
        //
        //     case ROTATE_TO_NODE:
        //     double angleDiff;
        //     angleDiff = robotAngle - currentAngle;
        //     if (!(abs(angleDiff) < 0.1)) {
        //         rotateRobot(angleDiff*Kpr);
        //     } else {
        //         state = STRAIGHT_TO_NODE;
        //     }
        //     break;
        //
        //     case STRAIGHT_TO_NODE:
        //     double distanceDiff;
        //     distanceDiff = distanceToNext - currentDistance;
        //     if (!(abs(distanceDiff) < 1.0)) {
        //         straightRobot(distanceDiff*Kps);
        //     } else {
        //         state = CALCULATE_NEXT_MOVE;
        //         pathIndex++;
        //     }
        //     break;
        //
        //     case CALCULATE_NEXT_MOVE:
        //     robotAngle = findNextAngle(path[pathIndex], path[pathIndex+1]);
        //     distanceToNext = findNextDistance(path[pathIndex], path[pathIndex+1]);
        //     state = ROTATE_TO_NODE;
        //     break;
        //
        //     default:
        //     break;
        // }
    }

    if (localTime != prevTime) {

        // Serial.println("Reading gyro!");
        // Serial.print("gyro: ");
        // Serial.print(gyro_z);
        // Serial.print(", Brightness: ");
        // Serial.print(brightness);
        // Serial.print(", motor1Rot: ");
        // Serial.print(motor1Rot);
        // Serial.print(", motor2Rot: ");
        // Serial.println(motor2Rot);
        gyroRead(gyro);
        locationUpdate();
        // Serial.print(" gyrozPos: ");
        // Serial.println(gyroZPos);
        prevTime = localTime;
    }
}
