#include <Arduino.h>
#include "L3G.h"
#include <Wire.h>
#include <algorithm>
#include <Wifi.h>
#include <Math.h>
#include "Coordinate.h"

#define MOTOR1_DIR 16
#define MOTOR1_PWM 17
#define MOTOR2_DIR 25
#define MOTOR2_PWM 26

#define MOTOR1_CHANNEL 0
#define MOTOR2_CHANNEL 1

#define MOTOR1_ENC_YELLOW  4
#define MOTOR1_ENC_WHITE   0
#define MOTOR2_ENC_YELLOW 34
#define MOTOR2_ENC_WHITE  35

#define GYRO_SDA 21
#define GYRO_SCL 22

#define PWM_RES 13
#define PWM_FREQ 1000

#define ALPHA 0.8

#define TICKS_PER_ROT 3200 // Pololu website 50:1 37Dx54L mm with 64 CPR encoder
#define WHEEL_CIRC 15.708 // 5" diameter wheels?
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

double gyroGain = 0.008679; // gyros gain factor for 250deg/se
double gyroZRawOld = 0.0;
double gyroZAccel = 0.0; //gyro x val
double gyroZVel = 0.0; //gyro cummulative z value
double gyroZPos = 0.0;
double gyroZAccelOld = 0.0;
double gyroZVelOld = 0.0; //gyro cummulative z value
double gyroZPosOld = 0.0;
double gyroErr; // Gyro 7 error
double minErr = 0.005;

boolean gyroGood;

double Kpr = 1; // Proportional control for rotating
double Kps = 1; // Proportional control for strait moving

double robotAngle = 0.0;      // Angle needed to turn
double currentAngle = 0.0;    // Current angle robot has turned
double distanceToNext = 0.0;  // Distance next node is from node
double currentDistance = 0.0; // Current distance robot has traveled from node

double toDegrees(double rad) {
    return rad * M_PI * 180.0;
}

void IRAM_ATTR onTimer() {
    timerCount++;
}

void motor1WhiteISR() {
    if (digitalRead(MOTOR1_ENC_YELLOW)) {
        portENTER_CRITICAL_ISR(&motor1Mux);
        motor1Rot++;
        portEXIT_CRITICAL_ISR(&motor1Mux);
    } else {
        portENTER_CRITICAL_ISR(&motor1Mux);
        motor1Rot--;
        portEXIT_CRITICAL_ISR(&motor1Mux);
    }
}

void motor1YellowISR() {
    if (digitalRead(MOTOR1_ENC_WHITE)) {
        portENTER_CRITICAL_ISR(&motor1Mux);
        motor1Rot--;
        portEXIT_CRITICAL_ISR(&motor1Mux);
    } else {
        portENTER_CRITICAL_ISR(&motor1Mux);
        motor1Rot++;
        portEXIT_CRITICAL_ISR(&motor1Mux);
    }
}

void motor2WhiteISR() {
    if (digitalRead(MOTOR2_ENC_YELLOW)) {
        portENTER_CRITICAL_ISR(&motor2Mux);
        motor2Rot++;
        portEXIT_CRITICAL_ISR(&motor2Mux);
    } else {
        portENTER_CRITICAL_ISR(&motor2Mux);
        motor2Rot--;
        portEXIT_CRITICAL_ISR(&motor2Mux);
    }
}

void motor2YellowISR() {
    if (digitalRead(MOTOR2_ENC_WHITE)) {
        portENTER_CRITICAL_ISR(&motor2Mux);
        motor2Rot--;
        portEXIT_CRITICAL_ISR(&motor2Mux);
    } else {
        portENTER_CRITICAL_ISR(&motor2Mux);
        motor2Rot++;
        portEXIT_CRITICAL_ISR(&motor2Mux);
    }
}

double calcDist() {
    portENTER_CRITICAL(&motor1Mux);
    long localMotor1Rot = motor1Rot;
    portEXIT_CRITICAL(&motor1Mux);

    portENTER_CRITICAL(&motor2Mux);
    long localMotor2Rot = motor2Rot;
    portEXIT_CRITICAL(&motor2Mux);

    long averageRot = localMotor1Rot/2 + localMotor2Rot/2;

    return (averageRot / TICKS_PER_ROT) * WHEEL_CIRC; // might divide by 12
}

//initializes the gyro
bool gyroSetup(L3G gyro) {
    delay(50);
    gyro.enableDefault(); // gyro init. default 250/deg/s
    delay(700);// allow time for gyro to settle
    for (int i = 0; i < 100; i++) { // takes 100 samples of the gyro
        gyro.read();
        gyroErr += gyro.g.z;
        delay(25);
    }
    gyroErr = gyroErr / 100;
    Serial.print("GyroErr: ");
    Serial.println(gyroErr);
    minErr = gyroErr*1.4;
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
    gyroZAccel = (double)((gyro.g.z*ALPHA + (1-ALPHA)*gyroZRawOld) - gyroErr) * gyroGain*0.75;
    gyroZVel = gyroZAccel * G_Dt;
    if (abs(gyro.g.z) > minErr) {
        gyroZVel += gyroZPos;
        gyroZPos = gyroZVel;
    }
    gyroZRawOld = gyro.g.z;
    // }
}

boolean turn(int angle) {
    gyroRead(gyro);

    if (gyroZAccel< angle) {
        robotDrive(153, -153);
        return false;
    }
    robotDrive(0, 0);

    //reset the gyro
    //  curLTicks = lEncode;
    //  curRTicks = rEncode;
    //  tickLDiff = 0;
    //  tickRDiff = 0;
    gyroGood = false;
    gyroZAccel= 0;
    gyroZVel = 0;
    return true;
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
    robotDrive(speed,speed);
}

void straightRobot(double speed) {
    robotDrive(speed,-speed);
}

double findNextAngle(Coordinate node1, Coordinate node2) {
    return toDegrees(atan2((node2.y-node1.y),(node2.x-node1.x)));
}

double findNextDistance(Coordinate node1, Coordinate node2) {
    return hypot((node1.x-node2.x),(node1.y-node2.y));
}

void setup() {
    while(millis()<3000);
    Serial.begin(115200);
    Wire.begin();
    //Connect to the WiFi network
    // WiFi.softAP(ssid, pass);

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
}

void loop() {
    unsigned long localTime = timerCount;


    // Every 20ms (50Hz)
    if (localTime % 4) {
        if (gyroZPos < 90) {
            rotateRobot(100);
        } else {
            Serial.print(" success! ");
            straightRobot(0);
        }

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
        Serial.print(" gyrozPos: ");
        Serial.println(gyroZPos);
        prevTime = localTime;
    }
}
