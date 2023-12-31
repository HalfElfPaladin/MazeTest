#include <VL53L0XsAnalog.h>
#include <IMUBoschBNO055.h>
#include <ReflectanceSensors.h>
#include <Motors.h>
#include <Encoders.h>
#include <Displays.h>
#include <IRDistanceSensors.h>
#include <Commands.h>
#include <Arduino.h>
#include <Wire.h>
#include <mrm-col.h>
//#include <i2c_t3.h>
#include "Maze.h"


VL53L0XsAnalog lidars;
Maze maze;
Robot robot;
Motors motors;
IMUBoschBNO055 compass;
Mrm_col col; 
Encoders encoders;
//ReflectanceSensors RefSen;
const int led1=35;
const int led2=36;
const int led3=37;
const int tip1=31;
const int tip2=32;
const int dgIRL=13;
const int dgIRD=12;
const int IRS=38;
const int IRL=39;
const int termL=40;
const int termD=41;
const int imuRst=34;
const int colRst=33;

int state_tip1=0;
int state_tip2=0;

int csb=0;

float startPitch=0;

void setup() {
	robot.setupHardware();
}

void loop() {
    robot.StartStop ();
    robot.Init();
    robot.program();
}


bool Robot::setupHardware() {
    //I2C start
    //	Wire.begin(I2C_MASTER, 0, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_100);
    Wire.begin();
	Wire.setTimeout(120);

    Serial.begin(115200);
    lidars.add(A1);
    lidars.add(A2);
    lidars.add(A3);
    lidars.add(A13);
    lidars.add(A12);
    lidars.add(A11);
    lidars.add(A10);
    lidars.add(A9);
    lidars.add(A8);
    lidars.add(A7);
    lidars.add(A6);
    motors.add(6,7,true,true);
    motors.add(5,4,true,true);
    motors.add(3,2,false,true);
    motors.add(0,1,false,true);
	encoders.add(9);//Left
	encoders.add(11);//Right
    pM (led1,OUTPUT);
    pM (led2,OUTPUT);
    pM (led3,OUTPUT);
    pM (tip1,INPUT);
    pM (tip2,INPUT);
    pM (dgIRL,INPUT);
    pM (dgIRD,INPUT);
    pM (IRS,INPUT);
    pM (IRL,INPUT);
    pM (termL,INPUT);
    pM (termD,INPUT);
    pM (imuRst,OUTPUT);
    pM (colRst,OUTPUT);
    compass.add(true);
    delay(10);
    startPitch=compass.pitch();
    col.add();
    col.ledForIlluminationSet(0, true);
    dW(colRst,HIGH);
    delay(50);
    dW(colRst,LOW);
    
    return true;
}

void testLidarsRead () {
    float dsFL=0, dsFM=0, dsFR=0, dsLF=0, dsLM=0, dsLB=0, dsRF=0, dsRM=0, dsRB=0, dsBL=0, dsBR=0;

    dsFL = lidars.distance(0);
    dsFM = lidars.distance(1);
    dsFR = lidars.distance(2);
    dsLF = lidars.distance(10);
    dsLM = lidars.distance(9);
    dsLB = lidars.distance(8);
    dsRF = lidars.distance(3);
    dsRM = lidars.distance(4);
    dsRB = lidars.distance(5);
    dsBL = lidars.distance(7);
    dsBR = lidars.distance(6);

    Serial.print ("Front   ");
    Serial.print(dsFL);
    Serial.print (" ");
    Serial.print(dsFM);
    Serial.print (" ");
    Serial.println(dsFR);
    Serial.print ("Left    ");
    Serial.print(dsLF);
    Serial.print (" ");
    Serial.print(dsLM);
    Serial.print (" ");
    Serial.println(dsLB);
    Serial.print ("Right   ");
    Serial.print(dsRF);
    Serial.print (" ");
    Serial.print(dsRM);
    Serial.print (" ");
    Serial.println(dsRB);
    Serial.print ("Back    ");
    Serial.print(dsBL);
    Serial.print (" ");
    Serial.println(dsBR);
    Serial.println(" ");
    robot.pauza(1000);
    if (state_tip1 == 0)
        return;
}

void testLidarsRead10 () {
    float dsFL=0, dsFM=0, dsFR=0, dsLF=0, dsLM=0, dsLB=0, dsRF=0, dsRM=0, dsRB=0, dsBL=0, dsBR=0;
    int i, n = 20;

    for(i=0;i<n;i++){
        dsFL = dsFL + lidars.distance(0);
        dsFM = dsFM + lidars.distance(1);
        dsFR = dsFR + lidars.distance(2);
        dsLF = dsLF + lidars.distance(10);
        dsLM = dsLM + lidars.distance(9);
        dsLB = dsLB + lidars.distance(8);
        dsRF = dsRF + lidars.distance(3);
        dsRM = dsRM + lidars.distance(4);
        dsRB = dsRB + lidars.distance(5);
        dsBL = dsBL + lidars.distance(7);
        dsBR = dsBR + lidars.distance(6);
        delay(1);
    }

    dsFL = dsFL / n;
    dsFM = dsFM / n;
    dsFR = dsFR / n;
    dsLF = dsLF / n;
    dsLM = dsLM / n;
    dsLB = dsLB / n;
    dsRF = dsRF / n;
    dsRM = dsRM / n;
    dsRB = dsRB / n;
    dsBL = dsBL / n;
    dsBR = dsBR / n;

    Serial.print ("Front   ");
    Serial.print(dsFL);
    Serial.print (" ");
    Serial.print(dsFM);
    Serial.print (" ");
    Serial.println(dsFR);
    Serial.print ("Left    ");
    Serial.print(dsLF);
    Serial.print (" ");
    Serial.print(dsLM);
    Serial.print (" ");
    Serial.println(dsLB);
    Serial.print ("Right   ");
    Serial.print(dsRF);
    Serial.print (" ");
    Serial.print(dsRM);
    Serial.print (" ");
    Serial.println(dsRB);
    Serial.print ("Back    ");
    Serial.print(dsBL);
    Serial.print (" ");
    Serial.println(dsBR);
    Serial.println(" ");
    robot.pauza(1000);
    if (state_tip1 == 0)
        return;
}

void testLidars () {
    Serial.print ("Front   ");
    Serial.print(robot.distanceFrontLeft());
    Serial.print (" ");
    Serial.print(robot.distanceFrontMiddle());
    Serial.print (" ");
    Serial.println(robot.distanceFrontRight());
    Serial.print ("Left    ");
    Serial.print(robot.distanceLeftFront());
    Serial.print (" ");
    Serial.print(robot.distanceLeftMiddle());
    Serial.print (" ");
    Serial.println(robot.distanceLeftBack());
    Serial.print ("Right   ");
    Serial.print(robot.distanceRightFront());
    Serial.print (" ");
    Serial.print(robot.distanceRightMiddle());
    Serial.print (" ");
    Serial.println(robot.distanceRightBack());
    Serial.print ("Back    ");
    Serial.print(robot.distanceBackLeft());
    Serial.print (" ");
    Serial.println(robot.distanceBackRight());
    Serial.println(" ");
    robot.pauza(1000);
    if (state_tip1 == 0)
        return;
}

void testMotors () {
    motors.go(0,0);
    robot.pauza(500);
    if (state_tip1 == 0)
        return;
    motors.go(70,0);
    robot.pauza(2000);
    if (state_tip1 == 0)
        return;
    motors.go(0,0);
    robot.pauza(500);
    if (state_tip1 == 0)
        return;
    motors.go(-70,0);
    robot.pauza(2000);
    if (state_tip1 == 0)
        return;
    motors.go(0,0);
    robot.pauza(500);
    if (state_tip1 == 0)
        return;
    motors.go(70,70);
    robot.pauza(2000);
    if (state_tip1 == 0)
        return;
}

void testTipLed () {
    Serial.print(dR(tip1));
    Serial.print(" ");
    Serial.println(dR(tip2));
    Serial.println(" ");
    dW(led1,HIGH);
    dW(led2,HIGH);
    dW(led3,HIGH);
    robot.pauza(1000);
    if (state_tip1 == 0)
        return;
    dW(led1,LOW);
    dW(led2,LOW);
    dW(led3,LOW);
    robot.pauza(1000);
    if (state_tip1 == 0)
        return;
}

void testIRDistSensors () {
    Serial.print("IR distance sensors     ");
    Serial.print(dR(dgIRL));
    Serial.print(" ");
    Serial.println(dR(dgIRD));
    Serial.println(" ");
    robot.pauza(1000);
    if (state_tip1 == 0)
        return;
}

void testIRLineSensors () {
    Serial.print("IR line sensors     ");
    Serial.print (aR(IRS));
    Serial.print (" ");
    Serial.println (aR(IRL));
    Serial.println ("  ");
    robot.pauza(1000);
    if (state_tip1 == 0)
        return;
}

void testThermal () {
    Serial.print("Termal sensors     ");
    Serial.print (aR(termL));
    Serial.print (" ");
    Serial.println (aR(termD));
    Serial.println(" ");
    robot.pauza(1000);
    if (state_tip1 == 0)
        return;
}

void testCompass () {
    Serial.print("Compass h:");
    Serial.print(compass.heading());
    Serial.print(" p:");
    Serial.println(compass.pitch());
    robot.pauza(1000);

    if (state_tip1 == 0)
        return;
}

void testColor () {
    Serial.print("Blue   ");
    Serial.println(col.blueCalibrated(0));
    dW(colRst,HIGH);
    delay(50);
    dW(colRst,LOW);
    Serial.print("Green  ");
    Serial.println(col.greenCalibrated(0));
    dW(colRst,HIGH);
    delay(50);
    dW(colRst,LOW);
    Serial.print("Red    ");
    Serial.println(col.redCalibrated(0));
    dW(colRst,HIGH);
    delay(50);
    dW(colRst,LOW);
    Serial.print("Yellow ");
    Serial.println(col.yellowCalibrated(0));
    dW(colRst,HIGH);
    delay(50);
    dW(colRst,LOW);
    Serial.print("Violet ");
    Serial.println(col.violetCalibrated(0));
    dW(colRst,HIGH);
    delay(50);
    dW(colRst,LOW);
    Serial.print("Orange ");
    Serial.println(col.orangeCalibrated(0));
    dW(colRst,HIGH);
    delay(50);
    dW(colRst,LOW);
    Serial.println(" ");
    robot.pauza(1000);
    if (state_tip1 == 0)
        return;
}

void testEncoders () {
    motors.go(0,0);
    Serial.print("L-");    
    Serial.print(encoders.counter(0));    
    Serial.print("   R-");    
    Serial.println(encoders.counter(1));    
    robot.pauza(1000);
    if (state_tip1 == 0)
        return;
    motors.go(70,0);
    Serial.print("L-");    
    Serial.print(encoders.counter(0));    
    Serial.print("   R-");    
    Serial.println(encoders.counter(1));    
    robot.pauza(1000);
    if (state_tip1 == 0)
        return;
    motors.go(0,70);
    Serial.print("L-");    
    Serial.print(encoders.counter(0));    
    Serial.print("   R-");    
    Serial.println(encoders.counter(1));    
    robot.pauza(1000);
    if (state_tip1 == 0)
        return;
    motors.go(70,70);
    Serial.print("L-");    
    Serial.print(encoders.counter(0));    
    Serial.print("   R-");    
    Serial.println(encoders.counter(1));    
    Serial.println(" ");
    robot.pauza(1000);
    if (state_tip1 == 0)
        return;
}

void Robot::testSensors() {
    testLidars();
    testLidarsRead10();
    testIRDistSensors();
    testIRLineSensors();
    testMotors();
    testEncoders();
    testCompass();
    testColor();
    testThermal();
    testTipLed();
    Serial.println(" ");
    Serial.println("Everythig is OK!");
    Serial.println(" ");
    delay(1000);
}

float Robot::distanceFrontLeft() {
    float lidarValue = lidars.distance(0) + LIDAR_READ_CORRECT;
    float retValue;
    if (lidarValue < FRONT_LEFT_LB){ 
        retValue = ((lidarValue * FRONT_LEFT_ANG1) + FRONT_LEFT_MOV1) / 10.0;
    }
    else {
        retValue = ((lidarValue * FRONT_LEFT_ANG2) + FRONT_LEFT_MOV2) / 10.0;
    };
    return retValue;
}

float Robot::distanceFrontMiddle() {
    float lidarValue = lidars.distance(1) + LIDAR_READ_CORRECT;
    float retValue;
    if (lidarValue < FRONT_MIDDLE_LB){ 
        retValue = ((lidarValue * FRONT_MIDDLE_ANG1) + FRONT_MIDDLE_MOV1) / 10.0;
    }
    else {
        retValue = ((lidarValue * FRONT_MIDDLE_ANG2) + FRONT_MIDDLE_MOV2) / 10.0;
    };
    return retValue;
}

float Robot::distanceFrontRight() {
    float lidarValue = lidars.distance(2) + LIDAR_READ_CORRECT;
    float retValue;
    if (lidarValue < FRONT_RIGHT_LB){ 
        retValue = ((lidarValue * FRONT_RIGHT_ANG1) + FRONT_RIGHT_MOV1) / 10.0;
    }
    else {
        retValue = ((lidarValue * FRONT_RIGHT_ANG2) + FRONT_RIGHT_MOV2) / 10.0;
    };
    return retValue;
}

float Robot::distanceRightFront() {
    float lidarValue = lidars.distance(3) + LIDAR_READ_CORRECT;
    float retValue;
    if (lidarValue < RIGHT_FRONT_LB){ 
        retValue = ((lidarValue * RIGHT_FRONT_ANG1) + RIGHT_FRONT_MOV1) / 10.0;
    }
    else {
        retValue = ((lidarValue * RIGHT_FRONT_ANG2) + RIGHT_FRONT_MOV2) / 10.0;
    };
    return retValue;
}

float Robot::distanceRightMiddle() {
    float lidarValue = lidars.distance(4) + LIDAR_READ_CORRECT;
    float retValue;
    if (lidarValue < RIGHT_MIDDLE_LB){ 
        retValue = ((lidarValue * RIGHT_MIDDLE_ANG1) + RIGHT_MIDDLE_MOV1) / 10.0;
    }
    else {
        retValue = ((lidarValue * RIGHT_MIDDLE_ANG2) + RIGHT_MIDDLE_MOV2) / 10.0;
    };
    return retValue;
}

float Robot::distanceRightBack() {
    float lidarValue = lidars.distance(5) + LIDAR_READ_CORRECT;
    float retValue;
    if (lidarValue < RIGHT_BACK_LB){ 
        retValue = ((lidarValue * RIGHT_BACK_ANG1) + RIGHT_BACK_MOV1) / 10.0;
    }
    else {
        retValue = ((lidarValue * RIGHT_BACK_ANG2) + RIGHT_BACK_MOV2) / 10.0;
    };
    return retValue;
}

float Robot::distanceBackRight() {
    float lidarValue = lidars.distance(6) + LIDAR_READ_CORRECT;
    float retValue;
    if (lidarValue < BACK_RIGHT_LB){ 
        retValue = ((lidarValue * BACK_RIGHT_ANG1) + BACK_RIGHT_MOV1) / 10.0;
    }
    else {
        retValue = ((lidarValue * BACK_RIGHT_ANG2) + BACK_RIGHT_MOV2) / 10.0;
    };
    return retValue;
}

float Robot::distanceBackLeft() {
    float lidarValue = lidars.distance(7) + LIDAR_READ_CORRECT;
    float retValue;
    if (lidarValue < BACK_LEFT_LB){ 
        retValue = ((lidarValue * BACK_LEFT_ANG1) + BACK_LEFT_MOV1) / 10.0;
    }
    else {
        retValue = ((lidarValue * BACK_LEFT_ANG2) + BACK_LEFT_MOV2) / 10.0;
    };
    return retValue;
}

float Robot::distanceLeftFront() {
    float lidarValue = lidars.distance(10) + LIDAR_READ_CORRECT;
    float retValue;
    if (lidarValue < LEFT_FRONT_LB){ 
        retValue = ((lidarValue * LEFT_FRONT_ANG1) + LEFT_FRONT_MOV1) / 10.0;
    }
    else {
        retValue = ((lidarValue * LEFT_FRONT_ANG2) + LEFT_FRONT_MOV2) / 10.0;
    };
    return retValue;
}

float Robot::distanceLeftMiddle() {
    float lidarValue = lidars.distance(9) + LIDAR_READ_CORRECT;
    float retValue;
    if (lidarValue < LEFT_MIDDLE_LB){ 
        retValue = ((lidarValue * LEFT_MIDDLE_ANG1) + LEFT_MIDDLE_MOV1) / 10.0;
    }
    else {
        retValue = ((lidarValue * LEFT_MIDDLE_ANG2) + LEFT_MIDDLE_MOV2) / 10.0;
    };
    return retValue;
}

float Robot::distanceLeftBack() {
    float lidarValue = lidars.distance(8) + LIDAR_READ_CORRECT;
    float retValue;
    if (lidarValue < LEFT_BACK_LB){ 
        retValue = ((lidarValue * LEFT_BACK_ANG1) + LEFT_BACK_MOV1) / 10.0;
    }
    else {
        retValue = ((lidarValue * LEFT_BACK_ANG2) + LEFT_BACK_MOV2) / 10.0;
    };
    return retValue;
}


float Robot::colorRed() {
    return (col.redCalibrated(0));
}

float Robot::colorOrange() {
    return (col.orangeCalibrated(0));
}

float Robot::colorYellow() {
    return (col.yellowCalibrated(0));
}
float Robot::colorGreen() {
    return (col.greenCalibrated(0));
}

float Robot::colorBlue() {
    return (col.blueCalibrated(0));
}

float Robot::colorViolet() {
    return (col.violetCalibrated(0));
}


float Robot::compassHeading(){
    this->pauza (10);
    return compass.heading();
    
}

float Robot::currentTemperatureL(){
    return  (aR(termL));
}

float Robot::currentTemperatureR(){
    return  (aR(termD));
}

float Robot::compassPitch(){
    this->pauza(10);
    return compass.pitch();
}

Orientation Robot::getRobotOrientation() {
    return this->robotOrientation;
}


void Robot::setRobotOrientation(Orientation newOrientation) {
    this->robotOrientation = newOrientation;
}

void Robot::changeRobotOrientation(bool isLeft = false) {
    Orientation currentOrientation = this->getRobotOrientation();

    if (isLeft == false) {
        switch (currentOrientation) {
            case ORIENTATION_UP:
                this->setRobotOrientation(ORIENTATION_RIGHT);
                break;
            case ORIENTATION_RIGHT:
                this->setRobotOrientation(ORIENTATION_DOWN);
                break;
            case ORIENTATION_DOWN:
                this->setRobotOrientation(ORIENTATION_LEFT);
                break;
            case ORIENTATION_LEFT:
                this->setRobotOrientation(ORIENTATION_UP);
                break;
            default:
                Serial.println("Error: No such orientation");
                break;
        }
    } else {
        switch (currentOrientation) {
            case ORIENTATION_UP:
                this->setRobotOrientation(ORIENTATION_LEFT);
                break;
            case ORIENTATION_RIGHT:
                this->setRobotOrientation(ORIENTATION_UP);
                break;
            case ORIENTATION_DOWN:
                this->setRobotOrientation(ORIENTATION_RIGHT);
                break;
            case ORIENTATION_LEFT:
                this->setRobotOrientation(ORIENTATION_DOWN);
                break;
            default:
                Serial.println("Error: No such orientation");
                break;
        }
    }
}

void Robot::StartStop () {
    if (dR(tip1) ==1) {
        if (state_tip1 ==0) {
            state_tip1 = 1;
            state_tip2 = 0;
            delay (500);
        }
        else {
            state_tip1 = 0;
            delay (500);
        }
    }
}

void Robot::Init () {
    if (dR(tip2) ==1) {
        if (state_tip2 == 0) {
            state_tip2 = 1;
            delay (500);
        }
        else {
            state_tip2 = 0;
            delay (500);
        }
    }
}

void Robot::stop() {
    motors.go(0,0);
    dW(led1,LOW);
    dW(led2,LOW);
    dW(led3,LOW);
}

void Robot::pauza (int duljina){
    int petlja=0;
    while (petlja < duljina){
        petlja++;
        delay(1);
        StartStop();
        if (state_tip1 == 0)
            return;
    }
}

int Robot::followingType(){
    if (this->isWallRight())
        return 1;
    else if (this->isWallLeft())
        return 2;
    else
        return 3;
}

void Robot::followToNextTyle(int fType, float startCompass){
    switch (fType)
    {
    case 1 :
        this->followRightLidars(startCompass);
        break;
    case 2 :
        this->followLeftLidars(startCompass);
        break;
    case 3 :
        this->followIMU(startCompass);
        break;  
    default:
        break;
    }
}
void Robot::slopeUp(){
    while (this->compassPitch()*(-1)-startPitch>DIFFERENCE_PITCH) {
        motors.go(95*LEFT_MOTOR_FACTOR,95*RIGHT_MOTOR_FACTOR);
        this->pauza(1);
    }
}

void Robot::blackTileBack(){
    float povrat;

    motors.go (0,0);
    this->pauza(250);
    if (state_tip1 == 0)
        return;
    if (encoders.counter(1) < 150)
        povrat = encoders.counter(1) * 2.2;
    else if (encoders.counter(1) < 350)
        povrat = encoders.counter(1) * 1.95;
    else
        povrat = encoders.counter(1) * 1.7;
    while ((encoders.counter(1) < povrat) && !((this->distanceBackLeft()<=8 && this->distanceBackRight()<=8))) {
        motors.go(-LEFT_MOTOR_FACTOR * 50, -RIGHT_MOTOR_FACTOR * 50); 
    }
    motors.go (0,0);
    this->pauza(250);
    this->turn90Degrees(true);
    this->pauza(250);
    if (state_tip1 == 0)
        return;
    while (this->isWallFront()){
        this->turn90Degrees(true);
        this->pauza(250);
        if (state_tip1 == 0)
            return;
    }
    this->goAheadOneTile();
}

void moveLeft(){
    robot.turn90Degrees(true);
    robot.pauza(250);
    if (state_tip1 == 0)
        return;
    encoders.reset();
    while ((encoders.counter(0) / LEFT_MOTOR_FACTOR) < (ONE_TILE_ENCODER_STEPS/4) && ((encoders.counter(1) / RIGHT_MOTOR_FACTOR ) < (ONE_TILE_ENCODER_STEPS/4))) {
        motors.go(LEFT_MOTOR_FACTOR * 60, RIGHT_MOTOR_FACTOR * 60); 
        robot.pauza(1);
        if (state_tip1 == 0)
            return;
    }
    robot.pauza(250);
    if (state_tip1 == 0)
        return;
    robot.turn90Degrees(false);
    robot.pauza(250);
    if (state_tip1 == 0)
        return;
    encoders.reset();
}

void moveRight(){
    robot.turn90Degrees(false);
    robot.pauza(250);
    if (state_tip1 == 0)
        return;
    encoders.reset();
    while ((encoders.counter(0) / LEFT_MOTOR_FACTOR) < (ONE_TILE_ENCODER_STEPS/4) && ((encoders.counter(1) / RIGHT_MOTOR_FACTOR ) < (ONE_TILE_ENCODER_STEPS/4))) {
        motors.go(LEFT_MOTOR_FACTOR * 60, RIGHT_MOTOR_FACTOR * 60); 
        robot.pauza(1);
        if (state_tip1 == 0)
            return;
    }
    robot.pauza(250);
    if (state_tip1 == 0)
        return;
    robot.turn90Degrees(true);
    robot.pauza(250);
    if (state_tip1 == 0)
        return;
    encoders.reset();
}

bool Robot::checkHeatedVictim(){
    if ((aR(termL) > THERMAL_LEVEL) || (aR(termD) > THERMAL_LEVEL)){
        motors.go(0,0);
        this->tileSignal('B');
        return true;
    }
    else
        return false;
}

int Robot::goAheadOneTile(){
    int retVal=0;
    float frontDistanceStart = this->WallFrontDistance();
    float startingHeading = this->compassHeading();
    int folowProcedureType = this->followingType();
    int encoderState = 0;
    bool heatedVictimDetected = false;
    char TileColor;

    encoders.reset();
    if (frontDistanceStart > 50.0) {
        while ((encoders.counter(0) / LEFT_MOTOR_FACTOR) < (ONE_TILE_ENCODER_STEPS-encoderState) && ((encoders.counter(1) / RIGHT_MOTOR_FACTOR ) < (ONE_TILE_ENCODER_STEPS-encoderState))) {
            this->followToNextTyle(folowProcedureType, startingHeading);
            this->pauza(1);
            if (this->compassPitch()*(-1)-startPitch>DIFFERENCE_PITCH) {
                this->slopeUp();
                this->pauza (400);
                break;
            }
            if (state_tip1 == 0)
                return;
            if ((heatedVictimDetected == false) && (encoders.counter(0) > 200)) {
                heatedVictimDetected = this->checkHeatedVictim();
            }
            if (crnosrebrnobijelo()==1) {
                this->blackTileBack();
                break;
            }
            if (this->isWallFrontNear()) {
                break;
            }
            if (this->WallFrontDistance() < 12.0){
                if (!this->isWallFront()){
                    this->followToNextTyle(folowProcedureType, startingHeading);
                    this->pauza(1);
                    if (state_tip1 == 0)
                        return;
                }
                else{
                    encoders.reset();
                    while ((encoders.counter(0) / LEFT_MOTOR_FACTOR) < 120 && ((encoders.counter(1) / RIGHT_MOTOR_FACTOR ) < 120)) {
                        motors.go(LEFT_MOTOR_FACTOR * 45, RIGHT_MOTOR_FACTOR * 45); 
                    }
                    break;
                }
            }
        }
    }
    else{
        while (this->distanceFrontLeft() > FRONT_NEAR_DISTANCE) {
            if (!this->isWallFront()){
                this->followToNextTyle(folowProcedureType, startingHeading);
                this->pauza(1);
                if (this->compassPitch()*(-1)-startPitch>DIFFERENCE_PITCH) {
                    this->slopeUp();
                    this->pauza (400);
                    break;
                }
                if (state_tip1 == 0)
                    return;
            }
            else{
                encoders.reset();
                while ((encoders.counter(0) / LEFT_MOTOR_FACTOR) < 120 && ((encoders.counter(1) / RIGHT_MOTOR_FACTOR ) < 120)) {
                    motors.go(LEFT_MOTOR_FACTOR * 45, RIGHT_MOTOR_FACTOR * 45); 
                }
                break;
            }
            if (((encoders.counter(0) / LEFT_MOTOR_FACTOR) > (ONE_TILE_ENCODER_STEPS * 1.3)) || ((encoders.counter(1) / RIGHT_MOTOR_FACTOR ) > (ONE_TILE_ENCODER_STEPS * 1.3)))
                break;
            this->pauza(1);
            if (state_tip1 == 0)
                return;
            if ((heatedVictimDetected == false) && (encoders.counter(0) > 200)) {
                heatedVictimDetected = this->checkHeatedVictim();
            }
            if (crnosrebrnobijelo()==1) {
                this->blackTileBack();
                break;
            }
            if (this->isWallFrontNear())
                break;
        }
    }
    this->stop();
    this->pauza(250);
    TileColor = this->colorDetection();
    if (state_tip1 == 0)
        return;
    this->tileSignal(TileColor);
    if (state_tip1 == 0)
        return;
    if (state_tip1 == 0)
        return;
    return retVal;
}

void Robot::followRightLidars (float startCompass) {
    if (this->distanceRightFront() > 15.0) {
        if (this->distanceLeftBack() < 15.0)
            this->followLeftLidars(startCompass);
        else
            this->followIMU(startCompass);
    }
    else if (this->distanceLeftMiddle()<5){
        motors.go (LEFT_MOTOR_FACTOR * 90,  RIGHT_MOTOR_FACTOR * 50);
    }
    else if (this->distanceRightFront() > 10.0) {
        if ((this->distanceRightBack()-this->distanceRightFront()) < 0.0) {
            motors.go (LEFT_MOTOR_FACTOR * 80,  RIGHT_MOTOR_FACTOR * 50);
        }
        else if ((this->distanceRightBack()-this->distanceRightFront()) > 2.0) {
            motors.go (LEFT_MOTOR_FACTOR * 50,  RIGHT_MOTOR_FACTOR * 90);
        }
        else if ((this->distanceRightBack()-this->distanceRightFront()) > 1.0) {
            motors.go (LEFT_MOTOR_FACTOR * 60,  RIGHT_MOTOR_FACTOR * 80);
        }
        else{
            motors.go (LEFT_MOTOR_FACTOR * 70,  RIGHT_MOTOR_FACTOR * 70);
        }
    }
    else if (this->distanceRightFront() < 8.0 ) {
            if (this->distanceRightMiddle() < 6.0) {
                motors.go (LEFT_MOTOR_FACTOR * 50,  RIGHT_MOTOR_FACTOR * 80);
            }
            else if ((this->distanceRightFront()-this->distanceRightBack()) < 0.0) {
                motors.go (LEFT_MOTOR_FACTOR * 50,  RIGHT_MOTOR_FACTOR * 80);
            }
            else if ((this->distanceRightFront()-this->distanceRightBack()) > 2.0) {
                motors.go (LEFT_MOTOR_FACTOR * 90,  RIGHT_MOTOR_FACTOR * 50);
            }
            else if ((this->distanceRightFront()-this->distanceRightBack()) > 1.0) {
                motors.go (LEFT_MOTOR_FACTOR * 80,  RIGHT_MOTOR_FACTOR * 60);
            }
            else{
                motors.go (LEFT_MOTOR_FACTOR * 70,  RIGHT_MOTOR_FACTOR * 60);
            }
        }
    else {
         motors.go (LEFT_MOTOR_FACTOR * 60,  RIGHT_MOTOR_FACTOR * 70);
    }
}

void Robot::followLeftLidars (float startCompass) {
    if ((this->distanceLeftFront() > 20.0) || (abs(this->distanceLeftBack()-this->distanceLeftFront()) > 3)) {
        if (this->distanceRightBack() < 15.0)
            this->followRightLidars(startCompass);
        else
            this->followIMU(startCompass);
    }
    else if (this->distanceLeftFront() > 10.0) {
        if ((this->distanceLeftBack()-this->distanceLeftFront()) < 0.0) {
            motors.go (LEFT_MOTOR_FACTOR * 50,  RIGHT_MOTOR_FACTOR * 80);
        }
        else if ((this->distanceLeftBack()-this->distanceLeftFront()) > 2.0) {
            motors.go (LEFT_MOTOR_FACTOR * 80,  RIGHT_MOTOR_FACTOR * 60);
        }
        else if ((this->distanceLeftBack()-this->distanceLeftFront()) > 1.0) {
            motors.go (LEFT_MOTOR_FACTOR * 75,  RIGHT_MOTOR_FACTOR * 65);
        }
        else{
            motors.go (LEFT_MOTOR_FACTOR * 70,  RIGHT_MOTOR_FACTOR * 70);
        }
    }
    else if (this->distanceLeftFront() < 8.0 ) {
            if ((this->distanceLeftFront()-this->distanceLeftBack()) < 0.0) {
                motors.go (LEFT_MOTOR_FACTOR * 80,  RIGHT_MOTOR_FACTOR * 50);
            }
            else if ((this->distanceLeftFront()-this->distanceLeftBack()) > 2.0) {
                motors.go (LEFT_MOTOR_FACTOR * 50,  RIGHT_MOTOR_FACTOR * 90);
            }
            else if ((this->distanceLeftFront()-this->distanceLeftBack()) > 1.0) {
                motors.go (LEFT_MOTOR_FACTOR * 60,  RIGHT_MOTOR_FACTOR * 80);
            }
            else{
                motors.go (LEFT_MOTOR_FACTOR * 60,  RIGHT_MOTOR_FACTOR * 70);
            }
        }
    else {
         motors.go (LEFT_MOTOR_FACTOR * 70,  RIGHT_MOTOR_FACTOR * 65);
    }
}

void Robot::followIMU (float startHeading) {
    if (startHeading - this->compassHeading() > 2.0 ) {
        motors.go (LEFT_MOTOR_FACTOR * 65,  RIGHT_MOTOR_FACTOR * 70);
        this->pauza(10);
    }
    else if (startHeading - this->compassHeading() < -2.0 ) {
        motors.go (LEFT_MOTOR_FACTOR * 70,  RIGHT_MOTOR_FACTOR * 65);
        this->pauza(10);
    }
    else {
        motors.go (LEFT_MOTOR_FACTOR * 60,  RIGHT_MOTOR_FACTOR * 60);
        this->pauza(10);
    }
}

int Robot::crnosrebrnobijelo () {
    // case crno
    if (aR(IRS) < BLACK_MIDLE
         and aR(IRL) < BLACK_RIGHT) { 
        return 1;
    }
    // case srebrno
    else if (aR(IRS) > SILVER_MIDLE
         and aR(IRL) > SILVER_RIGHT) {             
        return 2; }
    // case ostalo
    else {
        return 0; }
}

char Robot::colorDetection () {
    float red, green, blue, yellow, violet;
    dW(colRst,LOW);
    this->pauza(50);
    if (state_tip1 == 0)
        return;
    red = this->colorRed();
    green = this->colorGreen();
    blue = this->colorBlue();
    yellow = this->colorYellow();
    violet = this->colorViolet();

// za sve arene
    if (red>3000 &&  green>5000 && blue>5000) {                                    //white
        return 'W'; }

// arena
#if arena == 1
    else if (red>1600 && green<1500 && blue<2300) {                                //red
        return 'R'; }
    else if (red<800 && green<2000 && blue>1200 && violet>1000 && green<blue) {    //blue
        return 'B'; }
    else if (red<800 && green<1100 && blue>1000 && green<blue) {                   //dark blue
        return 'B'; }
    else if (red<800 && green>2000 && blue<3000 && yellow>2500 && violet<1000 ){   //green
        return 'G'; }
#endif
// mobilna arena
#if arena == 2
    else if (red>1600 && green<2000 && blue<2000) {                                //red
        return 'R'; }
    else if (red<800 && green<2000 && blue>1200 && violet>1000 && green<blue) {    //blue
        return 'B'; }
    else if (red<800 && green>1500 && blue<3000 && yellow>2000 && violet<1000 ){   //green
        return 'G'; }
#endif

    else {
        return 'O'; } 
    dW(colRst,HIGH);
}

void Robot::tileSignal (char Signal) {
    int i;
    if (Signal=='B') {
        for(i=0; i<12; i++){
            dW(led1,HIGH);
            this->pauza(250);
            if (state_tip1 == 0)
                return;
            dW(led1,LOW);
            this->pauza(250);
            if (state_tip1 == 0)
                return;
        }
    }
    else if (Signal=='R') {
        for(i=0; i<8; i++){
            dW(led2,HIGH);
            this->pauza(250);
            if (state_tip1 == 0)
                return;
            dW(led2,LOW);
            this->pauza(250);
            if (state_tip1 == 0)
                return;
        }
    }
    else if (Signal=='G') {
        for(i=0; i<8; i++){
            dW(led3,HIGH);
            this->pauza(250);
            if (state_tip1 == 0)
                return;
            dW(led3,LOW);
            this->pauza(250);
            if (state_tip1 == 0)
                return;
        }
    }
}

void Robot::turn90Degrees(bool isLeft = false) {
    //uzmi trenutni smjer
    //ako je true -> lijevo 90 stupnjeva, inače desno
    //resetiraj enkodere

    //okreći se dok ne bude 90 stupnjeva ili potreban broj okretaja (treba prvo izmjeriti)
    //resetiraj enkoder, promijeni Orientation varijablu robota
    float startingHeading = this->compassHeading();
    float degreesDiference = 0;
    float degreesDiferencePrevious = 0;
    bool heatedVictimDetected = false;
//    float startingHeading = this->robotOrientationAngle;
//    Serial.println("     ****** OKRET ZAPOCET  ********** ");
//    if ((this->distanceFrontMiddle() > 7) && (((this->distanceBackLeft()+this->distanceBackRight()) / 2) > 20)){
//        while (this->distanceFrontMiddle() < 6){
//            motors.go(-50,-50);
//        }
//        motors.go(0,0);
//    }
    if ((this->distanceFrontMiddle() < 10) && (((this->distanceBackLeft()+this->distanceBackRight()) / 2) < 10)){
        while (this->distanceFrontMiddle() < ((this->distanceBackLeft()+this->distanceBackRight()) / 2)){
            motors.go(-50,-50);
        }
        motors.go(0,0);
    }
    else if (this->distanceFrontMiddle() < 7){
        while (this->distanceFrontMiddle() < 7){
            motors.go(-50,-50);
        }
        motors.go(0,0);
    }
    this->pauza(250);
    if (state_tip1 == 0)
        return;
    encoders.reset();
    if (isLeft == false) {
//    if (isLeft) {
//        Serial.println("     ****** OKRET DESNO  ********** ");
        float currentHeading = (startingHeading >= 270 && this->compassHeading() < 100) ? this->compassHeading() + 360 : this->compassHeading(); 
        degreesDiference = currentHeading - startingHeading;
        degreesDiferencePrevious = degreesDiference;
        if ((this->distanceRightFront() > 30) && (this->distanceRightBack() < 20)) {
            while (this->distanceRightBack() < 30) {
                motors.go(50,50);
                this->pauza(10);
            }
            this->pauza(100);
        }
        while (degreesDiference <= TURN_DEGREES) {
            if ((heatedVictimDetected == false) && (degreesDiference >= 45.0)) {
                heatedVictimDetected = this->checkHeatedVictim();
            }
            motors.go(LEFT_MOTOR_FACTOR * 70,  RIGHT_MOTOR_FACTOR * -70);
            this->pauza(15);
            if (state_tip1 == 0)
                return;
            if((encoders.counter(0)+encoders.counter(1)) > 1200)
                break;
            currentHeading = (startingHeading >= 270 && this->compassHeading() < 100) ? this->compassHeading() + 360 : this->compassHeading(); 
            degreesDiference = currentHeading - startingHeading;
            if ((degreesDiference - degreesDiferencePrevious) > 30){
                degreesDiference = degreesDiferencePrevious;
            }
            else{
                degreesDiferencePrevious = degreesDiference;
            }
        }    
/*        if (this->isWallLeft()){
            if ((this->distanceLeftFront() - this->distanceLeftBack()) > 3){
                while (((this->distanceLeftFront() - this->distanceLeftBack()) > 2)
                        &&
                        ((this->distanceLeftMiddle() - this->distanceLeftBack()) > 1)){
                    motors.go(LEFT_MOTOR_FACTOR * -45,  RIGHT_MOTOR_FACTOR * 45);
                    this->pauza(1);
                }
            }
            else if ((this->distanceLeftBack() - this->distanceLeftFront()) > 3){
                while (((this->distanceLeftBack() - this->distanceLeftFront()) > 2)
                        &&
                        ((this->distanceLeftBack() - this->distanceLeftMiddle()) > 1)){
                    motors.go(LEFT_MOTOR_FACTOR * 50,  RIGHT_MOTOR_FACTOR * -50);
                    this->pauza(1);
                }
            }
        }     
*/
    } else {
//        Serial.println("     ****** OKRET LIJEVO  ********** ");
        float currentHeading = (startingHeading <= 90 && this->compassHeading() > 260) ? this->compassHeading() - 360 : this->compassHeading();
        degreesDiference = startingHeading - currentHeading;
        degreesDiferencePrevious = degreesDiference;
        if ((this->distanceLeftBack() < 20) && (this->distanceLeftFront() > 30)) {
            while (this->distanceLeftBack() < 30) {
                motors.go(50,50);
                this->pauza(10);
            }
            this->pauza(100);
        }
        while (degreesDiference <= TURN_DEGREES) {
            if ((heatedVictimDetected == false) && (degreesDiference >= 45.0)) {
                heatedVictimDetected = this->checkHeatedVictim();
            }
            motors.go(LEFT_MOTOR_FACTOR * -65,  RIGHT_MOTOR_FACTOR * 65);
            this->pauza(15);
            if (state_tip1 == 0)
                return;
            if((encoders.counter(0)+encoders.counter(1)) > 1200)
                break;
            currentHeading = (startingHeading <= 90 && this->compassHeading() > 260) ? this->compassHeading() - 360 : this->compassHeading();
            degreesDiference = startingHeading - currentHeading;
            if ((degreesDiference - degreesDiferencePrevious) > 30){
                degreesDiference = degreesDiferencePrevious;
            }
            else{
                degreesDiferencePrevious = degreesDiference;
            }
        }     
/*        if (this->isWallRight()){
            if ((this->distanceRightBack() - this->distanceRightFront()) > 3){
                while (((this->distanceRightBack() - this->distanceRightFront()) > 2)
                        &&
                        ((this->distanceRightBack() - this->distanceRightMiddle()) > 1)){
                    motors.go(LEFT_MOTOR_FACTOR * -45,  RIGHT_MOTOR_FACTOR * 45);
                    this->pauza(1);
                }
            }
            else if ((this->distanceRightFront() - this->distanceRightBack()) > 3){
                while (((this->distanceRightFront() - this->distanceRightBack()) > 2)
                        &&
                        ((this->distanceRightMiddle() - this->distanceRightBack()) > 1)){
                    motors.go(LEFT_MOTOR_FACTOR * 50,  RIGHT_MOTOR_FACTOR * -50);
                    this->pauza(1);
                }
            }
        }     
*/
    }
    motors.go(0,0);
    this->pauza(250);
    if (state_tip1 == 0)
        return;
    if (((this->distanceBackLeft() + this->distanceBackRight())/2) < 9){
        while (((this->distanceBackLeft() + this->distanceBackRight())/2) < 8){
            if (this->distanceBackLeft() > this->distanceBackRight())
                motors.go(45,60);
            else if (this->distanceBackLeft() < this->distanceBackRight())
                motors.go(60,45);
            else
                motors.go(50,50);
        }
        motors.go(0,0);
    }
    motors.go(0,0);
    this->pauza(250);
    if (state_tip1 == 0)
        return;
//    Serial.println("     ****** OKRET ZAVRSEN  ********** ");
//    Serial.println("  ");
    this->changeRobotOrientation(isLeft);
}

bool Robot::isWallFront () {
    int leftSensor = dR(dgIRL);
    int rightSensor = dR(dgIRD);
//    if (this->distanceFrontMiddle()<=15 && (this->distanceFrontLeft()<=15 || this->distanceFrontRight()<=15)) {
    if ((leftSensor == 0) && (rightSensor == 0)) {
        return true; }
    else {
        return false; }
}

bool Robot::isWallFrontNear () {
    if (this->distanceFrontMiddle() < 6 && (this->distanceFrontLeft() < 6 || this->distanceFrontRight() < 6)) {
        return true; }
    else {
        return false; }
}

float Robot::WallFrontDistance () {
    float distF;

    distF = this->distanceFrontMiddle() + this->distanceFrontLeft() + this->distanceFrontRight();
    distF = distF / 3.0;
    return distF;
}

bool Robot::isWallBack () {
    if (this->distanceBackLeft()<=15 && this->distanceBackRight()<=15) {
        return true; }
    else {
        return false; }
}

bool Robot::isWallLeft () {
    if (this->distanceLeftMiddle()<=20 && (this->distanceLeftFront()<=20 || this->distanceLeftBack()<=20)) {
        return true; }
    else {
        return false; }
}

bool Robot::isWallRight() {
    if (this->distanceRightMiddle()<=20 && (this->distanceRightFront()<=20 || this->distanceRightBack()<=20)) {
        return true; }
    else {
        return false; }
}

void Robot::program () {
    int retState;
    int i;
    if (state_tip1 == 0) {
        if (state_tip2 == 1) {
            this->testSensors();
        }
        else{
            this->stop();
//            delay(2000);
//            testCompass();
//            Serial.print(strpit);
//            Serial.print("   Razlika: ");
//            Serial.println(this->compassPitch()*(-1)-strpit);
//            this->testSensors();       
//            testLidarsRead();
//            testLidarsRead10();
//            testLidars();
//            testIRDistSensors();
//            testColor();
//            testIRLineSensors();
//            testThermal(); 
/*
            motors.go(50,50);
            this->pauza(1000);
            motors.go(0,0);
            this->pauza(1000);
            this->turn90Degrees(false);
            Serial.print(encoders.counter(0));
            Serial.print("   ");
            Serial.println(encoders.counter(1));
            this->pauza(10000);
            this->turn90Degrees(true);
            Serial.print(encoders.counter(0));
            Serial.print("   ");
            Serial.println(encoders.counter(1));
            this->pauza(10000);
*/
        }
    }
    else {
        if (!this->isWallRight()){
            this->turn90Degrees();
            this->goAheadOneTile();
            if (state_tip1 == 0)
                return;
        }
        else if (this->isWallFront()){
            this->turn90Degrees(true);
        }
        else {
            this->goAheadOneTile();
            if (state_tip1 == 0)
                return;
        }
    }
}

void Robot::setState(Robot::State state) {
    this->state = state;
}

/*
 *Metoda iz klase Tile, vraća pokazivač na strukturu Position trenutnog polja.
 */
Position *Tile::getPosition() {
    return this->tilePosition;
}


/*
 *Metoda iz klase Tile, vraća pokazivač na enumerator TileType trenutnog polja.
 */
Tile::TileType *Tile::getTileType() {
    return &(this->tileType);
}

/*
 *Metoda iz klase Tile, vraća pokazivač na char koji sadrži podatke o susjednim poljima.
 */
char *Tile::getAccesibleNeighbourTiles() {
    return &(this->accesibleTiles);
}

/*
 *Metoda iz klase Tile, vraća pokazivač na enumerator VictimType trenutnog polja.
 */

Tile::VictimType *Tile::getVictimType() {
    return &(this->victimType);
}

/*
 *Metoda iz klase Tile, postavlja vrijednosti koordinata danog polja na nove vrijednosti. Kao parametar prima 3 cijela broja.
 */
void Tile::setPosition(short int newPositionX, short int newPositionY, short int newPositionZ) {
    this->tilePosition->positionX = newPositionX;
    this->tilePosition->positionY = newPositionY;
    this->tilePosition->positionZ = newPositionZ;
}

/*
 *Metoda iz klase Tile, postavlja vrijednosti tipa danog polja na nove vrijednosti. Kao parametar prima TileType.
 */
void Tile::setTileType(Tile::TileType newTileType) {
    this->tileType = newTileType;
}

/*
 *Metoda iz klase Tile, postavlja vrijednosti tipa žrtve na danom polju na nove vrijednosti. Kao parametar prima VictimType.
 */
void Tile::setVictimType(Tile::VictimType newVictimType) {
    this->victimType = newVictimType;
}



/*
 *Metoda iz klase Tile, postavlja 1 bit u podacima o susjednim poljima danog polja. Kao parametar prima pokazivač na polje koje 
 *želimo postaviti kao susjedno, dostupno polje iz trenutnog polja.
 */
void Tile::setAccesibleTile(Tile *accesibleTile) {
    Position *accesibleTilePosition = accesibleTile->getPosition();
    if (accesibleTilePosition->positionX - this->tilePosition->positionX == 1) {
        this->accesibleTiles |= 0x4;
    } else if (accesibleTilePosition->positionX - this->tilePosition->positionX == -1) {
        this->accesibleTiles |= 0x1;
    } else {
        if (accesibleTilePosition->positionY - this->tilePosition->positionY == 1) {
            this->accesibleTiles |= 0x8;
        } else if (accesibleTilePosition->positionY - this->tilePosition->positionY == -1) {
            this->accesibleTiles |= 0x2;
        } else {
            //Greska, polje ne moze biti susjedno samo sebi
            this->accesibleTiles = this->accesibleTiles;
        }
    }
}

/*
 *Klasa iz metode Tile, na temelju promjene koordinata vraća je li to polje dostupno i susjedno. Ako je, vraća true, inače false.
 */
bool Tile::isAccesibleNeighbourTile(short int differenceX, short int differenceY, short int differenceZ) {
    short int designatedTile = 0;
    if (differenceX == 1) {
        //printf("first bit: %x; second bit: %x\n", *(this->getAccesibleNeighbourTiles()), 0x4);
        designatedTile = *(this->getAccesibleNeighbourTiles()) & 0x4;
    } else if (differenceX == -1) {
        //printf("first bit: %x; second bit: %x\n", *(this->getAccesibleNeighbourTiles()), 0x1);
        designatedTile = *(this->getAccesibleNeighbourTiles()) & 0x1;
    } else {
        if (differenceY == 1) {
            //printf("first bit: %x; second bit: %x\n", *(this->getAccesibleNeighbourTiles()), 0x8);
            designatedTile = *(this->getAccesibleNeighbourTiles()) & 0x8;
        } else if (differenceY == -1) {
            //printf("first bit: %x; second bit: %x\n", *(this->getAccesibleNeighbourTiles()), 0x2);
            designatedTile = *(this->getAccesibleNeighbourTiles()) & 0x2;
        } else {
           //printf("Error: observing the same tile\n");
           designatedTile = 0;
        }
    }
    //printf("isAccesible: %d\n", designatedTile);
    return designatedTile != 0 ? true : false;
}

/*
 *Metoda iz klase Tile, vraća bool tako što uspoređuje pozicije 2 polja. Kao parametar prima pokazivač na drugo polje.
*/
bool Tile::isEqualPosition(Tile *otherTile) {
    Position *thisTilePosition = this->getPosition();
    Position *otherTilePosition = otherTile->getPosition();

    return thisTilePosition->positionX == otherTilePosition->positionX && 
           thisTilePosition->positionY == otherTilePosition->positionY &&
           thisTilePosition->positionZ == otherTilePosition->positionZ;
}

/*
 *Metoda iz klase Tile, vraća bool tako što uspoređuje pozicije 2 polja. Kao parametre prima 3 broja (x, y, z) s kojima uspoređuje
 *poziciju promatranog polja. 
 */
bool Tile::isEqualPosition(short int positionX, short int positionY, short int positionZ) {
    Position *thisTilePosition = this->getPosition();

    return thisTilePosition->positionX == positionX && 
           thisTilePosition->positionY == positionY &&
           thisTilePosition->positionZ == positionZ;
}

/*
 *Metoda iz klase Tile koja svakom tipu polja pridružuje određenu brojevnu vrijednost. Koristi se prilikom Dijkstrinog algoritma.
*/
short int Tile::tileTypeToWeight() {
    short int tileWeight = 0;
    switch (*(this->getTileType()))
    {
    case NORMAL:
        tileWeight = 1;
        break;
    case BLUE:
        tileWeight = 5;
        break;
    case BLACK:
        break;
    case STAIRS:
        tileWeight = 2;
        break;
    case RAMP:
        tileWeight = 2;
        break;
    case CHECKPOINT:
        tileWeight = 1;
        break;
    case SPEED_BUMPS:
        tileWeight = 1;
        break;
    default:
        break;
    }
    return tileWeight;
}

/*
 *Metoda iz klase Tile, ispisuje sve podatke o trenutno promatranom polju.
 */
void Tile::printTile() {
    //printf("Majmune");
    printf("x:%d y:%d z:%d neighbours:%d\n", this->getPosition()->positionX, this->getPosition()->positionY, this->getPosition()->positionZ, *(this->getAccesibleNeighbourTiles()));
}



/*
 *Metoda iz klase TileListElement. Vraća pokazivač na prethodni element liste (ako ne postoji, vraća nullptr) 
 */
TileListElement *TileListElement::getPreviousElement() {
    return this->previousElement;
}


/*
 *Metoda iz klase TileListElement. Vraća pokazivač na sljedeći element liste (ako ne postoji, vraća nullptr) 
 */
TileListElement *TileListElement::getNextElement() {
    return this->nextElement;
}

/*
 *Metoda iz klase TileListElement. Vraća pokazivač na objekt klase Tile koji pripada zadanom elementu liste.
 */
Tile *TileListElement::getTile() {
    return this->tile;
}


/*
 *Metoda iz klase TileListElement. Postavlja vrijednost prethodnog elementa liste (pokazivač)
 */
void TileListElement::setPreviousElement(TileListElement *previousElement) {
    this->previousElement = previousElement;
}


/*
 *Metoda iz klase TileListElement. Postavlja vrijednost sljedećeg elementa liste (pokazivač) 
 */
void TileListElement::setNextElement(TileListElement *nextElement) {
    this->nextElement = nextElement;
}




/*
 *Metoda iz klase TilesList. Vraća pokazivač na prvi element liste (nullptr ako je lista prazna).
 */
TileListElement *TilesList::getFirstElement() {
    return this->firstElement;
}


/*
 *Metoda iz klase TilesList. Vraća pokazivač na zadnji element liste (nullptr ako je lista prazna).
 */
TileListElement *TilesList::getLastElement() {
    return this->lastElement;
}


/*
 *Metoda iz klase TilesList. Postavlja vrijednost prvog elementa liste (pokazivač).
*/
void TilesList::setFirstElement(TileListElement *firstElement) {
    this->firstElement = firstElement;
}


/*
 *Metoda iz klase TilesList. Postavlja vrijednost zadnjeg elementa liste (pokazivač).
*/
void TilesList::setLastElement(TileListElement *lastElement) {
    this->lastElement = lastElement;
}


/*
 *Metoda iz klase TilesList. Pronalazi i vraća pokazivač na prvi element liste čiji objekt Tile ima zadane koordinate. Ako takav element
 *ne postoji, vraća nullptr.
 */
TileListElement *TilesList::getElementByPosition(short int positionX, short int positionY, short int positionZ) {
    for (TileListElement *currentElement = this->getFirstElement(); currentElement != nullptr; currentElement = currentElement->getNextElement()) {
        if (currentElement->getTile()->isEqualPosition(positionX, positionY, positionZ)) {
            return currentElement;
        }
    }
    return nullptr;
}


/*
 *Metoda iz klase TilesList. Kreira novi objekt TilesListElement za zadani Tile i dodaje ga na kraj liste.
 */
void TilesList::addElement(Tile *newTile) {
    TileListElement *currentElement = this->getLastElement();
    TileListElement *newListElement = new TileListElement(newTile);
    
    if (currentElement == nullptr) {
        this->setFirstElement(newListElement);
    } else {
        currentElement->setNextElement(newListElement);
    }

    newListElement->setPreviousElement(currentElement);
    newListElement->setNextElement(nullptr);
    this->setLastElement(newListElement);
    //printf("firstElement: %p ; lastElement: %p\n", this->getFirstElement(), this->getLastElement());
}

/*
 *Metoda iz klase TilesList. Vraća true ako je lista prazna, false inače.
 */
bool TilesList::isEmpty() {
    return this->getFirstElement() == nullptr ? true : false;
}

/*
 *Metoda iz klase TilesList. Provjerava postoji li Tile sa zadanim koordinatama u listi. Vraća true ako postoji, false inače.
 */
bool TilesList::checkIfTileExists(short int positionX, short int positionY, short int positionZ) {
    TileListElement *listElement = nullptr;

    for (listElement = this->getFirstElement(); listElement != nullptr; listElement = listElement->getNextElement()) {
        if (listElement->getTile()->isEqualPosition(positionX, positionY, positionZ)) {
            return true;
        }
    }
    return false;
}

/*
 *Metoda iz klase TilesList. Ispisuje sve objekte tipa Tile koji se nazale u zadanoj listi.
 */
void TilesList::printAllElements() {
    for (TileListElement *currentElement = this->getFirstElement(); currentElement != nullptr; currentElement = currentElement->getNextElement()) {
        currentElement->getTile()->printTile();
    }
}

/*
 *Metoda iz klase TilesList. Briše sve elemente iz liste koje pokazuju na Tile čiji je pokazivač proslijeđen kao parametar.
 *Ako uspješno izbriše bar 1 element, vraća true, false inače.
 */
bool TilesList::removeElement(Tile *tile) {
    bool foundTile = true;
    short int numberOfRemovedTiles = 0;

    if (this->getFirstElement() == nullptr) {
        return false;
    } else {
        for (TileListElement *currentElement = this->getFirstElement(); currentElement != nullptr; ) {
            bool isEqualPosition = currentElement->getTile()->isEqualPosition(tile);

            if (isEqualPosition == false) {
                currentElement = currentElement->getNextElement();                
            } else {
                TileListElement *next = currentElement->getNextElement();
                TileListElement *previous = currentElement->getPreviousElement();

                if (next == nullptr) {
                    if (previous == nullptr) {
                        this->setFirstElement(nullptr);
                        this->setLastElement(nullptr);
                    } else {
                        currentElement->getPreviousElement()->setNextElement(nullptr);
                        this->setLastElement(currentElement->getPreviousElement());
                    }
                } else {
                    if (previous == nullptr) {
                        this->setFirstElement(currentElement->getNextElement());
                        currentElement->getNextElement()->setPreviousElement(nullptr);
                    } else {
                        currentElement->getPreviousElement()->setNextElement(currentElement->getNextElement());
                        currentElement->getNextElement()->setPreviousElement(currentElement->getPreviousElement());
                    }
                }
                TileListElement *elementToDelete = currentElement;
                delete elementToDelete;
                currentElement = this->getFirstElement();
            }
         }
    }
    return numberOfRemovedTiles > 0 ? true : false;
}

/*
 *Metoda iz klase TilesList koja briše sve elemente iz liste.
 */
void TilesList::removeAllElements() {
    while(this->isEmpty() == false) {
        Tile *currentTileToRemove = this->getFirstElement()->getTile();
        this->removeElement(currentTileToRemove);
    }
}



/*
 *Metoda iz klase DistanceSetElement. Vraća pokazivač na element koji prethodi trenutnom elementu (nullptr ako nema prethodnika).
 */
DistanceSetElement *DistanceSetElement::getPreviousElement() {
    return this->previousElement;
}

/*
 *Metoda iz klase DistanceSetElement. Vraća pokazivač na element koji slijedi nakon trenutnog elementa (nullptr ako nema sljedbenika).
 */
DistanceSetElement *DistanceSetElement::getNextElement() {
    return this->nextElement;
}

/*
 *Metoda iz klase DistanceSetElement. Vraća pokazivač na objekt tipa Tile pridružen trenutnom elementu seta.
 */
Tile *DistanceSetElement::getTile() {
    return this->tile;
}

/*
 *Metoda iz klase DistanceSetElement. Vraća vrijednost udaljenosti trenutno promatranog elementa seta.
 */
short int DistanceSetElement::getDistance() {
    return this->distance;
}

/*
 *Metoda iz klase DistanceSetElement. Postavlja pokazivač na element seta koji prethodi trenuto promatranom elementu.
 */
void DistanceSetElement::setPreviousElement(DistanceSetElement *previousElement) {
    this->previousElement = previousElement;
}

/*
 *Metoda iz klase DistanceSetElement. Postavlja pokazivač na element koji slijedi nakon trenutnog elementa.
 */
void DistanceSetElement::setNextElement(DistanceSetElement *nextElement) {
    this->nextElement = nextElement;
}

/*
 *Metoda iz klase DistanceSetElement. Postavlja udaljenost trenutno promatranog elementa seta.
 */
void DistanceSetElement::setDistance(short int distance) {
    this->distance = distance;
}



/*
 *Metoda iz klase DistanceSet. Vraća pokazivač na prvi element seta (nullptr ako set nema elemenata).
 */
DistanceSetElement *DistanceSet::getFirstElement() {
    return this->firstElement;
}

/*
 *Metoda iz klase DistanceSet. Vraća pokazivač na zadnji element seta (nullptr ako set nema elemenata).
 */
DistanceSetElement *DistanceSet::getLastElement() {
    return this->lastElement;
}

/*
 *Metoda iz klase DistanceSet. Postavlja pokazivač na prvi element seta na vrijednost proslijeđenu kao argument.
 *Kao parametar prima pokazivač na prvi element seta.
 */
void DistanceSet::setFirstElement(DistanceSetElement *firstElement) {
    this->firstElement = firstElement;
}

/*
 *Metoda iz klase DistanceSet. Postavlja pokazivač na zadnji element seta na vrijednost proslijeđenu kao argument.
 *Kao parametar prima pokazivač na zadnji element seta.
 */
void DistanceSet::setLastElement(DistanceSetElement *lastElement) {
    this->lastElement = lastElement;
}

/*
 *Metoda iz klase DistanceSet. Dodaje novi element na kraj seta, ako element s tim Tile-om već ne postoji u setu.
 *Postavlja udaljenost novog elementa na -1.
 */
void DistanceSet::addElement(Tile *newTile) {
    short int positionX = newTile->getPosition()->positionX;
    short int positionY = newTile->getPosition()->positionY;
    short int positionZ = newTile->getPosition()->positionZ;

    bool tileAlreadyExists = this->checkIfTileExists(positionX, positionY, positionZ);

    if (!tileAlreadyExists) {
        DistanceSetElement *currentElement = this->getLastElement();
        DistanceSetElement *newSetElement = new DistanceSetElement(newTile);
    
        if (currentElement == nullptr) {
            this->setFirstElement(newSetElement);
        } else {
            currentElement->setNextElement(newSetElement);
        }

        newSetElement->setPreviousElement(currentElement);
        newSetElement->setNextElement(nullptr);
        this->setLastElement(newSetElement);
    }   
    //printf("firstElement: %p ; lastElement: %p\n", this->getFirstElement(), this->getLastElement());
}

/*
 *Metoda iz klase DistanceSet. Kao parametre prima koordinate polja koje nas zanima.
 *Vraća pokazivač na element seta koji ima zadane koordinate, nullptr ako takav element ne postoji.
 */
DistanceSetElement *DistanceSet::getElementByPosition(short int positionX, short int positionY, short int positionZ) {
    for (DistanceSetElement *currentElement = this->getFirstElement(); currentElement != nullptr; currentElement = currentElement->getNextElement()) {
        if (currentElement->getTile()->isEqualPosition(positionX, positionY, positionZ)) {
            return currentElement;
        }
    }
    return nullptr;
}

/*
 *Metoda iz klase DistanceSet. Kao parametar prima pokazivač na Tile koji nas zanima.
 *Vraća pokazivač na element seta koji pokazuje na zadani Tile, nullptr ako takav element ne postoji. 
 */
DistanceSetElement *DistanceSet::getElementByTile(Tile *tile) {
    for (DistanceSetElement *currentElement = this->getFirstElement(); currentElement != nullptr; currentElement = currentElement->getNextElement()) {
        if (currentElement->getTile()->isEqualPosition(tile)) {
            return currentElement;
        }
    }
    return nullptr;
}

/*
 *Metoda iz klase DistanceSet. Postavlja sve udaljenosti u setu na vrijednost proslijeđenu kao argument.
 */
void DistanceSet::setAllDistances(short int newDistance) {
    for (DistanceSetElement *currentElement = this->getFirstElement(); currentElement != nullptr; currentElement = currentElement->getNextElement()) {
        currentElement->setDistance(newDistance);
    }
}

/*
 *Metoda iz klase DistanceSet. Vraća true ako je set prazan, false inače.
 */
bool DistanceSet::isEmpty() {
    return this->getFirstElement() == nullptr;
}


/*
 *Metoda iz klase DistanceSet. Kao parametre prima pokazivač na objekt tipa Tile i brojčanu vrijednost udaljenosti.
 *Svakom objektu u setu koji pokazuje na polje proslijeđeno kao argument postavlja udaljenost na novu udaljenost.
 */
bool DistanceSet::setDistanceOfElement(Tile *tile, short int newDistance) {
    bool set = false;

    for (DistanceSetElement *currentElement = this->getFirstElement(); currentElement != nullptr; currentElement = currentElement->getNextElement()) {
        if (currentElement->getTile()->isEqualPosition(tile)) {
            currentElement->setDistance(newDistance);
            set = true;
            break;
        }
    }
    return set;
}

/*
 *Metoda iz klase DistanceSet. Kao parametre prima koordinate polja koje nas zanima.
 *Vraća true ako polje s tim koordinatama postoji u setu, false inače.
 */
bool DistanceSet::checkIfTileExists(short int positionX, short int positionY, short int positionZ) {
    DistanceSetElement *currentElement = nullptr;

    for (currentElement = this->getFirstElement(); currentElement != nullptr; currentElement = currentElement->getNextElement()) {
        if (currentElement->getTile()->isEqualPosition(positionX, positionY, positionZ)) {
            return true;
        }
    }
    return false;
}

/*
 *Metoda iz klase DistanceSet. Kao parametar prima pokazivač na jedan Tile, a vraća pokazivač na drugi objekt tipa Tile koji ima najmanju 
 *vrijednost (najbliži je odredištu) te je susjedan početnom polju.
 */
Tile *DistanceSet::getSmallestNeighbour(Tile *tile) {
    short int smallestDistance = -1;
    Tile *smallestNeighbour = nullptr;

    short int startX = tile->getPosition()->positionX;
    short int startY = tile->getPosition()->positionY;
    short int startZ = tile->getPosition()->positionZ;


    for (int x = -1; x <= 1; x++) {
        for (int y = -1; y <= 1; y++) {
            if (x + y == -1 || x + y == 1) {
                if (this->checkIfTileExists(startX + x, startY + y, startZ) && tile->isAccesibleNeighbourTile(x, y, 0)) {
                    Tile *potentialNextTile = this->getElementByPosition(startX + x, startY + y, startZ)->getTile();
                    short int potentialDistance = this->getElementByPosition(startX + x, startY + y, startZ)->getDistance();

                    if (potentialDistance >= 0 && (potentialDistance < smallestDistance || smallestDistance == -1)) {
                        smallestDistance = potentialDistance;
                        smallestNeighbour = potentialNextTile;
                    }
                }
            }
        }
    }
    return smallestNeighbour;
}

/*
 *Metoda iz klase DistanceSet. Ispisuje sve parove vrijednosti (udaljenost, tile) trenutno zapisane u set.
 */
void DistanceSet::printAllElements() {
    printf("\nSvi elementi set:\n");
    for (DistanceSetElement *currentElement = this->getFirstElement(); currentElement != nullptr; currentElement = currentElement->getNextElement()) {
        printf("Udaljenost: %d ; tile: ", currentElement->getDistance());
        currentElement->getTile()->printTile();
    }
}


/*
 *Metoda iz klase DistanceSet. Briše sve elemente seta koji pokazuju na Tile dan kao parametar metode. Ako je obrisan barem jedan element
 *seta vraća true, false inače.
 */
bool DistanceSet::removeElement(Tile *tile) {
    bool foundTile = true;
    short int numberOfRemovedTiles = 0;

    if (this->getFirstElement() == nullptr) {
        return false;
    } else {
        for (DistanceSetElement *currentElement = this->getFirstElement(); currentElement != nullptr; ) {
            bool isEqualPosition = currentElement->getTile()->isEqualPosition(tile);

            if (isEqualPosition == false) {
                currentElement = currentElement->getNextElement();                
            } else {
                DistanceSetElement *next = currentElement->getNextElement();
                DistanceSetElement*previous = currentElement->getPreviousElement();

                if (next == nullptr) {
                    if (previous == nullptr) {
                        this->setFirstElement(nullptr);
                        this->setLastElement(nullptr);
                    } else {
                        currentElement->getPreviousElement()->setNextElement(nullptr);
                        this->setLastElement(currentElement->getPreviousElement());
                    }
                } else {
                    if (previous == nullptr) {
                        this->setFirstElement(currentElement->getNextElement());
                        currentElement->getNextElement()->setPreviousElement(nullptr);
                    } else {
                        currentElement->getPreviousElement()->setNextElement(currentElement->getNextElement());
                        currentElement->getNextElement()->setPreviousElement(currentElement->getPreviousElement());
                    }
                }
                DistanceSetElement *elementToDelete = currentElement;
                delete elementToDelete;
                currentElement = this->getFirstElement();
            }
         }
    }
    return numberOfRemovedTiles > 0 ? true : false;
}

/*
 *Metoda iz klase DistanceSet. Briše sve elemente iz seta.
 */
void DistanceSet::removeAllElements() {
    while(this->isEmpty() == false) {
        Tile *currentTileToRemove = this->getFirstElement()->getTile();
        this->removeElement(currentTileToRemove);
    }
}




/*
 *Metoda iz klase Maze. Vraća pokazivač na listu posjećenih polja.
 */
TilesList *Maze::getVisitedTiles() {
    return this->visitedTiles;
}

/*
 *Metoda iz klase Maze. Vraća pokazivač na listu neposjećenih polja.
 */
TilesList *Maze::getUnknownTiles() {
    return this->unknownTiles;
}

/*
 *Metoda iz klase Maze. Vraća pokazivač na skup svih parova (udaljenost, polje).
 */
DistanceSet *Maze::getDistanceSet() {
    return this->distanceSet;
}

/*
 *Metoda iz klase Maze. Vraća pokazivač na trenutni Tile.
 */
Tile *Maze::getCurrentTile() {
    return this->currentTile;
}

/*
 *Metoda iz klase Maze. Vraća trenutnu vrijednost x-koordinate robota.
 */
short int Maze::getCurrentX() {
    return this->currentX;
}

/*
 *Metoda iz klase Maze. Vraća trenutnu vrijednost y-koordinate robota.
 */
short int Maze::getCurrentY() {
    return this->currentY;
}

/*
 *Metoda iz klase Maze. Vraća trenutnu vrijednost z-koordinate robota.
 */
short int Maze::getCurrentZ() {
    return this->currentZ;
}

/*
 *Metoda iz klase Maze. Postavlja trenutno polje na kojem se robot nalazi u labirintu.
 *Kao parametre prima trenutne koordinate robota.
 */
void Maze::setCurrentTile(short int currentX, short int currentY, short int currentZ) {
    printf("Trazimo polje s koordinatama (%d, %d, %d)\n", currentX, currentY, currentZ);
    if (this->getUnknownTiles()->checkIfTileExists(currentX, currentY, currentZ) == true) {
        this->currentTile = this->getUnknownTiles()->getElementByPosition(currentX, currentY, currentZ)->getTile();
    } else if (this->getVisitedTiles()->checkIfTileExists(currentX, currentY, currentZ) == true) {
        this->currentTile = this->getVisitedTiles()->getElementByPosition(currentX, currentY, currentZ)->getTile();
    } else {
        printf("Error: Tile does not exist\n");
    }
}

/*
 *Metoda iz klase Maze. U listu neposjećenih polje dodaje novo polje.
 */
void Maze::addUnknownTile(Tile *unknownTile) {
    this->getUnknownTiles()->addElement(unknownTile);
}

/*
 *Metoda iz klase Maze. U skup(listu) posjećenih polja dodaje novo posjećeno polje.
 */
void Maze::addVisitedTile(Tile *visitedTile) {
    this->getVisitedTiles()->addElement(visitedTile);
}

/*
 *Metoda iz klase Maze. Na temelju trenutne pozicije i orijentacije robota pomiče se u labirintu za 1 polje.
 */
void Maze::advance(Orientation orientation) {
    switch (orientation)
    {
    case ORIENTATION_UP:
        this->currentY = this->getCurrentY() + 1;
        break;
    case ORIENTATION_RIGHT:
        this->currentX = this->getCurrentX() + 1;
        break;
    case ORIENTATION_DOWN:
        this->currentY = this->getCurrentY() - 1;
        break;
    case ORIENTATION_LEFT:
        this->currentX = this->getCurrentX() - 1;
        break;
    default:
        break;
    }
    this->setCurrentTile(this->getCurrentX(), this->getCurrentY(), this->getCurrentZ());
}

/*
 *Metoda iz klase Maze. Vraća true ako postoji neposjećeno polje u pripadnoj listi iz klase, false inače.
 */
bool Maze::existUnvisitedTile() {
    return !(this->getUnknownTiles()->isEmpty());
}


/*
 *Metoda iz klase Maze. Vraća sljedeći krajnje odredišni Tile. Ako postoji neposjećeno polje, onda je to zadnje dodano polje u tu listu. 
 *Ako ne postoji i robot se ne nalazi na početnom polju (koordinate (0, 0, 0)), onda je to početno polje. Inače vraća nullptr.
 */
Tile *Maze::getNextDestination() {
    if (this->existUnvisitedTile()) {
        return this->getUnknownTiles()->getLastElement()->getTile();
    } else {
        if (this->getCurrentTile()->isEqualPosition(0, 0, 0)) {
            return nullptr;
        } else {
            return this->getVisitedTiles()->getFirstElement()->getTile();
        }
    }
}

/*
 *Metoda iz klase Maze. Vraća pokazivač na sljedeći Tile na koji robot treba otići. U slučaju da je sve obiđeno i robot je na 
 *početnom polju, vraća nullptr.
 */
Tile *Maze::getNextTile() {
    Tile *nextDestination = this->getNextDestination();
    Tile *nextTile = nullptr;

    if (nextDestination == nullptr) {
        return nullptr;
    } else {
        shortestDistanceToDestination(this->getCurrentTile(), nextDestination);
        nextTile = this->getDistanceSet()->getSmallestNeighbour(this->getCurrentTile());
        return nextTile;
    }
}

/*
 *Metoda iz klase Maze. Pronalazi najkraći put od trenutnog do ciljnog polja. Ustvari implementacija Dijsktrinog algoritma.
 *Kao parametre prima pokazivače na početni i odredišni Tile. 
 *Privremeno kreira 2 nove liste polja:
 *   markedTiles - skup polja do kojih smo odredili udaljenosti
 *   tilesToDetermine - skup polja do kojih nam je preostalo odrediti udaljenosti
 *U svakoj iteraciji algoritma u skup markedTiles dodamo jedan Tile te isto tako maknemo jedan iz skupa tilesToDetermine. 
 */
void Maze::shortestDistanceToDestination(Tile *startTile, Tile *destinationTile) {
    this->getDistanceSet()->setAllDistances(-1);

    TilesList *markedTiles = new TilesList();
    TilesList *tilesToDetermine = new TilesList();

    markedTiles->addElement(destinationTile);
    this->getDistanceSet()->setDistanceOfElement(destinationTile, 0);
    
    for (TileListElement *currentElement = this->getVisitedTiles()->getFirstElement(); currentElement != nullptr; currentElement = currentElement->getNextElement()) {
        Tile *currentTile = currentElement->getTile();

        if (*(currentTile->getTileType()) != Tile::TileType::UNKNOWN && *(currentTile->getTileType()) != Tile::TileType::BLACK) {
            tilesToDetermine->addElement(currentTile);
        }
    }

    while (this->getDistanceSet()->getElementByTile(startTile)->getDistance() == -1) {
        Tile *currentNearestTile = nullptr;
        short int nearestDistance = -1;

        for (TileListElement *currentElement = markedTiles->getFirstElement(); currentElement != nullptr; currentElement = currentElement->getNextElement()) {
            Tile *currentTile = currentElement->getTile();

            for (int x = -1; x <= 1; x++) {
                for (int y = -1; y <= 1; y++) {
                    if (x + y == -1 || x + y == 1) {
                        short int potentialX = currentTile->getPosition()->positionX + x;
                        short int potentialY = currentTile->getPosition()->positionY + y;
                        short int potentialZ = currentTile->getPosition()->positionZ;

                        if ((tilesToDetermine->checkIfTileExists(potentialX, potentialY, potentialZ) == true) && currentTile->isAccesibleNeighbourTile(x, y, 0) == true) {
                            short int currentDistance = this->getDistanceSet()->getElementByTile(currentTile)->getDistance();
                            short int distanceToAdd = tilesToDetermine->getElementByPosition(potentialX, potentialY, potentialZ)->getTile()->tileTypeToWeight();

                            if ((nearestDistance == -1 || currentDistance + distanceToAdd < nearestDistance) && distanceToAdd > 0) {
                                currentNearestTile = tilesToDetermine->getElementByPosition(potentialX, potentialY, potentialZ)->getTile();
                                nearestDistance = currentDistance + distanceToAdd;
                            }
                        }
                    }
                }
            }
        }
        this->getDistanceSet()->setDistanceOfElement(currentNearestTile, nearestDistance);
        markedTiles->addElement(currentNearestTile);
        tilesToDetermine->removeElement(currentNearestTile);
    }
    
    markedTiles->removeAllElements();
    delete markedTiles;

    tilesToDetermine->removeAllElements();
    delete tilesToDetermine;
}

/*
 *Metoda iz klase Maze. Na temelju trenutnog (*startTile) i odredišnog polja (*nextTile) vraća smjer u kojem robot treba biti okrenut
 *da bi stigao na odredišno polje. Radi samo za susjedna polja. Kao parametre prima pokazivače na početno i odredišno polje.
 */
Orientation Maze::getOrientationToNextTile(Tile *startTile, Tile *nextTile) {
    Position *startPosition = startTile->getPosition();
    Position *nextPosition = nextTile->getPosition();

    short int distanceX = nextPosition->positionX - startPosition->positionY;
    short int distanceY = nextPosition->positionY - startPosition->positionY;

    if (distanceX == 1) {
        return Orientation::ORIENTATION_RIGHT;
    } else if (distanceX == 0) {
        if (distanceY == 1) {
            return Orientation::ORIENTATION_UP;
        } else if (distanceY == -1) {
            return Orientation::ORIENTATION_DOWN;
        } else {
            return Orientation::UNKNOWN_ORIENTATION;
        }
    } else if (distanceX == -1) {
        return Orientation::ORIENTATION_LEFT;
    } else {
        return Orientation::UNKNOWN_ORIENTATION;
    }
}

/*
 *Metoda iz klase Maze. Ispisuje neke podatke o trenutnom stanju objekta.
 */
void Maze::printMaze() {
    printf("visitedTiles: %p ; unknownTiles: %p ; distanceSet: %p ; currentTile: %p\n", this->getVisitedTiles(), this->getUnknownTiles(), this->getDistanceSet(), this->getCurrentTile());
}
