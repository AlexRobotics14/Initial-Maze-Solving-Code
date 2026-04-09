#include <stdio.h>
#include <stdbool.h>
#include <Wire.h>
#include <Motoron.h>
#include <Servo.h>
#include <SparkFun_GridEYE_Arduino_Library.h>

#define HOT 80
#define COLD 0

int moveCounter = 0;
int valids[10][10] = { {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0} };
int position[2] = {0, 9};
int xhistory[1000];
int yhistory[1000];
int hottestLocation = 0;

const uint8_t sensorPin = 2;
const int minPulse = 500;
const int maxPulse = 2500;

Servo myservo;
GridEYE grideye;

MotoronI2C mc1(17);
MotoronI2C mc2(18);

void addValidMoves();
bool directionIsValid(int x1, int y1); // verified
bool checkHistory(int x2, int y2); // verified
int getTemperature(); // verified
void move();
int searchForValids(); // verified
void retraceToLocation(int target); // verified
void checkForRFID();
void clearBufferArray();

void setupMotoron(MotoronI2C & mc) {
  mc.reinitialize();
  mc.disableCrc();

  mc.clearResetFlag();
}

void setup() {
    //  set initial history to initial starting point
    xhistory[0] = 0;
    yhistory[0] = 9;


    Serial.begin(115200);
    Wire.begin();
    Wire1.begin();
    grideye.begin(0x69, Wire1);
    myservo.attach(9, minPulse, maxPulse);

    setupMotoron(mc1);
    setupMotoron(mc2);
    mc1.setMaxAcceleration(1, 140);
    mc1.setMaxDeceleration(1, 300);

    mc1.setMaxAcceleration(2, 200);
    mc1.setMaxDeceleration(2, 300);

    mc1.setMaxAcceleration(3, 80);
    mc1.setMaxDeceleration(3, 300);

    mc2.setMaxAcceleration(1, 140);
    mc2.setMaxDeceleration(1, 300);

    mc2.setMaxAcceleration(2, 200);
    mc2.setMaxDeceleration(2, 300);

    mc2.setMaxAcceleration(3, 80);
    mc2.setMaxDeceleration(3, 300);
}

void loop() {
    searchArea();
}

void searchArea() {
    int x = position[0];
    int y = position[1];
    bool needToAddValids = true;
    bool validMovesExist = false;
    int currentTemperature;
    int hottestTemperature = 0;
    // no need to add valids in this location if they already exist
    if (valids[y][x] > 0) {
        needToAddValids = false;
    }
    // for each direction point in that direction, check if it's valid, add it to valids and get the temperature if necesaary
    myservo.write(0);
    if (directionIsValid(x, y - 1)) {
        validMovesExist = true;
        currentTemperature = getTemperature();
        if (currentTemperature > hottestTemperature) {
            hottestLocation = 0;
        }
        if (needToAddValids) {
            valids[y][x] = valids[y][x] + 1;
        }
    }
    myservo.write(90 * 180 / 270);
    if (directionIsValid(x - 1, y)) {
        validMovesExist = true;
        currentTemperature = getTemperature();
        if (currentTemperature > hottestTemperature) {
            hottestLocation = 1;
        }
        if (needToAddValids) {
            valids[y][x] = valids[y][x] + 1;
        }
    }
    myservo.write(180 * 180 / 270);
    if (directionIsValid(x, y + 1)) {
        validMovesExist = true;
        currentTemperature = getTemperature();
        if (currentTemperature > hottestTemperature) {
            hottestLocation = 2;
        }
        if (needToAddValids) {
            valids[y][x] = valids[y][x] + 1;
        }
    }
    myservo.write(270 * 180 / 270);
    if (directionIsValid(x + 1, y)) {
        validMovesExist = true;
        currentTemperature = getTemperature();
        if (currentTemperature > hottestTemperature) {
            hottestLocation = 3;
        }
        if (needToAddValids) {
            valids[y][x] = valids[y][x] + 1;
        }
    }
    // move if we have a valid move and check for an rfid signal
    if (validMovesExist) {
        move();
        checkForRFID();
    // if we don't have a valid move move to the last location there was one. If none exist, create an infinite loop signifying the end of the algorithm
    } else {
        int retraceLocation = searchForValids();
        if (retraceLocation == -1) {
            while (true) {

            }
        } else {
            retraceToLocation(retraceLocation);
        }
    }
}

// direction is valid if no objects block it and it's not in our move history
bool directionIsValid(int x1, int y1) {
    int16_t t = pulseIn(sensorPin, HIGH);
    int16_t d = (t - 1000) * 2;
    if (checkHistory(x1, y1) && d > 5) {
        return true;
    }
    return false;
}

bool checkHistory(int x2, int y2) {
    for (int i1 = 0; i1 <= moveCounter; i1++) {
        if (xhistory[i1] == x2 && yhistory[i1] == y2) {
            return false;
        }
    }
    return true;
}

// detect the temperature
int getTemperature() {
    int count1 = 0;
    for (unsigned char i2 = 0; i2 < 64; i2++) {
        count1 = count1 + map(grideye.getPixelTemperature(i2), COLD, HOT, 0, 80);
    }
    return count1;
}

void move() {
    // set servo motor
    myservo.write(hottestLocation * 90 * 180 / 270);
    int16_t tInitial = pulseIn(sensorPin, HIGH);
    // wait for distance sensor to detect something before proceeding
    while (tInitial > 1850) {
        tInitial = pulseIn(sensorPin, HIGH);
    }
    int16_t dInitial = (tInitial - 1000) * 2;
    int16_t tCurrent = tInitial;
    int16_t dCurrent = dCurrent;
    // continue to move until we have moved a set distance
    while (dInitial - dCurrent < 100) {
        tCurrent = pulseIn(sensorPin, HIGH);
        // prevent missed readings from interfering with the algorithm
        while (current > 1850) {
            tCurrent = pulseIn(sensorPin, HIGH);
        }
        dCurrent = (tCurrent - 1000) * 2;
        // set motors based on hottest location
        if (hottestLocation == 0) {
        // set all motors to forward
            mc2.setSpeed(1, 800);
            mc2.setSpeed(2, 800);
        } else if (hottestLocation == 2) {
        // set all motors to backwards
            mc2.setSpeed(1, -800);
            mc2.setSpeed(2, -800);
        } else if (hottestLocation == 1) {
        // set FL BR motors to backwards, FR BL motors to forwards
            mc2.setSpeed(1, -800);
            mc2.setSpeed(2, 800);
        } else {
        // set FL BR motors to forwards, FR BL to backwards
            mc2.setSpeed(1, 800);
            mc2.setSpeed(2, -800);
        }
    }
    // set motors to zero once we have moved the set distance
    mc2.setSpeed(1, 0);
    mc2.setSpeed(2, 0);
    // update valids and move counter
    moveCounter = moveCounter + 1;
    int x3 = position[0];
    int y3 = position[1];
    valids[y3][x3] = valids[y3][x3] - 1;
    // change position based on direction and add this to our history
    if (hottestLocation == 0) {
        position[1] = position[1] - 1;
    } else if (hottestLocation == 2) {
        position[1] = position[1] + 1;
    } else if (hottestLocation == 1) {
      position[0] = position[0] - 1;
    } else {
      position[0] = position[0] + 1;
    }
    xhistory[moveCounter] = position[0];
    yhistory[moveCounter] = position[1];
}

// search for the most recent valid move in our history and return the move number this was on. If none are found return -1
int searchForValids() {
    int x4;
    int y4;
    for (int i3 = moveCounter; i3 >= 0; i3--) {
        x4 = xhistory[i3];
        y4 = yhistory[i3];
        if (valids[y4][x4] > 0) {
            return i3;
        } 
    }
    return -1;
}

// retrace to the location we last had a valid move
void retraceToLocation(int target) {
    for (int i4 = moveCounter - 1; i4 - target >= 0; i4--) {
        // find the direction to move based on our position and next move
        int x5 = xhistory[i4];
        int y5 = yhistory[i4];
        if (position[1] - y5 == 1) {
            hottestLocation = 0;
        } else if (position[1] - y5 == -1) {
            hottestLocation = 2;
        } else if (position[0] - x5 == 1) {
            hottestLocation = 1;
        } else {
            hottestLocation = 3;
        }
        move();
    }
}

// check for RFID signal
void checkForRFID() {
    unsigned char buffer[64];
    int count2 = 0;
    if (SoftSerial.available()) {
        while (SoftSerial.available()) {
            buffer[count2++] = SoftSerial.read();
            if (count2 == 64) {
                break;
            }
        }
        Serial.write(buffer, count2);
        clearBufferArray();
        count2 = 0;
    }
    int temp = getTemperature();
    if (Serial.available() && temp > someValue) {
        SoftSerial.write(Serial.read());
    }
}

void clearBufferArray() {
    for (int i5 = 0; i5 < count2; i5++)
    {
        buffer[i5]=NULL;
    }                  
}