/*
   CodeRunners RoboCup Code
   Season: 2019
   Members: Nadav Soudry, Aytan Geschwind, Noam Goldwasser, Adiel Benisty

   Code includes for offensive and defensive modes and test/diagnostic code

   (store batteries at 15.4V)
*/

// math library includes M_PI variable for angle calculations
#include <math.h>

#include <PID_v1.h>
// PID(Input, Output, Setpoint, Kp, Ki, Kd, Direction [DIRECT or INVERSE])

#include <stdio.h>

// OLED
#include <U8g2lib.h>
#include <U8x8lib.h>
#include <SPI.h>
U8G2_SSD1306_128X64_NONAME_2_HW_I2C OLED(U8G2_R0); // OLED definition

// Pixy
#include <Pixy.h>
Pixy pixy;
uint16_t blocks;
/*useful pixy methods:
  pixy.blocks[i].signature The signature number of the detected object (1-7 for normal signatures)
  pixy.blocks[i].x The x location of the center of the detected object (0 to 319)
  pixy.blocks[i].y The y location of the center of the detected object (0 to 199)
  pixy.blocks[i].width The width of the detected object (1 to 320)
  pixy.blocks[i].height The height of the detected object (1 to 200)
  pixy.blocks[i].angle The angle of the object detected object if the detected object is a color code.
  pixy.blocks[i].print() A member function that prints the detected object information to the serial port
*/

// Unique Robot Configurations
// #include "Offense_Config.h"
#include "Defense_Config.h"

#include "Pinout.h"
#include "ClassConstants.h"

void setup() {
  // Serial monitor
  SerialUSB.begin(9600);
  Serial1.begin(9600);
  Serial.begin(9600);
  SerialUSB.println("Starting");

  // OLED setup
  OLED.begin();
  OLED.setFont(u8g2_font_helvB12_tr);
  OLED.setFontMode(1); //transparent
  OLED.setDrawColor(2); //shows white on black and black on white
  OLED.clearDisplay();

  // button setup
  pinMode(B1, INPUT); // INPUT_PULLUP);
  pinMode(B2, INPUT); // INPUT_PULLUP);
  pinMode(B3, INPUT); // INPUT_PULLUP);

  // Motor Setup
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M1_PWM, OUTPUT);

  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  pinMode(M2_PWM, OUTPUT);

  pinMode(M3_IN1, OUTPUT);
  pinMode(M3_IN2, OUTPUT);
  pinMode(M3_PWM, OUTPUT);

  pinMode(M4_IN1, OUTPUT);
  pinMode(M4_IN2, OUTPUT);
  pinMode(M4_PWM, OUTPUT);

  pinMode(M5_IN1, OUTPUT);
  pinMode(M5_IN2, OUTPUT);
  pinMode(M5_PWM, OUTPUT);

  //Ground Sensor Setup
  pinMode(GS_ANI, INPUT);
  pinMode(GS_S0, OUTPUT); pinMode(GS_S1, OUTPUT); pinMode(GS_S2, OUTPUT); pinMode(GS_S3, OUTPUT);
  pinMode(VLED, OUTPUT);
  digitalWrite(VLED, HIGH);

  // pixy camera
  pixy.init();
  delay(2000);

  // displays different options for testing or playing.
  drawOptions(options, choice, optionSize);

  // set north when robot turns on to the value it is currently
  fieldNorth = readCompass();
  startTime = millis();
}

void loop() {
  // timer for loop consistency and for goalie state switch to offense
  while (!(currentTime >= startTime + loopDelay)) {
    currentTime = millis();
    if (currentTime % ballStoppedTime <= 5) {
      ballStopped = true;
    }

    // continue checking white line between frames
    avoidOutOfBounds();
  }
  if (currentTime % ballStoppedTime <= 5) {
    ballStopped = true;
  }
  startTime = millis();

  // switch statement for all states mid-playing
  // switch case will carry out a specific function based on the value of STATE
  switch (STATE)
  {
    case 0:
      // check buttons, update screen, update STATE and choice
      checkButtons();
      break;
    case 1:
      // offense against yellow
      offense(2);
      break;
    case 2:
      // defense against yellow
      defense(2);
      break;
    case 3:
      // camera test
      cameraTest();
      break;
    case 4:
      // ground sensor test
      groundSensorTest();
      break;
    case 5:
      // ground sensor calibration
      groundSensorCalibration();
      break;
    case 6:
      // compass test
      compassTest();
      STATE = 0;
      break;
    case 7:
      // compass calibration
      compassCalibration();
      while (digitalRead(B3) == 0);
      STATE = 0;
      break;
    case 8:
      // current sensing
      currentSensingTest();
      break;
    case 9:
      // dribbler test
      dribblerTest();
      break;
    case 10:
      // drive to the middle of the field
      forwardLostBall();
      break;
    case 11:
      // against blue
      forwardHasBall(3);
      break;
    case 12:
      // against yellow
      forwardHasBall(2);
      break;
    case 13:
      // defense against blue
      defense(3);
      break;
    case 14:
      // offense against blue
      offense(3);
      break;
  }
}

void offense (int goalObject) {
  // update blocks object
  uint16_t blocks;
  blocks = pixy.getBlocks();

  if (true) // blocks
  {
    // filter objects for largest ball object
    boolean seenLargerBall = false;

    for (int i = 0; i < blocks; i++) {
      if (pixy.blocks[i].signature == 1 && !seenLargerBall) {
        seenLargerBall = true;

        // distance angle formula for ball
        int ballXCoord = pixy.blocks[i].x - 160;
        int ballYCoord = pixy.blocks[i].y - 100;
        ballAngle = atan2(ballYCoord, ballXCoord) * 180 / M_PI;
        ballDistance = sqrt(ballYCoord * ballYCoord + ballXCoord * ballXCoord);
        // make angle positive
        if (ballAngle < 0) {
          ballAngle += 360;
        }

        // shift angle by objectOffset degrees so north is 0
        ballAngle += objectOffset;
        if (ballAngle >= 360) {
          ballAngle -= 360;
        }
      }
    }

    // hasn't seen the ball this loop
    if (!seenLargerBall) {
      forwardLostBallCounter++;
    } else {
      forwardLostBallCounter = 0;
    }
    if (forwardLostBallCounter >= 70) {
      targetGoal = goalObject;
      STATE = 10;
      return;
    }

    // captured ball
    if ((ballAngle < capturedLeft || ballAngle > capturedRight) && ballDistance < capturedDistance) {
      if (forwardHasBallTimer >= 10) {
        forwardHasBallTimer = 15;
        if (goalObject == 2) {
          STATE = 12;
        } else {
          STATE = 11;
        }
        return;
      } else {
        forwardHasBallTimer++;
      }
    } else {
      if (forwardHasBallTimer > 0) {
        forwardHasBallTimer -= 2;
      }

      motor(5, 190);

      drivePID(ballAngle, map(ballDistance, 50, 120, 225, 255));
    }
  } else {
    // no objects seen
    // brakeMotors();
  }

  avoidOutOfBounds();
}

void forwardHasBall(int goalObject) {
  // update blocks object
  uint16_t blocks;
  blocks = pixy.getBlocks();

  boolean seenLargerBall = false;
  boolean seenLargerGoal = false;

  for (int i = 0; i < blocks; i++) {
    if (pixy.blocks[i].signature == 1 && (!seenLargerBall)) {
      seenLargerBall = true;

      // distance angle formula for ball
      int ballXCoord = pixy.blocks[i].x - 160;
      int ballYCoord = pixy.blocks[i].y - 100;
      ballAngle = atan2(ballYCoord, ballXCoord) * 180 / M_PI;
      ballDistance = sqrt(ballYCoord * ballYCoord + ballXCoord * ballXCoord);
      // make angle positive
      if (ballAngle < 0) {
        ballAngle += 360;
      }

      // shift angle by objectOffset degrees so north is 0
      ballAngle += objectOffset;
      if (ballAngle >= 360) {
        ballAngle -= 360;
      }
    }
    else if (pixy.blocks[i].signature == goalObject && (!seenLargerGoal)) {
      seenLargerGoal = true;

      // distance angle formula for goal
      int goalX = pixy.blocks[i].x - 160;
      int goalY = pixy.blocks[i].y - 100;
      yellowAngle = atan2(goalY, goalX) * 180 / M_PI;
      yellowDistance = sqrt(goalY * goalY + goalX * goalX);

      // make angle positive
      if (yellowAngle < 0) {
        yellowAngle += 360;
      }
      // shift angle by objectOffset degrees so north is 0
      yellowAngle += objectOffset;
      if (yellowAngle >= 360) {
        yellowAngle -= 360;
      }
    }
  }

  // if we still have the ball
  if ((ballAngle < capturedLeft || ballAngle > capturedRight) && ballDistance < capturedDistance) {
    if (forwardHasBallTimer < 10) {
      forwardHasBallTimer ++;
    }
  } else {
    forwardHasBallTimer --;
  }
  if (forwardHasBallTimer <= 0 && goalObject == 2) {
    STATE = 1;
    return;
  } else if (forwardHasBallTimer <= 0 && goalObject == 3) {
    STATE = 14;
    return;
  }

  // get compass values
  float headDif = headingDif();
  if (headDif < 0) {
    headDif += 360;
  }

  if ((yellowAngle < headDif + 70 && yellowAngle > headDif - 70) && seenLargerGoal) {
    goalAngle = yellowAngle;
  } else {
    goalAngle = headDif;
    // goalAngle = yellowAngle;
  }

  // rotate around the ball or drive to goal
  if (goalAngle > 25 && goalAngle < 180) {
    motor(5, 190); // suck in ball

    motor(1, 0);
    motor(2, -190);
    motor(3, 0);
    motor(4, 190);
  } else if (goalAngle > 180 && goalAngle < 335) {
    motor(5, 190); // suck in ball

    motor(1, 0);
    motor(2, 190);
    motor(3, 0);
    motor(4, -190);
  } else {
    motor(5, 190);
    // roll ball away slowly to increase robot velocity

    drivePID(goalAngle, 255); // full speed to goal
    // drive(goalAngle + 180, 0, 255); // full speed to goal

  }
}

// if the offensive robot loses the ball, drive to the middle of the field using the goal
void forwardLostBall () {
  // update blocks object
  uint16_t blocks;
  blocks = pixy.getBlocks();

  boolean seenLargerBall = false;
  boolean seenLargerGoal = false;

  for (int i = 0; i < blocks; i++) {
    if (pixy.blocks[i].signature == 1 && (!seenLargerBall)) {
      seenLargerBall = true;

      // distance angle formula for ball
      int ballXCoord = pixy.blocks[i].x - 160;
      int ballYCoord = pixy.blocks[i].y - 100;
      ballAngle = atan2(ballYCoord, ballXCoord) * 180 / M_PI;
      ballDistance = sqrt(ballYCoord * ballYCoord + ballXCoord * ballXCoord);
      // make angle positive
      if (ballAngle < 0) {
        ballAngle += 360;
      }

      // shift angle by objectOffset degrees so north is 0
      ballAngle += objectOffset;
      if (ballAngle >= 360) {
        ballAngle -= 360;
      }
    }
    else if (pixy.blocks[i].signature == 2 && (!seenLargerGoal)) {
      seenLargerGoal = true;

      // distance angle formula for goal
      int goalX = pixy.blocks[i].x - 160;
      int goalY = pixy.blocks[i].y - 100;
      yellowAngle = atan2(goalY, goalX) * 180 / M_PI;
      yellowDistance = sqrt(goalY * goalY + goalX * goalX);

      // make angle positive
      if (yellowAngle < 0) {
        yellowAngle += 360;
      }
      // shift angle by objectOffset degrees so north is 0
      yellowAngle += objectOffset;
      if (yellowAngle >= 360) {
        yellowAngle -= 360;
      }
    }
  }

  // return to offense if we see the ball
  if (seenLargerBall) {
    if (targetGoal == 2) {
      STATE = 1;
    } else {
      STATE = 14;
    }
  }

  if (yellowAngle > 40 && yellowAngle < 180) {
    drive(0, 100, 200);
  } else if (yellowAngle < 320 && yellowAngle > 180) {
    drive(0, -100, 200);
  } else {
    if (yellowDistance > 100) {
      drive(yellowAngle + 180, 0, 245);
    } else if (yellowDistance < 60) {
      drive(yellowAngle, 0, 245);
    } else {
      brakeMotors();
    }
  }
}

void defense (int goalObject) {
  // update blocks object
  uint16_t blocks;
  blocks = pixy.getBlocks();

  boolean seenLargerBall = false;
  boolean seenLargerGoal = false;

  int ballXCoord;
  int ballYCoord;

  if (blocks)
  {
    for (int i = 0; i < blocks; i++) {
      // distance angle formula
      if (pixy.blocks[i].signature == 1 && !seenLargerBall) {
        seenLargerBall = true;

        ballXCoord = pixy.blocks[i].x - 160;
        ballYCoord = pixy.blocks[i].y - 100;

        ballAngle = atan2(ballYCoord, ballXCoord) * 180 / M_PI;
        ballDistance = sqrt(ballYCoord * ballYCoord + ballXCoord * ballXCoord);

        // make angle positive
        if (ballAngle < 0) {
          ballAngle += 360;
        }

        // shift angle by objectOffset degrees so north is 0
        ballAngle += objectOffset;
        if (ballAngle >= 360) {
          ballAngle -= 360;
        }
      }
    }

    // if we don't see a ball
    if (!seenLargerBall) {
      ballStopped = false;
    }


    // change to offense if the ball stopped
    if (ballStopped == true) {
      if ((abs(lastBallX - ballXCoord) < 12 && abs(lastBallY - ballYCoord) < 12) && (ballAngle < 90 || ballAngle > 270)) {
        if (goalObject == 2) {
          STATE = 1;
          return;
        } else {
          STATE = 14;
          return;
        }
      } else {
        ballStopped = false;
        lastBallX = ballXCoord;
        lastBallY = ballYCoord;
      }
    }

    // if we have the ball
    if ((ballAngle < capturedLeft || ballAngle > capturedRight) && (ballDistance < capturedDistance && ballDistance > 20)) {
      goalieHasBallCounter++;
    } else {
      goalieHasBallCounter = 0;
    }
    
    if (goalieHasBallCounter > 10) {
      if (goalObject == 2) {
        STATE = 1;
        return;
      } else {
        STATE = 14;
        return;
      }
    }

    if (ballAngle < capturedLeft && ballAngle > capturedRight) {
      compassInPlace();
    } else if (ballAngle > 270) {
      // drive to the right
      drivePIDGoalie(265, 240);
      // drive(265 + 180, 0, 240);
    } else if (ballAngle < 90) {
      // drive to the left
      drivePIDGoalie(95, 240);
      // drive(95 + 180, 0, 240);
    } else {
      // compassInPlace();
      brakeMotors();
    }
  } else {
    brakeMotors();
    updateScreen("", "no ball", "");
  }

  avoidOutOfBounds();

  char angleString1[6];
  dtostrf(ballAngle, 6, 2, angleString1);
  updateScreen(angleString1, "", "");
}

void avoidOutOfBounds () {
  // AVOID OUT OF BOUNDS
  lineLeft = false;
  lineRight = false;
  lineFront = false;
  lineBack = false;

  readGroundSensor();

  if (groundSensor[GS0] > GS_THRESHOLD_WHITE || groundSensor[GS1] > GS_THRESHOLD_WHITE || groundSensor[GS2] > GS_THRESHOLD_WHITE)
  {
    lineFront = true;
    drive(0, 0, 240);
    delay(300);
    brakeMotors();
    delay(100);

    while ((ballAngle < 80 || ballAngle > 280) && !(groundSensor[GS0] > GS_THRESHOLD_WHITE || groundSensor[GS1] > GS_THRESHOLD_WHITE || groundSensor[GS2] > GS_THRESHOLD_WHITE)) {
      // update blocks object
      delay(20);
      uint16_t blocks;
      blocks = pixy.getBlocks();

      brakeMotors();
    }
  }
  if (groundSensor[GS3] > GS_THRESHOLD_WHITE || groundSensor[GS4] > GS_THRESHOLD_WHITE || groundSensor[GS5] > GS_THRESHOLD_WHITE || groundSensor[GS6] > GS_THRESHOLD_WHITE)
  {
    lineRight = true;
    drive(270, 0, 240);
    delay(300);
    brakeMotors();
    delay(100);

    while ((ballAngle < 350 && ballAngle > 190) && !(groundSensor[GS3] > GS_THRESHOLD_WHITE || groundSensor[GS4] > GS_THRESHOLD_WHITE || groundSensor[GS5] > GS_THRESHOLD_WHITE || groundSensor[GS6] > GS_THRESHOLD_WHITE)) {
      // update blocks object
      delay(20);
      uint16_t blocks;
      blocks = pixy.getBlocks();

      brakeMotors();
    }
  }
  if (groundSensor[GS7] > GS_THRESHOLD_WHITE || groundSensor[GS8] > GS_THRESHOLD_WHITE || groundSensor[GS9] > GS_THRESHOLD_WHITE || groundSensor[GS10] > GS_THRESHOLD_WHITE)
  {
    lineLeft = true;
    drive(90, 0, 240);
    delay(300);
    brakeMotors();
    delay(100);

    while ((ballAngle < 170 && ballAngle > 10) && !(groundSensor[GS7] > GS_THRESHOLD_WHITE || groundSensor[GS8] > GS_THRESHOLD_WHITE || groundSensor[GS9] > GS_THRESHOLD_WHITE || groundSensor[GS10] > GS_THRESHOLD_WHITE)) {
      // update blocks object
      delay(20);
      uint16_t blocks;
      blocks = pixy.getBlocks();

      brakeMotors();
    }
  }
  if (groundSensor[GS11] > GS_THRESHOLD_WHITE || groundSensor[GS12] > GS_THRESHOLD_WHITE || groundSensor[GS13] > GS_THRESHOLD_WHITE || groundSensor[GS14] > GS_THRESHOLD_WHITE)
  {
    lineBack = true;
    drive(180, 0, 240);
    delay(300);
    brakeMotors();
    delay(100);

    while ((ballAngle > 100 && ballAngle < 280) && !(groundSensor[GS11] > GS_THRESHOLD_WHITE || groundSensor[GS12] > GS_THRESHOLD_WHITE || groundSensor[GS13] > GS_THRESHOLD_WHITE || groundSensor[GS14] > GS_THRESHOLD_WHITE)) {
      // update blocks object
      delay(20);
      uint16_t blocks;
      blocks = pixy.getBlocks();

      brakeMotors();
    }
  }
}

/*
   capture ball test code
*/
void driveToBall() {
  // update blocks object
  uint16_t blocks;
  blocks = pixy.getBlocks();

  if (blocks)
  {
    // distance angle formula
    int ballXCoord = pixy.blocks[0].x - 160;
    int ballYCoord = pixy.blocks[0].y - 100;
    ballAngle = atan2(ballYCoord, ballXCoord) * 180 / M_PI;
    ballDistance = sqrt(ballYCoord * ballYCoord + ballXCoord * ballXCoord);

    // make angle positive
    if (ballAngle < 0) {
      ballAngle += 360;
    }

    // shift angle by objectOffset degrees so north is 0
    ballAngle += objectOffset;
    if (ballAngle >= 360) {
      ballAngle -= 360;
    }

    // double to string
    char angleString[6];
    char distanceString[6];
    dtostrf(ballAngle, 6, 2, angleString);
    dtostrf(ballDistance, 6, 2, distanceString);

    if ((ballAngle < 10 || ballAngle > 355) && ballDistance < 58) {
      brakeMotors();
      updateScreen("Captured Ball", distanceString, angleString);
    } else {
      updateScreen("Ball", distanceString, angleString);

      // drive to ball
      ballAngle = ballAngle + 180;
      drive(ballAngle, 0, 230);
    }
  } else {
    brakeMotors();
    updateScreen("No Object", "", "");
  }
}

/*
   functions to control motors:
   (1) drive PID for offense and goalie
   (2) control driving
   (3) control a single motor
   (4) break all motors
   (5) current sensing
*/

void driveCompass(float dir, int PWR) {
  dir += 180;

  // get compass values
  float headDif = headingDif();
  if (headDif < -180) {
    headDif += 360;
  }
  headDif = -headDif;

  if (headDif < 10 && headDif > -10) {
    drive(dir, 0, PWR);
  } else {
    float correctedDir = dir - headDif;

    // P
    int rot = -map(headDif, -180, 180, -120, 120);

    // D
    float change = headDif - lastCompassError; // CHECK SIGNS
    change *= derTerm;
    // FINISH here

    drive(correctedDir, rot, PWR); // angle, rot, power
  }

  lastCompassError = headDif;
}

void compassInPlace () {
  // get compass values
  float headDif = headingDif();
  if (headDif < -180) {
    headDif += 360;
  }
  headDif = -headDif;

  if (headDif < 8 && headDif > -8) {
    brakeMotors();
  } else {
    // P
    int correctionSpeed = 0;

    if (headDif > 0) {
      correctionSpeed = map(headDif, 0, 180, 100, 150);

      drive(20, -correctionSpeed, 0);
    } else {
      correctionSpeed = map(headDif, -180, 0, -150, -100);

      drive(-20, -correctionSpeed, 0);
    }
  }
}

double Input, rot, Setpoint;
double Kp = 2.8, Ki = 0.0, Kd = 0.4; // 7;  // Kp = 1.5, Ki = 0.6, Kd = 0.8;

// declare:
// PID(Input, Output, Setpoint, Kp, Ki, Kd, Direction [DIRECT or REVERSE])
PID anglePID(&Input, &rot, &Setpoint, Kp, Ki, Kd, REVERSE);

// modify:
// anglePID.SetTunings(Kp, Ki, Kd);
// anglePID.SetOutputLimits(0, 160);

// angle is the input, rot is the output
void drivePID (int angle, int PWR) {
  anglePID.SetTunings(Kp, Ki, Kd);

  boolean negativeAngle = false;
  if (angle > 180) {
    angle = abs(angle - 360);
    negativeAngle = true;
  }

  anglePID.SetSampleTime(1); // CHECK THIS
  Input = angle;
  Setpoint = 0;
  anglePID.SetMode(AUTOMATIC);
  anglePID.Compute();

  double correctionSpeed = 0;
  // double compensationModifier = map(ballDistance, 35, 120, 1.15, 0.85);
  // SerialUSB.println(compensationModifier, 5);

  if (angle > 6 && !negativeAngle) {
    correctionSpeed = map(rot, 0, 255, 0, 160);
    // correctionSpeed = rot;
    // correctionSpeed *= compensationModifier;
    drive(180 + angle, correctionSpeed, PWR);
  } else if (angle > 6) {
    correctionSpeed = -map(rot, 0, 255, 0, 160);
    // correctionSpeed = -rot;
    // correctionSpeed *= compensationModifier;
    drive(180 + angle, correctionSpeed, PWR);
  } else {
    drive(180, 0, PWR);
  }
  // SerialUSB.println(correctionSpeed);
}

void drivePIDGoalie (int dir, int PWR) {
  // get compass values
  float headDif = headingDif();
  if (headDif < -180) {
    headDif += 360;
  }
  headDif = -headDif;
  float correctedDir = dir - headDif; // CHECK THIS

  anglePID.SetTunings(Kp, Ki, Kd);

  boolean negativeAngle = false;
  if (headDif < 0) {
    headDif = abs(headDif);
    negativeAngle = true;
  }

  anglePID.SetSampleTime(1);

  Input = headDif;
  Setpoint = 0;
  anglePID.SetMode(AUTOMATIC);
  anglePID.Compute();

  double correctionSpeed = 0;
  if (headDif > 3 && !negativeAngle) {
    correctionSpeed = -map(rot, 0, 255, 0, 120);
    drive(180 + correctedDir, correctionSpeed, PWR);
  } else if (headDif > 3) {
    correctionSpeed = map(rot, 0, 255, 0, 120);
    drive(180 + correctedDir, correctionSpeed, PWR); // MAKE 0 POWER
  } else {
    drive(180 + correctedDir, 0, PWR);
  }
}

float angleDif(float x, float y) //used to calculate the difference between actual heading and the target heading in drivePID()
{
  float radianDif = ((x - y) * (float)71) / ((float)4068); // to radians
  float a = (float) atan2(sin(radianDif), cos(radianDif));
  a =  (a * (float)4068) / (float)71; //to degrees

  return a;
}

#define POSITIVE true
#define NEGATIVE false
void drive(int dir, int rot, int PWR)
{
  //In order to make the math for x-configuration omnidrive simpler, the axes are shifted 45 degrees
  //Instead of using an x-axis for left/right and a y-axis for forward/back,
  //an alpha axis that points forward-right/backward-left and a beta axis that points forward-left/backward right are used.

  int alphaVec = 0; //this vector points forward-right
  int betaVec = 0; //this vector points forward-left

  dir =  dir - 90; // make 0 straight

  dir = dir - 45; //shifting axis
  if (dir < 0) {
    dir = dir + 360;
  }
  else if (dir >= 360) {
    dir = dir - 360;
  }

  int scale = 10000; //creates a range between -10,000 and 10,000 for alphaVec and betaVec

  float dirRad = (float)dir * 71 / 4068; //convert dir to radians

  //calculate alpha and beta vectors
  alphaVec = (float)(scale * cos(dirRad));
  betaVec = (float)(scale * sin(dirRad));

  //record the signs of alphaVec and betaVec for later use
  bool alphaSign;
  if (alphaVec >= 0)
  {
    alphaSign = POSITIVE;
  }
  else
  {
    alphaSign = NEGATIVE;
  }
  bool betaSign;
  if (betaVec >= 0)
  {
    betaSign = POSITIVE;
  }
  else
  {
    betaSign = NEGATIVE;
  }
  //remove the signs from alpha and betaVec
  alphaVec = abs(alphaVec);
  betaVec = abs(betaVec);
  //Create a maximum value based on the larger of the two vectors (in magnitude, without signs)
  int max;
  if (alphaVec >= betaVec)
  {
    max = alphaVec;
  }
  else
  {
    max = betaVec;
  }
  //map vectors using the max value obtained and the given motor PWR
  alphaVec = map(alphaVec, 0, max, 0, PWR);
  betaVec = map(betaVec, 0, max, 0, PWR);
  //put the signs back
  if (alphaSign == NEGATIVE)
  {
    alphaVec = -(alphaVec);
  }
  if (betaSign == NEGATIVE)
  {
    betaVec = -(betaVec);
  }
  //Serial.print("AlphaVec: "); Serial.println(alphaVec);
  //Serial.print("BetaVec: "); Serial.println(betaVec);
  //Serial.print("AlphaSign: "); Serial.println(alphaSign);
  //Serial.print("BetaSign: "); Serial.println(betaSign);
  //Serial.print("Rot: "); Serial.println(rot);

  // Send Commands to motors
  motor(M1, constrain(betaVec + rot, -255, 255));
  motor(M2, constrain(alphaVec + rot, -255, 255));
  motor(M3, constrain(betaVec - rot, -255, 255));
  motor(M4, constrain(alphaVec - rot, -255, 255));
}

void motor(int motorID, int PWR) //produces low level signals to each individual motor drivers
{
  bool motorDirection;
  //Checking for valid PWR value
  if (PWR <= 255 && PWR >= 0) {
    motorDirection = true;
  }
  else if (PWR >= -255 && PWR < 0) {
    motorDirection = false;
    PWR = -(PWR);
  }
  else {
    SerialUSB.println("Invalid motor power");
    return; //error
  }
  //Handling Inverted motor control for second robot
  if (INVERT)
  {
    motorDirection = !motorDirection;
  }

  //Selecting a motor to run
  if (motorID == M1)
  {
    if (motorDirection == true) {
      digitalWrite(M1_IN1, LOW);
      digitalWrite(M1_IN2, HIGH);
    }
    else {
      digitalWrite(M1_IN1, HIGH);
      digitalWrite(M1_IN2, LOW);
    }
    analogWrite(M1_PWM, PWR);
  }
  else if (motorID == M2)
  {
    if (motorDirection == true) {
      digitalWrite(M2_IN1, LOW);
      digitalWrite(M2_IN2, HIGH);
    }
    else {
      digitalWrite(M2_IN1, HIGH);
      digitalWrite(M2_IN2, LOW);
    }
    analogWrite(M2_PWM, PWR);
  }
  else if (motorID == M3)
  {
    if (motorDirection == true) {
      digitalWrite(M3_IN1, LOW);
      digitalWrite(M3_IN2, HIGH);
    }
    else {
      digitalWrite(M3_IN1, HIGH);
      digitalWrite(M3_IN2, LOW);
    }
    analogWrite(M3_PWM, PWR);
  }
  else if (motorID == M4)
  {
    if (motorDirection == true) {
      digitalWrite(M4_IN1, LOW);
      digitalWrite(M4_IN2, HIGH);
    }
    else {
      digitalWrite(M4_IN1, HIGH);
      digitalWrite(M4_IN2, LOW);
    }
    analogWrite(M4_PWM, PWR);
  }
  else if (motorID == M5)
  {
    if (motorDirection == true) {
      digitalWrite(M5_IN1, LOW);
      digitalWrite(M5_IN2, HIGH);
    }
    else {
      digitalWrite(M5_IN1, HIGH);
      digitalWrite(M5_IN2, LOW);
    }
    analogWrite(M5_PWM, PWR);
  }
}
void brakeMotors()
{
  motor(1, 0);
  motor(2, 0);
  motor(3, 0);
  motor(4, 0);
  motor(5, 0);
}

void currentSensingTest() {
  drive(0, 0, 220);

  int current1 = analogRead(CS1);
  int current2 = analogRead(CS2);
  int current3 = analogRead(CS3);
  int current4 = analogRead(CS4);
  SerialUSB.println(current4);
  delay(50);

  float averageCurrent = (current1 + current2 + current3 + current4 - 8) / 4.0;

  // double to string
  char averageString[6];
  dtostrf(averageCurrent, 6, 2, averageString);

  updateScreen("Average Current:", averageString, "");
}

/*
   print to the screen the values from 3 sensors
   to find a threshold for white line
*/
void groundSensorCalibration() {
  readGroundSensor();

  char sensor0[6];
  char sensor3[6];
  char sensor7[6];

  dtostrf(groundSensor[GS0], 6, 2, sensor0);
  dtostrf(groundSensor[GS3], 6, 2, sensor3);
  dtostrf(groundSensor[GS7], 6, 2, sensor7);

  updateScreen(sensor0, sensor3, sensor7);
}

/*
   test mode for the ground sensor
*/
void groundSensorTest() {
  // updateScreen("GS Test", "", "");

  lineLeft = false;
  lineRight = false;
  lineFront = false;
  lineBack = false;

  readGroundSensor();

  SerialUSB.println(groundSensor[GS6]);

  //Serial.println(groundSensor[GS0]);
  if (groundSensor[GS0] > GS_THRESHOLD_WHITE || groundSensor[GS1] > GS_THRESHOLD_WHITE || groundSensor[GS2] > GS_THRESHOLD_WHITE)
  {
    lineFront = true;
    updateScreen("GS Test", "front", "");
  }
  if (groundSensor[GS3] > GS_THRESHOLD_WHITE || groundSensor[GS4] > GS_THRESHOLD_WHITE || groundSensor[GS5] > GS_THRESHOLD_WHITE || groundSensor[GS6] > GS_THRESHOLD_WHITE)
  {
    lineRight = true;
    updateScreen("GS Test", "", "right");
  }
  if (groundSensor[GS7] > GS_THRESHOLD_WHITE || groundSensor[GS8] > GS_THRESHOLD_WHITE || groundSensor[GS9] > GS_THRESHOLD_WHITE || groundSensor[GS10] > GS_THRESHOLD_WHITE)
  {
    lineLeft = true;
    updateScreen("GS Test", "", "left");
  }
  if (groundSensor[GS11] > GS_THRESHOLD_WHITE || groundSensor[GS12] > GS_THRESHOLD_WHITE || groundSensor[GS13] > GS_THRESHOLD_WHITE || groundSensor[GS14] > GS_THRESHOLD_WHITE)
  {
    lineBack = true;
    updateScreen("GS Test", "back", "");
  }

  if (!lineLeft && !lineRight && !lineFront && !lineBack) {
    updateScreen("GS Test", "no line", "");
  }
  /*
    while (digitalRead(B3) == 0) {
      STATE = 0;
    }
  */
}

/*
   Ground sensor checking
*/
void readGroundSensor()
{
  for (int i = 0; i < 15; i++)
  {
    groundSensor[i] = readGSMux(i);
  }
}
int readGSMux(int channel) //selects the given channel on the ground sensor multiplexor and reads the analog signal
{
  switch (channel) { //Uses truth table in multiplexor datasheet for selecting channels: http://www.mouser.com/ds/2/405/cd74hc4067-441121.pdf
    case 0:
      digitalWrite(GS_S0, LOW); digitalWrite(GS_S1, LOW); digitalWrite(GS_S2, LOW); digitalWrite(GS_S3, LOW);
      break;
    case 1:
      digitalWrite(GS_S0, HIGH); digitalWrite(GS_S1, LOW); digitalWrite(GS_S2, LOW); digitalWrite(GS_S3, LOW);
      break;
    case 2:
      digitalWrite(GS_S0, LOW); digitalWrite(GS_S1, HIGH); digitalWrite(GS_S2, LOW); digitalWrite(GS_S3, LOW);
      break;
    case 3:
      digitalWrite(GS_S0, HIGH); digitalWrite(GS_S1, HIGH); digitalWrite(GS_S2, LOW); digitalWrite(GS_S3, LOW);
      break;
    case 4:
      digitalWrite(GS_S0, LOW); digitalWrite(GS_S1, LOW); digitalWrite(GS_S2, HIGH); digitalWrite(GS_S3, LOW);
      break;
    case 5:
      digitalWrite(GS_S0, HIGH); digitalWrite(GS_S1, LOW); digitalWrite(GS_S2, HIGH); digitalWrite(GS_S3, LOW);
      break;
    case 6:
      digitalWrite(GS_S0, LOW); digitalWrite(GS_S1, HIGH); digitalWrite(GS_S2, HIGH); digitalWrite(GS_S3, LOW);
      break;
    case 7:
      digitalWrite(GS_S0, HIGH); digitalWrite(GS_S1, HIGH); digitalWrite(GS_S2, HIGH); digitalWrite(GS_S3, LOW);
      break;
    case 8:
      digitalWrite(GS_S0, LOW); digitalWrite(GS_S1, LOW); digitalWrite(GS_S2, LOW); digitalWrite(GS_S3, HIGH);
      break;
    case 9:
      digitalWrite(GS_S0, HIGH); digitalWrite(GS_S1, LOW); digitalWrite(GS_S2, LOW); digitalWrite(GS_S3, HIGH);
      break;
    case 10:
      digitalWrite(GS_S0, LOW); digitalWrite(GS_S1, HIGH); digitalWrite(GS_S2, LOW); digitalWrite(GS_S3, HIGH);
      break;
    case 11:
      digitalWrite(GS_S0, HIGH); digitalWrite(GS_S1, HIGH); digitalWrite(GS_S2, LOW); digitalWrite(GS_S3, HIGH);
      break;
    case 12:
      digitalWrite(GS_S0, LOW); digitalWrite(GS_S1, LOW); digitalWrite(GS_S2, HIGH); digitalWrite(GS_S3, HIGH);
      break;
    case 13:
      digitalWrite(GS_S0, HIGH); digitalWrite(GS_S1, LOW); digitalWrite(GS_S2, HIGH); digitalWrite(GS_S3, HIGH);
      break;
    case 14:
      digitalWrite(GS_S0, LOW); digitalWrite(GS_S1, HIGH); digitalWrite(GS_S2, HIGH); digitalWrite(GS_S3, HIGH);
      break;
    case 15:
      digitalWrite(GS_S0, HIGH); digitalWrite(GS_S1, HIGH); digitalWrite(GS_S2, HIGH); digitalWrite(GS_S3, HIGH);
      break;
  }
  delayMicroseconds(2); //Give time for the multiplexor to switch - can change to 1 microsecond
  int analogInput = analogRead(GS_ANI);
  return analogInput;
}

int selectedObject = 1; // 0 = ball, 1 = yellow goal, 2 = blue goal

void cameraTest() {
  // update blocks object
  uint16_t blocks;
  blocks = pixy.getBlocks();

  boolean seenLarger = false;
  if (blocks)
  {
    if (selectedObject == 0) {
      // filter objects for largest ball object
      for (int i = 0; i < blocks; i++) {
        if (pixy.blocks[i].signature == 1) {
          seenLarger = true;

          // distance angle formula for ball
          int ballXCoord = pixy.blocks[i].x - 160;
          int ballYCoord = pixy.blocks[i].y - 100;
          ballAngle = atan2(ballYCoord, ballXCoord) * 180 / M_PI;
          ballDistance = sqrt(ballYCoord * ballYCoord + ballXCoord * ballXCoord);
          // make angle positive
          if (ballAngle < 0) {
            ballAngle += 360;
          }

          // shift angle by objectOffset degrees so north is 0
          ballAngle += objectOffset;
          if (ballAngle >= 360) {
            ballAngle -= 360;
          }

          // double to string
          char angleString[6];
          char distanceString[6];
          dtostrf(ballAngle, 6, 2, angleString);
          dtostrf(ballDistance, 6, 2, distanceString);

          if ((ballAngle < capturedLeft || ballAngle > capturedRight) && ballDistance < capturedDistance) {
            updateScreen("Captured Ball", distanceString, angleString);
          } else {
            updateScreen("Ball", distanceString, angleString);
          }

          break;
        }
      }
    }
    else if (selectedObject == 1) {
      for (int i = 0; i < blocks; i++) {
        if (pixy.blocks[i].signature == 2) {
          seenLarger = true;

          // distance angle formula
          int yellowXCoord = pixy.blocks[i].x - 160;
          int yellowYCoord = pixy.blocks[i].y - 100;
          yellowAngle = atan2(yellowYCoord, yellowXCoord) * 180 / M_PI;
          yellowDistance = sqrt(yellowYCoord * yellowYCoord + yellowXCoord * yellowXCoord);

          // make angle positive
          if (yellowAngle < 0) {
            yellowAngle += 360;
          }

          // shift angle by objectOffset degrees so north is 0
          yellowAngle += objectOffset;
          if (yellowAngle >= 360) {
            yellowAngle -= 360;
          }

          // double to string
          char angleString[6];
          char distanceString[6];
          dtostrf(yellowAngle, 6, 2, angleString);
          dtostrf(yellowDistance, 6, 2, distanceString);

          updateScreen("yellow goal", distanceString, angleString);

          break;
        }
      }
    }
  } else {
    updateScreen("No Object", "", "");
  }

  // check buttons
  // when the down button is pressed
  if (digitalRead(B2) == 0)
  {
    selectedObject--;
    if (selectedObject == -1)
    {
      selectedObject = 2;
    }
    while (digitalRead(B2) == 0); // only switches when the button is released.
  }

  // when the up button is pressed.
  if (digitalRead(B1) == 0)
  {
    selectedObject++;
    if (selectedObject == 3)
    {
      selectedObject = 0;
    }
    while (digitalRead(B2) == 0); // only switches when the button is released.
  }

  while (digitalRead(B3) == 0) {
    STATE = 0;
  }

  if (!seenLarger) {
    updateScreen("No Selected", "Object", "");
  }
}

void updateBlock(int objectNumber) {
}

/*
    OLED controls:
    (1) updateScreen("line 1", "line 2", "line 3")
    (2) OLED.clearDisplay() clears screen (built in)
*/
void updateScreen(char input1[], char input2[], char input3[])
{
  OLED.firstPage();
  do
  {
    updateScreenHelper(input1, 0);
    updateScreenHelper(input2, 1);
    updateScreenHelper(input3, 2);
  } while (OLED.nextPage());
}

void drawOptions(char optionArray[][15], int selection, int numOptions)
{
  // OLED printing
  OLED.firstPage();
  do {
    OLED.drawBox(2, 22, 120, 18);
    if (selection == 0)
    {
      updateScreenHelper(options[numOptions - 1], 0);
    }
    else
    {
      updateScreenHelper(options[selection - 1], 0);
    }
    updateScreenHelper(options[selection], 1);
    if (selection == numOptions - 1)
    {
      updateScreenHelper(options[0], 2);
    }
    else
    {
      updateScreenHelper(options[selection + 1], 2);
    }
  } while (OLED.nextPage());
}
void updateScreenHelper(char input[], int line)
{
  //OLED.setCursor(2, 15 + (20 * line));
  OLED.drawStr(2, 15 + (20 * line), input);
}

/*
   Check the buttons and update variables
   Updates screen if up or down is selected
*/
void checkButtons () {
  // when the select button is pressed
  if (digitalRead(B3) == 0)
  {
    STATE = choice + 1; // STATE only changes when the button is pressed.
    while (digitalRead(B3) == 0);
  }

  // when the down button is pressed
  if (digitalRead(B2) == 0)
  {
    choice++; // the choice value increases by one
    if (choice == optionSize) // the option returns to itself
    {
      choice = 0;
    }
    while (digitalRead(B2) == 0); // only switches when the button is released.

    // displays different options for testing or playing.
    drawOptions(options, choice, optionSize);
  }

  // when the up button is pressed.
  if (digitalRead(B1) == 0)
  {
    choice--; // choice value decreases by one
    if (choice == -1)
    {
      choice = optionSize - 1;
    }
    while (digitalRead(B1) == 0);

    // displays different options for testing or playing.
    drawOptions(options, choice, optionSize);
  }
}

/*
   display values from the compass to determine if calibration is necessary
*/
void compassTest() {
  while (!(digitalRead(B3) == 0)) {
    float headDif = headingDif();
    if (headDif < -180) {
      headDif += 360;
    }
    headDif = -headDif;

    if (compassWorking == true)
    {
      // double to string
      char compassHeadingString[6];
      char compassHeadingDifString[6];
      dtostrf(headDif, 6, 2, compassHeadingDifString);

      updateScreen("Compass:", "", compassHeadingDifString);  // compassHeadingString
    }
    else
    {
      updateScreen("Compass:", "Not Found", "");
    }
  }
  while (!(digitalRead(B3) == 0));
}
float headingDif() {
  float dif = readCompass() - fieldNorth;
  if (dif > 180) {
    dif = dif - 360;
  }
  return dif;
}

/*
   calibrate the compass
*/
void compassCalibration()
{
  updateScreen("CMPS_Cal", "Start   ", "");

  brakeMotors();

  SerialUSB.println("Press to Start then rotate Robot 360 Degrees to Calibrate");
  //Run Calibration Sequence
  //send calibration start sequence and recieve confirmation bytes
  byte confirm1;
  byte confirm2;
  byte confirm3;
  Serial.write(startCalibration1);
  delay(10);
  if (Serial.available()) {
    confirm1 = Serial.read();
  }
  else {
    SerialUSB.println("Error - compass not available");
    updateScreen("CMPS_Cal", "Start   ", "error");
    return;
  }
  delay(100);
  Serial.write(startCalibration2);
  delay(10);
  if (Serial.available()) {
    byte confirm2 = Serial.read();
  }
  else
  {
    SerialUSB.println("Error - compass not available");
    updateScreen("CMPS_Cal", "Start   ", "error");
    return;
  }
  delay(100);
  Serial.write(startCalibration3);
  delay(10);
  if (Serial.available()) {
    confirm3 = Serial.read();
  }
  else {
    updateScreen("CMPS_Cal", "", "error");
    return;
  }
  if (confirm1 != 0x55 || confirm2 != 0x55 || confirm3 != 0x55)
  {
    SerialUSB.println("Error - Wrong confirmation code");
    SerialUSB.println(confirm1);
    SerialUSB.println(confirm2);
    SerialUSB.println(confirm3);
    updateScreen("CMPS_Cal", "", "error");
  }
  delay(100);

  byte confirm4;
  SerialUSB.println("Press Button to finish when you cannot get further LED flashes");
  updateScreen("CMPS_Cal", "Start   ", "rotate");

  delay(500);

  while (digitalRead(B3) == HIGH) {
  }
  Serial.write(endCalibration);
  delay(5);
  if (Serial.available()) {
    confirm4 = Serial.read();
  }
  else
  {
    SerialUSB.println("Error - compass not available");
    return;
  }
  SerialUSB.println("Calibration Complete");
  updateScreen("CMPS_Cal", "complete.   ", "");
  delay(1000);
  return;
}

/*
   Return Compass data
*/
float readCompass() //8 bit mode to avoid errors
{
reset:
  Serial.write(compassCommand);
  long waitStartTime = millis();
  while (Serial.available() < 1)
  {
    long currentTime = millis();
    if ((currentTime - waitStartTime) > 400)
    {
      SerialUSB.println("No Compass Plugged In");
      compassWorking = false;
      return -1;
    }
  }
  if (compassWorking == false) //The compass wasn't working previously but it came back on
  {
    //Reset Serial communication
    SerialUSB.println("Compass Reset");
    Serial.end();
    Serial.begin(9600);
    compassWorking = true;
    //Serial.print("Number of Bytes Available: ");
    //Serial.println(Serial.available());
    goto reset; //restart function
  }
  byte compass_raw = Serial.read();
  int mappedVal = map(compass_raw, 0, 255, 0, 3600); //maps the 8bit angle to 0-3600
  float processedVal = ((float)mappedVal) / (float)10;
  return processedVal;
}

double pursuitAngle2()
{
  // update blocks object
  uint16_t blocks;
  blocks = pixy.getBlocks();

  if (blocks)
  {
    // distance angle formula
    int ballXCoord = pixy.blocks[0].x - 160;
    int ballYCoord = pixy.blocks[0].y - 100;
    ballAngle = atan2(ballYCoord, ballXCoord) * 180 / M_PI;
    ballDistance = sqrt(ballYCoord * ballYCoord + ballXCoord * ballXCoord);
    // make angle positive
    if (ballAngle < 0) {
      ballAngle += 360;
    }

    // shift angle by objectOffset degrees so north is 0
    ballAngle += objectOffset;
    if (ballAngle >= 360) {
      ballAngle -= 360;
    }

    double idealAngleFromBall = (acceptableDistanceToBall / ballDistance) * (180 / M_PI); // rely less on distance
    double idealAngle1 = ballAngle + idealAngleFromBall;
    double idealAngle2 = ballAngle - idealAngleFromBall;

    if ((ballAngle < 10 || ballAngle > 355) && ballDistance < 57) {      // captured ball
      updateScreen("captuerd ball", "", "");
      brakeMotors();
    } else if (ballAngle < 10 || ballAngle > 350)                       // straight ahead
    {
      driveCompass(ballAngle, 250);
      // drivePID(ballAngle + 180, 250);
    }
    else if (ballAngle < 180)                                           // on the left
    {
      driveCompass(idealAngle1, 250);// 230 + map(ballDistance, 50, 120, 0, 25));
      // drivePID(idealAngle1 + 180, 240 + map(ballDistance, 50, 120, 0, 15));
    }
    else                                                                // on the right
    {
      driveCompass(idealAngle2, 250);// 230 + map(ballDistance, 50, 120, 0, 25));
      // drivePID(idealAngle2 + 180, 240 + map(ballDistance, 50, 120, 0, 15));
    }
  } else {
    // brakeMotors();
  }

  // avoidOutOfBounds();
}

double pursuitAngle()
{
  // update blocks object
  uint16_t blocks;
  blocks = pixy.getBlocks();

  if (blocks)
  {
    // distance angle formula
    int ballXCoord = pixy.blocks[0].x - 160;
    int ballYCoord = pixy.blocks[0].y - 100;
    ballAngle = atan2(ballYCoord, ballXCoord) * 180 / M_PI;
    ballDistance = sqrt(ballYCoord * ballYCoord + ballXCoord * ballXCoord);
    // make angle positive
    if (ballAngle < 0) {
      ballAngle += 360;
    }

    // shift angle by objectOffset degrees so north is 0
    ballAngle += objectOffset;
    if (ballAngle >= 360) {
      ballAngle -= 360;
    }

    if ((ballAngle < 10 || ballAngle > 355) && ballDistance < 57) {      // captured ball
      // driveCompass(0, 250);
      brakeMotors();
    } else if (ballAngle < 10 || ballAngle > 350)                       // straight ahead
    {
      drive(ballAngle, 0, 250);
    } else {
      float rot = ballAngle;
      if (ballAngle > 180) {
        rot -= 360;
      }
      if (rot > 0) {
        // driveAngle(ballAngle, 20 + map(rot, 0, 180, -60, 60), 250);

      } else {
        // driveAngle(ballAngle, -20 + map(rot, -180, 0, -60, 60), 250);
      }
      // SerialUSB.println(ballAngle);
    }
  } else {
    // brakeMotors();
  }

  // avoidOutOfBounds();
}

void returnNorth() {
  // get heading dif
  float headDif = headingDif();
  if (headDif < -180) {
    headDif += 360;
  }

  // drive accordingly
  if (headDif < -8) {
    drive(20, constrain(headDif, -150, -100), 30 + map(headDif, -180, 0, 0, 50));
  }  else if (headDif > 8) {
    drive(-20, constrain(headDif, 100, 150), 30 + map(headDif, 0, 180, 0, 50));
  } else {
    brakeMotors();
  }
}

// test dribbler motor and mechanism
void dribblerTest() {
  while (!(digitalRead(B3) == 0)) {
    // spin the dribbler with the dribblerSpeed speed
    motor(5, dribblerSpeed);

    if (digitalRead(B2) == 0) // decrease speed if down button is pressed
    {
      if (dribblerSpeed == 60) {
        dribblerSpeed = -61;
      } else if (dribblerSpeed > -255) // don't go bellow -255
      {
        dribblerSpeed--;
      }
    }

    if (digitalRead(B1) == 0) // increase speed if up button is pressed
    {
      if (dribblerSpeed == -60) {
        dribblerSpeed = 61;
      } else if (dribblerSpeed < 255) // don't go above 255
      {
        dribblerSpeed++;
      }
    }

    // update screen info
    char speedString[6];
    dtostrf((double)dribblerSpeed, 6, 2, speedString);
    updateScreen("speed up", speedString, "speed down");
  }

  while ((digitalRead(B3) == 0));
  STATE = 0;
  brakeMotors();
}

//dtostrf
char *dtostrf (double val, signed char width, unsigned char prec, char *sout) {
  asm(".global _printf_float");

  char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;
}
