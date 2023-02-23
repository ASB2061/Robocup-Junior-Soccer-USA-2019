// timer
int startTime, currentTime;
int ballStoppedCounter = 0;
int forwardHasBallTimer = 0;
int loopDelay = 22;
int ballStoppedTime = 5000;
boolean ballStopped = false;
int forwardLostBallCounter = 0;
int goalieHasBallCounter = 0;

// Compass
float fieldNorth = 0;
float rotOffset = 0; //Change this to point the robot at a different angle relative to fieldNorth
boolean compassWorking = true;
float derTerm = 5;
float lastCompassError = 0;

// White line variables
int groundSensor[15];
bool lineLeft = false;
bool lineRight = false;
bool lineFront = false;
bool lineBack = false;

// menu choice and switch statement
int choice = 0;     // show menu        // 0
char options[][15] = {"Offense yellow", // 1
                      "Defense yellow", // 2
                      "Camera Test",    // 3
                      "GS Test",        // 4
                      "GS Calib",       // 5
                      "CMPS Test",      // 6
                      "CMPS Calib",     // 7
                      "Current Sensin", // 8
                      "Dribbler Test",  // 9
                      "Drive To Ball",  // 10
                      "Pursuit Angle",  // 11
                      "Return North",   // 12
                      "Defense blue",   // 14
                      "Offense blue",   // 15
                     };
int optionSize = 14;
int STATE = 0; // what the robot is currently doing

// ball pursuit
double acceptableDistanceToBall = 40; // arc length of robot
double ballAngle;
double ballDistance;
double lastBallX;
double lastBallY;

// goals
double yellowAngle;
double yellowDistance;
double blueAngle;
double blueDistance;
double goalAngle;
int targetGoal;

// dribbler speed
int dribblerSpeed = 170;
