#include <Arduino.h>
#include<cppQueue.h>
#include<TimerOne.h>
#include<limits.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// The code logic was constructed using the IEEE lecture on micromouse.                                                                                          //
// it is set to the rules of mirco-mouse competition : https://attend.ieee.org/r2sac-2020/wp-content/uploads/sites/175/2020/01/MicroMouse_Rules_2020.pdf         //
// It uses a maze solving algothrim called flood fill algothrim.                                                                                                 //
// U (up) D(down) R(right) L(left) point orientation movement and N(North) S(south) W(west) E(East) local orientation of the robot                               //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Written by Sai Manohar Vangara

//IR sensor pins 
int const RIR=11;
int const LIR=12;

// Maze size in this case 4x4
const int ROWS = 4; 
const int COLS = 4;

// unit per grid in cm
const int unit=26;

//turing steps
const int turn=6;

//Ultra sonic sensor settings
const int tri = 7; 
const int echoPin = 4;

// Wheel dia in mm

const float wheeldiameter=66.10l;

// loop() control flag parameter
int T=0;

//left motor//
const int EEP=13;
const int Int1=9;
const int Int2=10;

// Right motor//
const int Int3=5;
const int Int4=6;

//Encoder settings
const int Motor1=2;
const int Motor2=3;
const float diskslots=20.00;

//ISR counters for motors
volatile int count1=0;
volatile int count2=0;

//Robot direction
char orientation ='N';

// Destination
const int destinationX = 2;
const int destinationY = 2;

// Current Robot position
int currentx=ROWS-1;
int currenty=0;

// Directions for moving up, down, left, and right for assiging
int dRow[] = {-1, 1, 0, 0};
int dCol[] = {0, 0, -1, 1};

// Creating a point structure for the queue
struct Point {
    int x, y;
};

// Creating a Queue structure
struct Queue {
    Point points[ROWS * COLS];
    int size = 0;
    int front = 0;
    void push(Point p) {
        if (size < ROWS * COLS) {
            points[size++] = p;
        }
    }
    Point pop() {
        return points[front++];
    }

    bool isEmpty() {
        return front == size;
    }
};

//object oriented programming to store flood fill cell values, detectednm walls 

struct Maze {
    int horizontalWalls[ROWS-1][COLS]; 
    int verticalWalls[ROWS][COLS-1];
    int distances[ROWS][COLS];
};

Maze maze;

// initilizing the Maze with goal being zero and other being blank {}

void initializeMaze() {
    // Set horizontal and vertical walls to zero 
    for (int i = 0; i < ROWS-1; ++i) {
        for (int j = 0; j < COLS; ++j) {
            maze.horizontalWalls[i][j] = 0;
        }
    }
    
    for (int i = 0; i < ROWS; ++i) {
        for (int j = 0; j < COLS-1; ++j) {
            maze.verticalWalls[i][j] = 0;
        }
    }
    // Set all distances to blank
    for (int i = 0; i < ROWS; ++i) {
        for (int j = 0; j < COLS; ++j) {
            maze.distances[i][j] = {};
        }
    }
}

void markHorizontalWall(int row, int col) {
    // Boundary conditions
    if (row >= 0 && row < ROWS - 1 && col >= 0 && col < COLS) {
        maze.horizontalWalls[row][col] = 1;
    }
}
void markVerticalWall(int row, int col) {
   
    if (row >= 0 && row < ROWS && col >= 0 && col < COLS - 1) {
        maze.verticalWalls[row][col] = 1;
    }
}


// Assigns the flood fill values taking walls into consideration, intially maze with no walls is assmued
void floodFill(int startRow, int startCol) {
    Queue q;

    // Initialize distances with a placeholder value
    for (int i = 0; i < ROWS; ++i)
        for (int j = 0; j < COLS; ++j)
            maze.distances[i][j] = -1;

    // Start the flood fill from the specified point
    q.push({startRow, startCol});
    maze.distances[startRow][startCol] = 0;

    while (!q.isEmpty()) {
        Point cell = q.pop();

        // Explore adjacent cells
        for (int i = 0; i < ROWS; i++) {
            int adjRow = cell.x + dRow[i];
            int adjCol = cell.y + dCol[i];

            // Check if the adjacent cell is within the maze bounds
            if (adjRow >= 0 && adjRow < ROWS && adjCol >= 0 && adjCol < COLS) {
                // Check for walls before updating the distance
                bool hasWall = false;
                if (i == 0 && maze.horizontalWalls[cell.x - 1][cell.y] == 1) hasWall = true; // Up
                if (i == 1 && maze.horizontalWalls[cell.x][cell.y] == 1) hasWall = true; // Down
                if (i == 2 && maze.verticalWalls[cell.x][cell.y - 1] == 1) hasWall = true; // Left
                if (i == 3 && maze.verticalWalls[cell.x][cell.y] == 1) hasWall = true; // Right

                if (!hasWall && maze.distances[adjRow][adjCol] == -1) {
                    q.push({adjRow, adjCol});
                    maze.distances[adjRow][adjCol] = maze.distances[cell.x][cell.y] + 1;
                }
            }
        }
    }
}

// Detecting wall
bool deWall() {
  const int numReadings = 5; 
  int totalDistance = 0;
  int distance;

  for (int i = 0; i < numReadings; i++) {
    digitalWrite(tri, LOW);
    delayMicroseconds(2);
    digitalWrite(tri, HIGH);
    delayMicroseconds(10); 
    digitalWrite(tri, LOW);
    
    long duration = pulseIn(echoPin, HIGH);
    distance = duration * 0.034 / 2;

    totalDistance += distance;
    delay(10); 
  }
  int averageDistance = totalDistance / numReadings;
  if (averageDistance < 10) {
    return true; // Wall detected
  } else {
    return false; // No wall detected
  }
}

bool rightIR(){
  if (digitalRead(11)==LOW)
  {
    return true;// Wall detected
  }else{return false ;}// No wall detected
  
}

bool leftIR(){
  if (digitalRead(12)==LOW)
  {
    return true; // Wall detected;
  }else{return false;} // No wall detected
  
}


// Function to detect walls and update the maze structure
bool detectwithUltrasonic() {
    bool wallDetected = deWall(); 

    if (wallDetected) {
    
        // Update horizontal and vertical walls based on the robot's current position and orientation
        switch (orientation) {
            case 'N': // Facing North
                if (currentx > 0) {
                    markHorizontalWall(currentx - 1, currenty); // Wall is in front
                }
                break;
            case 'E': // Facing East
                if (currenty < COLS - 1) {
                    markVerticalWall(currentx, currenty); // Wall is to the right
                }
                break;
            case 'S': // Facing South
            
                if (currentx < ROWS - 1) {
                    markHorizontalWall(currentx, currenty); // Wall is behind
                }
                break;
            case 'W': // Facing West
            
                if (currenty > 0) {
                    markVerticalWall(currentx, currenty-1 ); // Wall is to the left
                }
                break;
        }
        // Re-run the flood-fill algorithm to update distances after wall update
        floodFill(destinationX, destinationY); 
        delay(200);
    return true;
    }
  return false;
}

// Detects the walls on the sides  using IR sensors
void detectWallwithIR() {
    // Detect right wall
 if (rightIR()) {
        switch (orientation) {
            case 'N':
                if (currenty < COLS - 1) {
                    maze.verticalWalls[currentx][currenty] = 1; // Wall on the right (East)
                }
                break;
            case 'E':
                if (currentx < ROWS - 1) {
                    maze.horizontalWalls[currentx][currenty] = 1; // Wall below (South)
                }
                break;
            case 'S':
                if (currenty > 0) {
                    maze.verticalWalls[currentx][currenty - 1] = 1; // Wall on the left (West)
                }
                break;
            case 'W':
                if (currentx > 0) {
                    maze.horizontalWalls[currentx - 1][currenty] = 1; // Wall above (North)
                }
                break;
        }
    }

    // Detect left wall
    if (leftIR()) {
        switch (orientation) {
            case 'N':
                if (currenty > 0) {
                    maze.verticalWalls[currentx][currenty - 1] = 1; // Wall on the left 
                }
                break;
            case 'E':
                if (currentx > 0) {
                    maze.horizontalWalls[currentx - 1][currenty] = 1; // Wall above 
                }
                break;
            case 'S':
                if (currenty < COLS - 1) {
                    maze.verticalWalls[currentx][currenty] = 1; // Wall on the right 
                }
                break;
            case 'W':
                if (currentx < ROWS - 1) {
                    maze.horizontalWalls[currentx][currenty] = 1; // Wall below 
                }
                break;
        }
    }

    // Update the flood fill values
    floodFill(destinationX, destinationY);
}





// Encoder intercept service routine
void ISR_count1() {
  count1++;
}

void ISR_count2() {
  count2++;
}

// converts cm to no of disk-pulses
int cm(float cm){
    float cir=(wheeldiameter*3.14)/10;
  float cm_step=cir/diskslots;
  float fresult=cm/cm_step;
  int step=(int)fresult;
  return step;
}

// forward motion 
void MoveForward(int steps)
{

  count1=0;
  count2=0;
  digitalWrite(EEP,HIGH);
  while (steps > count1 && steps > count2)
  {
    if (steps > count1)
    {
      digitalWrite(Int1,LOW);
    digitalWrite(Int2,HIGH);
    }
    if (steps>count2)
    {
      digitalWrite(Int3,LOW);
    digitalWrite(Int4,HIGH);
   
    }
    }
    digitalWrite(Int1,0);
 digitalWrite(Int2,0);
  digitalWrite(Int3,0);
 digitalWrite(Int4,0);
  count1=0;
  count2=0;
}

//back motion
void MoveBack(int steps)
{

  count1=0;
  count2=0;
  digitalWrite(EEP,HIGH);
  while (steps > count1 && steps > count2)
  {
    if (steps > count1)
    {
    digitalWrite(Int1,HIGH);
    digitalWrite(Int2,LOW);
    }
    if (steps>count2)
    {
    digitalWrite(Int3,HIGH);
    digitalWrite(Int4,LOW);
    }
    }
digitalWrite(Int1,LOW);
digitalWrite(Int2,LOW);
digitalWrite(Int3,LOW);
digitalWrite(Int4,LOW);
  count1=0;
  count2=0;
}

//  1 unit forward  motion
void forward(){
  MoveForward(cm(unit));
}

//  1 unit back motion
void back(){
  MoveBack(cm(unit));
}

// left motion
void Leftt()
{

  count1=0;
  count2=0;
  int steps =turn;
  digitalWrite(EEP,HIGH);
  while (steps > count1 && steps > count2)
  {
    if (steps > count1)
    {
       digitalWrite(Int1,HIGH);
    digitalWrite(Int2,LOW);
    }
    if (steps>count2)
    {
      digitalWrite(Int3,LOW);
    digitalWrite(Int4,HIGH);
    }
    }
  digitalWrite(Int1,LOW);
 digitalWrite(Int2,LOW);
digitalWrite(Int3,LOW);
 digitalWrite(Int4,LOW);
  count1=0;
  count2=0;
  
  if (orientation == 'N') orientation = 'W';
    else if (orientation == 'W') orientation = 'S';
    else if (orientation == 'S') orientation = 'E';
    else if (orientation == 'E') orientation = 'N';  
}

// left and forward logic depending on the wall
void left() {
  Leftt(); // Turn left
   delay(1000); // Short delay for stabilization
    forward(); // Move forward if no wall
  } 

//right motion
void Rightt()
{

  count1=0;
  count2=0;
  int steps =turn;
  digitalWrite(EEP,HIGH);
  while (steps > count1 && steps > count2)
  {
    if (steps > count1)
    {
       digitalWrite(Int1,LOW);
    digitalWrite(Int2,HIGH);
    }
    if (steps>count2)
    {
      digitalWrite(Int3,HIGH);
    digitalWrite(Int4,LOW);
    }
    }
    digitalWrite(Int1,LOW);
   digitalWrite(Int2,LOW);
  digitalWrite(Int3,LOW);
  digitalWrite(Int4,LOW);
  count1=0;
  count2=0;
  
 if (orientation == 'N') orientation = 'E';
    else if (orientation == 'E') orientation = 'S';
    else if (orientation == 'S') orientation = 'W';
    else if (orientation == 'W') orientation = 'N';
  }

// right and forward logic depending on the wall
void right() {
  Rightt(); // Turn right
  delay(1000); // Short delay for stabilizatio
    forward(); // Move forward if no wall
  } 


// Navigate(move the robot) based on current orientation and desired direction
void Navigate(char H) {
    switch (orientation) {
        case 'N': // When facing North
            if (H == 'U') forward();
            else if (H == 'D') back();
            else if (H == 'L') left();
            else if (H == 'R') right();
            break;
        case 'E': // When facing East
            if (H == 'D') right();
            else if (H == 'U') left();
            else if (H == 'L') back();
            else if (H == 'R') forward();
            break;
        case 'S': // When facing South
            if (H == 'U') back();
            else if (H == 'D') forward();
            else if (H == 'L') right();
            else if (H == 'R') left();
            break;
        case 'W': // When facing West
            if (H == 'U') right();
            else if (H == 'D') left();
            else if (H == 'L') forward();
            else if (H == 'R') back();
            break;
        default:
            break;
    }
}

char getDirectionToLowestNeighbor(int currentX, int currentY) {
    int lowestValue = INT_MAX;
    char direction = 'G'; // G for invalid direction

    
    if (currentX > 0 && maze.horizontalWalls[currentX - 1][currentY] == 0 && maze.distances[currentX - 1][currentY] < lowestValue) {
        lowestValue = maze.distances[currentX - 1][currentY];
        direction = 'U'; // Up
    }

   
    if (currentY > 0 && maze.verticalWalls[currentX][currentY - 1] == 0 && maze.distances[currentX][currentY - 1] < lowestValue) {
        lowestValue = maze.distances[currentX][currentY - 1];
        direction = 'L'; // Left
    }

    
    if (currentY < COLS - 1 && maze.verticalWalls[currentX][currentY] == 0 && maze.distances[currentX][currentY + 1] < lowestValue) {
        lowestValue = maze.distances[currentX][currentY + 1];
        direction = 'R'; // Right
    }

    
    if (currentX < ROWS - 1 && maze.horizontalWalls[currentX][currentY] == 0 && maze.distances[currentX + 1][currentY] < lowestValue) {
        lowestValue = maze.distances[currentX + 1][currentY];
        direction = 'D'; // Down
    }

    return direction; 
}


void Updateposition(char D) {
    switch (orientation) {
        case 'N': // North
            if (D == 'U' && currentx >= 0) currentx--;
            else if (D == 'D' && currentx < ROWS - 1) currentx++;
            break;
        case 'E': // East
            if (D == 'R' && currenty < COLS - 1) {currenty++;}
            else if (D == 'L' && currenty >0) {currenty--;}
            break;
        case 'S': // South
            if (D == 'U' && currentx < ROWS - 1) currentx--;
            else if (D == 'D' && currentx >= 0) currentx++;
            break;
        case 'W': // West
            if (D == 'R' && currenty >= 0) {currenty--;}
            else if (D == 'L' && currenty < COLS - 1 ) {currenty++;}
            break;
    }
}


void setup() {

Serial.begin(9600);

//initialize the interrupt timer
Timer1.initialize(1000000);

//attaching interrupts
attachInterrupt( digitalPinToInterrupt(Motor1),ISR_count1, RISING);
attachInterrupt(digitalPinToInterrupt (Motor2),ISR_count2, RISING);

//Initilization of the pins for motor driver
pinMode(Int1,OUTPUT);
pinMode(Int2,OUTPUT);
pinMode(Int3,OUTPUT);
pinMode(Int4,OUTPUT);
pinMode(EEP,OUTPUT);
pinMode(RIR,INPUT);
pinMode(LIR,INPUT);
// Initilize pins for the ultra sonic senosor 
pinMode(tri, OUTPUT);
pinMode(echoPin, INPUT);

// Generating a queue  for the flood fill matrix   
  initializeMaze();

// Starting flood fill from destination points 
floodFill(destinationX,destinationY);
}

 void loop() {

    while (T == 0) {
        
        if (currentx == destinationX && currenty == destinationY) {
            T++;
            break; // break if the current position and destination 
        }

    detectwithUltrasonic(); // detects walls at front with Ultrasonic sensor updates the maze
    detectWallwithIR(); // detects walls side of the robot with IR sensor updates the maze
   
        // Get the best accessible direction based on updated maze
        char Direction = getDirectionToLowestNeighbor(currentx, currenty);
        
      
        Navigate(Direction); //Move the robot in the direction 
       Updateposition(Direction); // Update position after moving
        delay(1000); 
    }
}


