/***************************************************************
 MECH 534 - Computer-Based Modeling and Simulation - Project 2 - Rigid Body Simulation Part 2

 Berke Ataseven
 54326

 ***************************************************************/
//#define DEBUG

#include <Inventor/Win/SoWin.h>
#include <Inventor/Win/viewers/SoWinExaminerViewer.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <iostream>
#include <cmath>

constexpr auto PI = 3.14159265359;

const float objectHeight = 20; // Y - axis , a
const float objectWidth = 10; // X - axis , b
const float objectDepth = 5; // Z - axis , c

const float roomObjectRatio = 4;

SbVec3f absoluteVertexPositions[8] = { // Vertex coordinates calculated from the center of the object
    {objectWidth / 2, objectHeight / 2, objectDepth / 2},
    {objectWidth / 2, objectHeight / 2, -objectDepth / 2},
    {objectWidth / 2, -objectHeight / 2, objectDepth / 2},
    {objectWidth / 2, -objectHeight / 2, -objectDepth / 2},
    {-objectWidth / 2, objectHeight / 2, objectDepth / 2},
    {-objectWidth / 2, objectHeight / 2, -objectDepth / 2},
    {-objectWidth / 2, -objectHeight / 2, objectDepth / 2},
    {-objectWidth / 2, -objectHeight / 2, -objectDepth / 2},
};

float m = 1; // Mass of the object
float g = 0; // Gravitational acceleration
float Cd = 0.001; // Drag coefficent
float Cdm = 50; // Drag moment coefficent

float Ixx = m * (pow(objectHeight, 2) + pow(objectDepth, 2)) / 12;
float Iyy = m * (pow(objectWidth, 2) + pow(objectDepth, 2)) / 12;
float Izz = m * (pow(objectHeight, 2) + pow(objectWidth, 2)) / 12;
SbVec3f I = {Ixx, Iyy, Izz}; // Inertia matrix

float dt = 0.1; // Euler time steps. Needs to be as small as possible for an accurate simulation. Trade-off between performance and accuracy
float updateInterval = 0.01; // How frequently the frame is updated (Calculations are made).

// These variables are calculated at every time step
SbVec3f objectPosition;
SbVec3f objectPosition_prev;
SbVec3f objectAngularPosition;

SbVec3f objectVelocity;
SbVec3f objectAngularVelocity;

SbVec3f objectAcceleration;
SbVec3f objectAngularAcceleration;

SbVec3f objectMoment;
SbVec3f objectForce;

SbVec3f impactForce;

SbVec3f relativeVertexPositions[8];
bool isVertexCollided[8];
int collidingVertexIndex[8];
int numOfCollidingVertices = 0;
SbVec3f collidedSurfaceNormal[8];
bool reverseDirectionFlag[3] = {true, true, true};

SbVec3f calculatedCenter; // To constrain the movements of the rigid body within the room

SbVec3f userAppliedForce;

SoTransform *objectTransform;
SoTimerSensor* timer = new SoTimerSensor();

using namespace std;

SbVec3f calculate_transformed_point(SbVec3f pointToBeTransformed, SbVec3f originVector) { // Calculate the coordinates of vertices from the fxed reference frame.
    float result[4][1] = { {0}, {0}, {0}, {0} };
    static int r1 = 4;
    static int c2 = 1;
    static int c1 = 4;

    float a = objectAngularPosition[2]; // alpha
    float b = objectAngularPosition[1]; // beta
    float g = objectAngularPosition[0]; // gamma

    float transformMatrix[4][4] = {
        {cos(a) * cos(b), cos(a) * sin(b) * sin(g) - sin(a) * cos(g), cos(a) * sin(b) * cos(g) + sin(a) * sin(g), originVector[0]},
        {sin(a) * cos(b), sin(a) * sin(b) * sin(g) + cos(a) * cos(g), sin(a) * sin(b) * cos(g) - cos(a) * sin(g), originVector[1]},
        {-sin(b)        , cos(b) * sin(g)                           , cos(b) * cos(g)                           , originVector[2]},
        {0              , 0                                         , 0                                         , 1}
    };
    float P[4][1] = {
        {pointToBeTransformed[0]},
        {pointToBeTransformed[1]},
        {pointToBeTransformed[2]},
        {1}
    };
   // cout << g << endl;
    for (int i = 0; i < r1; ++i)
        for (int j = 0; j < c2; ++j)
            for (int k = 0; k < c1; ++k)
            {
                result[i][j] += transformMatrix[i][k] * P[k][j];
            }
    return SbVec3f(result[0][0], result[0][1], result[0][2]);
}

void calculate_vertex_positions() { // Vertex positions is needed for the collision check
    for (int i = 0; i < 8; i++) {
        relativeVertexPositions[i] = calculate_transformed_point(absoluteVertexPositions[i], objectPosition);
    }
}

void check_for_collisions() { // Check if any of the vertices are outside of the room. If so, save them in 'isVertexCollided'
    static const float roomBoundary = objectHeight * roomObjectRatio / 2; // 20 * 5 / 2
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 3; j++) {
            if (relativeVertexPositions[i][j] <= -roomBoundary || relativeVertexPositions[i][j] >= roomBoundary) { // These can never be true at the same time
                isVertexCollided[i] = true;
                collidingVertexIndex[i] = j;
                collidedSurfaceNormal[i][j] = relativeVertexPositions[i][j] <= -roomBoundary ? 1 : -1;
                if (reverseDirectionFlag[j]) {
#ifdef DEBUG
                    cout << "Reversing velocity: " << j << endl;
#endif // DEBUG                    
                    objectVelocity[j] *= -0.9;
                    reverseDirectionFlag[j] = false;
                }
            }
        }
        if (isVertexCollided[i]) numOfCollidingVertices++;
    }
}

void calculate_forces() {
    static SbVec3f weight = { 0, m * g, 0 };
    static SbVec3f dragForce;    
    
    // Impact Force
    // Force = m * v / dt
    //for (int i = 0; i < 8; i++) {
    //    if (isVertexCollided[i]) {
           //objectForce += (collidedSurfaceNormal[i] * objectVelocity[collidingVertexIndex[i]] / numOfCollidingVertices );
    //    }
    //}
        
    // Drag force
    SbVec3f dummy = objectVelocity; // Dummy vector to keep the original value of 'objectVelocity'
    dummy.normalize();
    dragForce = (dummy * objectVelocity.sqrLength() * Cd); // F = Cd * V^2
    dragForce.negate(); // Reverse the direction of the drag.

     objectForce += dragForce - weight + userAppliedForce;
}

void calculate_moments() {
    static SbVec3f dragMoment;
    static SbVec3f weight = { 0, m * g, 0 };
    
    // Drag force
    SbVec3f dummy = objectAngularVelocity; // Dummy vector to keep the original value of 'objectAngularVelocity'
    dummy.normalize();
    dragMoment = (dummy * objectAngularVelocity.sqrLength() * Cdm); // F = Cd * V^2
    dragMoment.negate(); // Revervse the direction of the drag. 

    // Apply the user force to the corner of the body
    if (!(userAppliedForce[0] == 0 && userAppliedForce[1] == 0 && userAppliedForce[2] == 0)) {
        SbVec3f r = relativeVertexPositions[0] - objectPosition;
        objectMoment += r.cross(userAppliedForce / 5);
    }

    for (int i = 0; i < 8; i++) {
        if (isVertexCollided[i]) {
            SbVec3f r = relativeVertexPositions[i] - objectPosition;
            objectMoment += r.cross(collidedSurfaceNormal[i] * objectVelocity.length());
            objectMoment += (-r).cross(-weight);
        }
    }    
    objectMoment += dragMoment;
}

void constrain_position() {
    static const float roomBoundary = objectHeight * roomObjectRatio / 2 - 0.01; // 20 * 5 / 2
    SbVec3f objectPosition_new;

    for (int i = 0; i < 8; i++) {
        if (isVertexCollided[i]) {
            boolean isBigger = relativeVertexPositions[i][collidingVertexIndex[i]] >= roomBoundary;
            switch (collidingVertexIndex[i]) {
            case 0:
                objectPosition_new = isBigger ?
                    calculate_transformed_point(-absoluteVertexPositions[i],SbVec3f(roomBoundary, relativeVertexPositions[i][1], relativeVertexPositions[i][2])) :
                    calculate_transformed_point(-absoluteVertexPositions[i], SbVec3f(-roomBoundary, relativeVertexPositions[i][1], relativeVertexPositions[i][2]));
                break;
            case 1:
                objectPosition_new = isBigger ?
                    calculate_transformed_point(-absoluteVertexPositions[i], SbVec3f(relativeVertexPositions[i][0], roomBoundary, relativeVertexPositions[i][2])) :
                    calculate_transformed_point(-absoluteVertexPositions[i], SbVec3f(relativeVertexPositions[i][0], -roomBoundary, relativeVertexPositions[i][2]));
                break;
            case 2:
                objectPosition_new = isBigger ?
                    calculate_transformed_point(-absoluteVertexPositions[i], SbVec3f(relativeVertexPositions[i][0], relativeVertexPositions[i][1], roomBoundary)) :
                    calculate_transformed_point(-absoluteVertexPositions[i], SbVec3f(relativeVertexPositions[i][0], relativeVertexPositions[i][1], -roomBoundary));
                break;
            }
            objectPosition = objectPosition_new;
            objectTransform->translation.setValue(objectPosition);
        }
    }
}

void euler_to_quaternion(float yaw, float pitch, float roll, float(&quaternion_vector)[4]) { // yaw (Z), pitch (Y), roll (X)
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    quaternion_vector[0] = sr * cp * cy - cr * sp * sy; // qx
    quaternion_vector[1] = cr * sp * cy + sr * cp * sy; // qy
    quaternion_vector[2] = cr * cp * sy - sr * sp * cy; // qz
    quaternion_vector[3] = cr * cp * cy + sr * sp * sy; // qw
}

void update_frame_callback(void* data, SoSensor* timer) {
    calculate_vertex_positions();
    check_for_collisions();
    calculate_forces();
    calculate_moments();
    
    objectAcceleration = objectForce / m; // F = m.a
    objectVelocity += objectAcceleration * dt; // Eulers method
    objectPosition += objectVelocity * dt; // Eulers method
    constrain_position();    // Comment to disable position constraining
    objectTransform->translation.setValue(objectPosition);

    for(int i=0; i < 3 ;i++) objectAngularAcceleration[i] = objectMoment[i] / I[i];
    objectAngularVelocity += objectAngularAcceleration * dt; // Eulers method
    objectAngularPosition += objectAngularVelocity * dt; // Eulers method
    float q[4];
    euler_to_quaternion(objectAngularPosition[2], objectAngularPosition[1], objectAngularPosition[0], q); // Convert euler angles to quaternion
    objectTransform->rotation.setValue(q[0], q[1], q[2], q[3]);


    // Set the reverse direction flag to true if the object has passed the origin
    for (int i = 0; i < 3; i++)
        if (objectPosition_prev[i] * objectPosition[i] <= 0)
            reverseDirectionFlag[i] = true;

#ifdef DEBUG
    printf_s("%.2f\t%.2f\t%.2f\t%d\n",objectPosition[1], objectVelocity[1], objectAcceleration[1], numOfCollidingVertices);
#endif

    memset(isVertexCollided, 0, sizeof(isVertexCollided)); // Set collisions to false
    numOfCollidingVertices = 0; // Set number of colliding vertices to 0
    userAppliedForce = SbVec3f(0, 0, 0); // Set the user applied force to zero

    objectPosition_prev = objectPosition;

    for (int i = 0; i < 3; i++) { // Set the forces and moments to zero. They are calculated from scratch in the next loop
        objectForce[i] = 0;
        objectMoment[i] = 0;
    }
}

void key_press_callback(void* userData, SoEventCallback* eventCB) {
    SoSelection* selection = (SoSelection*)userData;
    const SoEvent* event = eventCB->getEvent();
    
    if (SO_KEY_PRESS_EVENT(event, SoKeyboardEvent::A)) { // Generate a random user force
        std::cout << "The rigid body is pushed from its corner.\n";
        userAppliedForce = SbVec3f(rand() % 100, rand() % 100, rand() % 100);
    }
    if (SO_KEY_PRESS_EVENT(event, SoKeyboardEvent::S)) { // Set the velocities to zero
        objectVelocity = { 0, 0, 0 };
        objectAngularVelocity = { 0, 0, 0 };
        std::cout << "Rigid body stopped.\n";
    }     
    if (SO_KEY_PRESS_EVENT(event, SoKeyboardEvent::R)) { // Set the positions and velocities to zero
        std::cout << "Simulation is reset.\n";
        objectPosition = { 0, 0, 0 };
        objectAngularPosition = { 0, 0, 0 };
        objectVelocity = { 0, 0, 0 };
        objectAngularVelocity = { 0, 0, 0 };
    }
    if (SO_KEY_PRESS_EVENT(event, SoKeyboardEvent::SPACE)) { // Pressing spacebar pauses/continues the simulation
        if (!timer->isScheduled()) {
            timer->schedule();
           std::cout << "Simulation continued.\n";
         }
        else {
            std::cout << "Simulation paused.\n";
           timer->unschedule();
        }
        eventCB->setHandled();
    }
}

int main(int, char ** argv)
{
    srand(time(NULL));
#ifdef START_WITH_RANDOM_MOVEMENT
    for (int i = 0; i < 3; i++) { // Start the movement with random
        objectMoment[i] = rand() % 100 + 100;
        objectForce[i] = rand() % 50;
    }
#endif
  	HWND window = SoWin::init(argv[0]);
  	if ( window == NULL ) 
  		exit(1);

    SoWinExaminerViewer *viewer = new SoWinExaminerViewer(window);
    
    SoEventCallback* myEventCB = new SoEventCallback;	// Keyboard event listener
    myEventCB->addEventCallback(SoKeyboardEvent::getClassTypeId(), key_press_callback);
   
    // Scene
    SoSeparator* myScene = new SoSeparator();

    // Initialize the elements of the "room"
    SoSeparator* roomSeperator = new SoSeparator();
    SoMaterial* roomMaterial = new SoMaterial();
    roomMaterial->ambientColor = { 0.8, 0.8, 0.1 };
    roomMaterial->diffuseColor = { 0.9, 0.9, 0.2 };
    roomMaterial->transparency = 0.9;    
    SoCube* roomBox = new SoCube();
    roomBox->height = objectHeight * roomObjectRatio;
    roomBox->width = roomBox->height;
    roomBox->depth = roomBox->height;

    // Initialize the elements of the "rigid body"
    SoSeparator* objectSeperator = new SoSeparator();
    objectTransform = new SoTransform;
    SoMaterial* objectMaterial = new SoMaterial();
    objectMaterial->ambientColor = { 0.1, 0.8, 0.8 };
    objectMaterial->diffuseColor = { 0.1, 0.8, 0.8 };
    SoCube* objectBox = new SoCube();
    objectBox->height = objectHeight;
    objectBox->width = objectWidth;
    objectBox->depth = objectDepth;
    
    // Create the room
    roomSeperator->addChild(roomMaterial);
    roomSeperator->addChild(roomBox);
   
    // Create the object
    objectSeperator->addChild(objectTransform);
    objectSeperator->addChild(objectMaterial);
    objectSeperator->addChild(objectBox);

    myScene->addChild(myEventCB); // Add the keyboard listener as a child    
    myScene->addChild(objectSeperator); // Add the object to the scene
    myScene->addChild(roomSeperator); // Add the room to the scene

    cout << "HOW TO USE\n==========" << endl;
    cout << "Press 'A' to apply a random force.\n";
    cout << "Press 'S' to stop the velocity.\n";
    cout << "Press 'R' to reset the simulation.\n";
    cout << "Press spacebar to pause/continue the simulation.\n\n";

    // Set the calback function to update the frame with fixed intervals
    timer->setFunction(update_frame_callback);
    timer->setInterval(updateInterval);
    if (!timer->isScheduled()) timer->schedule();
	
    // Display the program
  	viewer->setSceneGraph(myScene);
  	viewer->setTitle("MECH 534 - Project 2 - Part 2");
  	viewer->show();

  	SoWin::show(window);
  	SoWin::mainLoop();

  	return 0;
}
