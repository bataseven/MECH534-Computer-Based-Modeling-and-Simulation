/***************************************************************
 MECH 534 - Computer-Based Modeling and Simulation - HW6

 Berke Ataseven
 54326

 ***************************************************************/

constexpr auto PI = 3.14159265359;

#include <Inventor/Win/SoWin.h>
#include <Inventor/Win/viewers/SoWinExaminerViewer.h>

#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoCylinder.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <iostream>

using namespace std;

static float vertices[8][3] =
{
   { 0.0,  0.0,  0.0},
   { 1.0,  0.0,  0.0},
   { 1.0,  1.0,  0.0},
   { 0.0,  1.0,  0.0},
   { 0.0,  0.0,  1.0},
   { 1.0,  0.0,  1.0},
   { 1.0,  1.0,  1.0},
   { 0.0,  1.0,  1.0},
};


static int indices[48] =
{
   // Define the connections clockwise so that the surface normals of the triangles are correct

   2, 1, 0, -1, // FRONT
   0, 3, 2, -1, // FRONT

   4, 3, 0, -1, //LEFT
   4, 7, 3, -1, //LEFT

   7, 2, 3, -1, //TOP
   7, 6, 2, -1, //TOP

   5, 1, 2, -1, //RIGHT
   2, 6, 5, -1, //RIGHT  

   5, 6, 7, -1, //BACK
   5, 7, 4, -1, //BACK

   5, 4, 0, -1, //BOTTOM
   0, 1, 5, -1, //BOTTOM
};


static float colors[12][3] =
{
    // Rubik's cube colors
   {0,1,0}, {0,1,0}, // FRONT - GREEN
   {1,0,0}, {1,0,0}, // LEFT - RED
   {1,1,1}, {1,1,1}, // TOP - WHITE
   {1,0.65,0}, {1,0.65,0}, // RIGHT - ORANGE
   {1,1,0}, {1,1,0}, // BACK - YELLOW
   {0,0,1}, {0,0,1}, // BOTTOM - BLUE
};


SoSeparator* makeIndexedTriangularCube()
{
    SoSeparator* output = new SoSeparator;
    output->ref();

    // Colors
    SoMaterial* myMaterial = new SoMaterial;
    myMaterial->diffuseColor.setValues(0, 12, colors);
    output->addChild(myMaterial);
    SoMaterialBinding* myMaterialBinding = new SoMaterialBinding;
    myMaterialBinding->value = SoMaterialBinding::PER_FACE;
    output->addChild(myMaterialBinding);

    SoCoordinate3* myCoordinate = new SoCoordinate3;
    myCoordinate->point.setValues(0, 8, vertices);
    output->addChild(myCoordinate);


    SoIndexedFaceSet* myFaceSet = new SoIndexedFaceSet;
    myFaceSet->coordIndex.setValues(0, 48, indices);
    output->addChild(myFaceSet);

    output->unrefNoDelete();
    return output;
}


int main(int, char** argv)
{
    //============== PART A ========== //

    HWND window = SoWin::init(argv[0]);
    if (window == NULL)
        exit(1);

    SoWinExaminerViewer* viewer = new SoWinExaminerViewer(window);

   // Robot with legs and arms
   
   // Construct parts for legs (thigh, calf and foot)
    SoCube* thigh = new SoCube;
    thigh->width = 1.2;
    thigh->height = 2.2;
    thigh->depth = 1.1;

    SoTransform* calfTransform = new SoTransform;
    calfTransform->translation.setValue(0, -2.25, 0.0);

    SoCube* calf = new SoCube;
    calf->width = 1;
    calf->height = 2.2;
    calf->depth = 1;

    SoTransform* footTransform = new SoTransform;
    footTransform->translation.setValue(0, -2, .5);

    SoCube* foot = new SoCube;
    foot->width = 0.8;
    foot->height = 0.8;
    foot->depth = 2;


    SoCube* upperArm = new SoCube;
    upperArm->width = 0.8;
    upperArm->height = 3;
    upperArm->depth = 0.8;

    SoTransform* foreArmTransform = new SoTransform;
    foreArmTransform->translation.setValue(0, -3, 0);

    SoCube* forerArm = new SoCube;
    forerArm->width = 0.6;
    forerArm->height = 2.5;
    forerArm->depth = 0.6;

    // Put arm parts together
    SoGroup* arm = new SoGroup;
    arm->addChild(upperArm);
    arm->addChild(foreArmTransform);
    arm->addChild(forerArm);

    SoMaterial* whiteOutfit = new SoMaterial;
    whiteOutfit->ambientColor.setValue(.9, .9, .9);
    whiteOutfit->diffuseColor.setValue(9, .9, .9);
    whiteOutfit->specularColor.setValue(.99, .99, .99);
    whiteOutfit->shininess = .75;

    // Put leg parts together
    SoGroup* leg = new SoGroup;
    leg->addChild(thigh);
    leg->addChild(calfTransform);
    leg->addChild(calf);
    leg->addChild(footTransform);
    leg->addChild(whiteOutfit);
    leg->addChild(foot);

    SoTransform* leftLegTransform = new SoTransform;
    leftLegTransform->translation = SbVec3f(1, -4.25, 0);
    leftLegTransform->rotation.setValue(SbVec3f(1, 0, 0), -PI / 2);

    // Left leg
    SoSeparator* leftLeg = new SoSeparator;
    leftLeg->addChild(leftLegTransform);
    leftLeg->addChild(leg);

    SoTransform* rightLegTransform = new SoTransform;
    rightLegTransform->translation.setValue(-1, -4.25, 0);
    rightLegTransform->rotation.setValue(SbVec3f(1, 0, 0), PI / 2);

    // Right leg
    SoSeparator* rightLeg = new SoSeparator;
    rightLeg->addChild(rightLegTransform);
    rightLeg->addChild(leg);

    SoTransform* leftArmTransform = new SoTransform;
    leftArmTransform->translation = SbVec3f(3, 2, 0);
    leftArmTransform->rotation.setValue(SbVec3f(0, 0, 1), PI / 2);


    //Left arm
    SoSeparator* leftArm = new SoSeparator;
    leftArm->addChild(leftArmTransform);
    leftArm->addChild(arm);

    SoTransform* rightArmTransform = new SoTransform;
    rightArmTransform->translation = SbVec3f(-3, 2, 0);
    rightArmTransform->rotation.setValue(SbVec3f(0, 0, 1), -PI / 2);

    //Right arm
    SoSeparator* rightArm = new SoSeparator;
    rightArm->addChild(rightArmTransform);
    rightArm->addChild(arm);

    // Parts for body
    SoTransform* bodyTransform = new SoTransform;
    bodyTransform->translation.setValue(0.0, 3.0, 0.0);

    SoMaterial* pinkOutfit = new SoMaterial;
    pinkOutfit->ambientColor.setValue(.33, .22, .27);
    pinkOutfit->diffuseColor.setValue(.8, .1, .7);
    pinkOutfit->specularColor.setValue(.92, .5, .81);
    pinkOutfit->shininess = .1;

    SoMaterial* purpleOutfit = new SoMaterial;
    purpleOutfit->ambientColor.setValue(.33, 0, 33);
    purpleOutfit->diffuseColor.setValue(.4, .1, .4);
    purpleOutfit->specularColor.setValue(.70, .5, .70);
    purpleOutfit->shininess = .4;

    SoCylinder* bodyCylinder = new SoCylinder;
    bodyCylinder->radius = 2;
    bodyCylinder->height = 6;

    // Construct body out of parts 
    SoSeparator* body = new SoSeparator;
    body->addChild(bodyTransform);
    body->addChild(pinkOutfit);
    body->addChild(bodyCylinder);
    body->addChild(leftArm);
    body->addChild(rightArm);
    body->addChild(purpleOutfit);
    body->addChild(leftLeg);
    body->addChild(rightLeg);

    // Head parts
    SoTransform* headTransform = new SoTransform;
    headTransform->translation.setValue(0, 7.5, 0);
    headTransform->scaleFactor.setValue(1.5, 1.5, 1.5);

    SoMaterial* gray = new SoMaterial;
    gray->ambientColor.setValue(.2, .2, .2);
    gray->diffuseColor.setValue(.6, .6, .6);
    gray->specularColor.setValue(.5, .5, .5);
    gray->shininess = .5;

    SoSphere* headSphere = new SoSphere;

    // Construct head
    SoSeparator* head = new SoSeparator;
    head->addChild(headTransform);
    head->addChild(gray);
    head->addChild(headSphere);

    // Robot is just head and body
    SoSeparator* robot = new SoSeparator;
    robot->addChild(body);
    robot->addChild(head);

    viewer->setSceneGraph(robot);
    viewer->setTitle("MECH534 - HW6 Part A");
    viewer->show();

    SoWin::show(window);
    SoWin::mainLoop();
 
    //============== PART B ========== //

    SoSeparator* rubiksCube = makeIndexedTriangularCube();

    SoWriteAction writeAction;
    writeAction.getOutput()->openFile("myCube.iv");
    writeAction.getOutput()->setBinary(FALSE);  // Human readable format
    writeAction.apply(rubiksCube);
    writeAction.getOutput()->closeFile();

    SoInput sceneInput;
    sceneInput.openFile("myCube.iv");
    SoSeparator* inputFile = new SoSeparator;
    inputFile = SoDB::readAll(&sceneInput);
    viewer->setSceneGraph(inputFile);
    viewer->setTitle("MECH534 - HW6 Part B");
    viewer->show();

    SoWin::show(window);
    SoWin::mainLoop();

    return 0;
}
