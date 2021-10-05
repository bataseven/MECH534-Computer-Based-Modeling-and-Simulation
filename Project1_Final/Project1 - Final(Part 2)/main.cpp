/***************************************************************
 * MECH 534 - Computer-Based Modeling and Simulation
 * Project 1 - 3D SIMULATION OF A FLEXIBLE OBJECT
 * Berke Ataseven
 * 005432
 ***************************************************************/

#include <Inventor/Win/SoWin.h>
#include <Inventor/Win/viewers/SoWinExaminerViewer.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoCylinder.h>
#include <Inventor/nodes/SoNurbsCurve.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoComplexity.h>
#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <iostream>
#include <cmath>
// This is for debugging purposes
// #define PRINT_PARTICLE_POS
 
// ONLY PLAY WITH THE VARIABLES IN THIS SECTION. NOTE THAT FOR SOME VALUES OF THESE VARIABLE SYSTEM BECOMES UNSTABLE CAUSING THE SIMULATION TO CRASH
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Comment the next line for invisible particles 
#define VISIBLE_PARTICLES 
const int PARTICLE_COUNT = 13; // Min allowable value = 3
const float ROPE_LENGTH = 30;

float k = 20; // Stiffness: Use 46 instead of 4.6 for a stiffer and more realistic rope
float b = 22 / 10; // Damping
float m = 1.5; // Mass
float Cd = 0.25; // Drag coefficient: 0.9 feels too viscous. 0.25 is more fluent
float g = 9.81; // Gravitational acceleration

float dt = 0.1; // Euler time steps. Needs to be as small as possible for an accurate simulation. Trade-off between performance and accuracy
float updateInterval = 0.01; // How frequently the frame is updated (Calculations are made).
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef VISIBLE_PARTICLES
float radius = .01; // Make the particles too small to see
#else
float radius = 3; // Make large enough particles to see
#endif

boolean isWindXOn = false;
boolean isWindZOn = false;
float windStrength = 10;
int selectedParticle = 0;

SbVec3f particlePositions[PARTICLE_COUNT];
SbVec3f particleVelocities[PARTICLE_COUNT];
SbVec3f particleAccelerations[PARTICLE_COUNT];
SbVec3f particleForces[PARTICLE_COUNT];

SbVec3f anchorPoint = { 0, 50, 0 };

SoTimerSensor* timer = new SoTimerSensor();

SbVec3f particleAndAnchorPositions[PARTICLE_COUNT+1]; // Control points of the nurbs curve are updated from this array
float knots[PARTICLE_COUNT + 5]; // Knot vector for the nurbs curve
SoCoordinate3* controlPts;

SoTransform* particleTransforms[PARTICLE_COUNT];
SoMaterial* particleMaterials[PARTICLE_COUNT];

SoCylinder* strengthIndicator;
SoTransform* indicatorTF;
SoMaterial* indicatorMaterial;

SoSeparator* make_curve() {
	SoSeparator* curveSep = new SoSeparator();
	curveSep->ref();

	// Set the draw style of the curve.
	SoDrawStyle* drawStyle = new SoDrawStyle;
	drawStyle->lineWidth = 4;
	curveSep->addChild(drawStyle);

	// Define the NURBS curve including the control points
	// and a complexity.
	SoComplexity* complexity = new SoComplexity;	
	SoNurbsCurve* curve = new SoNurbsCurve;
#ifdef VISIBLE_PARTICLES
	complexity->value = 0.7; // value in range [0,1], higher values have better visuals at the cost of performance
#else
	complexity->value = 1; // value in range [0,1], higher values have better visuals at the cost of performance
#endif
	controlPts = new SoCoordinate3;
	controlPts->point.setValues(0, PARTICLE_COUNT+1, particleAndAnchorPositions);
	curve->numControlPoints = PARTICLE_COUNT+1;
	curve->knotVector.setValues(0, PARTICLE_COUNT + 5, knots);
	curveSep->addChild(complexity);
	curveSep->addChild(controlPts);
	curveSep->addChild(curve);

	curveSep->unrefNoDelete();
	return curveSep;
}

void change_material_color(SoMaterial*& material, SbVec3f color) {
	material->diffuseColor.setValue(color);
}

void key_press_callback(void* userData, SoEventCallback* eventCB){
	SoSelection* selection = (SoSelection*)userData;
	const SoEvent* event = eventCB->getEvent();

	if (SO_KEY_PRESS_EVENT(event, SoKeyboardEvent::Z)) { // Wind starts blowing in the Z-axis when the key 'Z' is pressed	
		if(!isWindZOn)
		std::cout << "Z-Axis wind is turned on." << std::endl;
		isWindZOn = true;
		eventCB->setHandled();
	}
	if (SO_KEY_RELEASE_EVENT(event, SoKeyboardEvent::Z)) { // Wind stops blowing in the Z-axis when the key 'Z' is released	
		isWindZOn = false;
		std::cout << "Z-Axis wind is turned off." << std::endl;
		eventCB->setHandled();
	}
	if (SO_KEY_PRESS_EVENT(event, SoKeyboardEvent::X)) { // Wind starts blowing in the X-axis when the key 'X' is pressed	
		if (!isWindXOn)
			std::cout << "X-Axis wind is turned on." << std::endl;
		isWindXOn = true;
		eventCB->setHandled();
	}
	if (SO_KEY_RELEASE_EVENT(event, SoKeyboardEvent::X)) { // Wind stops blowing in the X-axis when the key 'X' is released	
		isWindXOn = false;
		std::cout << "X-Axis wind is turned off." << std::endl;
		eventCB->setHandled();
	}
	if (SO_KEY_PRESS_EVENT(event, SoKeyboardEvent::UP_ARROW)) {
		change_material_color(particleMaterials[selectedParticle], { 0.8, 0.8, 0.8 });
		selectedParticle--;
		selectedParticle = selectedParticle == -1 ? selectedParticle + PARTICLE_COUNT : selectedParticle;
		change_material_color(particleMaterials[selectedParticle], {0.85, 0, 0});
	}
	if (SO_KEY_PRESS_EVENT(event, SoKeyboardEvent::DOWN_ARROW)) {
		change_material_color(particleMaterials[selectedParticle], {0.8, 0.8, 0.8});
		selectedParticle++;
		selectedParticle %= PARTICLE_COUNT;
		change_material_color(particleMaterials[selectedParticle], {0.85, 0, 0});
	}

	if (SO_KEY_PRESS_EVENT(event, SoKeyboardEvent::RIGHT_ARROW)) 
		windStrength = windStrength >= 100 ? 100 : windStrength + 1; // Increment the wind strength if not above a certain max value
	if (SO_KEY_PRESS_EVENT(event, SoKeyboardEvent::LEFT_ARROW)) 
		windStrength = windStrength <= 0 ? 0 : windStrength - 1; // Decrement the wind strength if not below a certain min value

	if (SO_KEY_PRESS_EVENT(event, SoKeyboardEvent::SPACE)) { // Pressing spacebar pauses/continues the simulation
		if (!timer->isScheduled()) {
			timer->schedule();
			std::cout << "Simulation continued" << std::endl;
		}
		else {
			std::cout << "Simulation stopped" << std::endl;
			timer->unschedule();
		}
		eventCB->setHandled();
	}
}

void calculate_forces() {
	static SbVec3f weight = { 0, m * g, 0 };
	static float piecewiseLength = ROPE_LENGTH / PARTICLE_COUNT;

	static SbVec3f upSideSpringForce;
	static SbVec3f downSideSpringForce;
	static SbVec3f upSideDamperForce;
	static SbVec3f downSideDamperForce;
	static SbVec3f dragForce;

	static float stretch;

	for (int i = 0; i < PARTICLE_COUNT; i++) {
		// String forces at the top of the mass
		if (i == 0) upSideSpringForce = particlePositions[i] - anchorPoint;
		else		upSideSpringForce = particlePositions[i] - particlePositions[i - 1];
		float upSideSpringForceLength = upSideSpringForce.normalize();	// Returns the length before normalization
		stretch = (piecewiseLength - upSideSpringForceLength);		// How for the spring is stretched from the resting length
		upSideSpringForce = upSideSpringForce * (k * stretch);		// F = k.x

		// String forces at the bottom of the mass
		downSideSpringForce = particlePositions[i + 1] - particlePositions[i];
		float downSideSpringForceLength = downSideSpringForce.normalize(); // Returns the length before normalization
		stretch = (piecewiseLength - downSideSpringForceLength); // How for the spring is stretched from the resting length
		downSideSpringForce = downSideSpringForce * (k * stretch); // F = k.x

		// Damper at the top of the mass
		if (i == 0) upSideDamperForce = particleVelocities[i];
		else		upSideDamperForce = particleVelocities[i] - particleVelocities[i - 1];
		upSideDamperForce = upSideDamperForce * b;

		// Damper at the bottom of the mass
		downSideDamperForce = particleVelocities[i + 1] - particleVelocities[i];
		downSideDamperForce = downSideDamperForce * b;

		// Drag force
		SbVec3f dummy = particleVelocities[i]; // Dummy vector to keep the original values of 'particleVelocities' array
		dummy.normalize();
		dragForce = (dummy * particleVelocities[i].sqrLength() * Cd); // F = Cd * V^2
		dragForce.negate(); // Revervse the direction of the drag.

		particleForces[i] = dragForce + upSideSpringForce - upSideDamperForce - weight; // upSide spring and damper forces exists on every mass 
		if (i != PARTICLE_COUNT - 1)	particleForces[i] += (-downSideSpringForce + downSideDamperForce); // Add the downSide forces aswell if it is not the bottom-most particle
	}
	// Wind forces
	if (isWindXOn || isWindZOn) { // Enter here only if either of the winds are active
			if (isWindXOn) {
				SbVec3f WindX = {windStrength, 0, 0};
				particleForces[selectedParticle] += WindX;
			}
			if (isWindZOn) {
				SbVec3f WindZ = {0, 0, windStrength};
				particleForces[selectedParticle] += WindZ;
			}
		}
}

void update_frame_callback(void* data, SoSensor* timer) {
	calculate_forces(); // Calculate the forces acting on each particle before updating the frame
	
	for (int i = 0; i < PARTICLE_COUNT; i++) {		
		particleAccelerations[i] = particleForces[i] / m; // F = m.a
		particleVelocities[i] += particleAccelerations[i] * dt; // Eulers method
		particlePositions[i] += particleVelocities[i] * dt; // Eulers method
		particleTransforms[i] -> translation.setValue(particlePositions[i]);
		particleAndAnchorPositions[i + 1] = particlePositions[i]; // Update the positions for the nurbs curve
#ifdef PRINT_PARTICLE_POS
		printf_s("%7.4f ", particlePositions[i].getValue()[1]);
#endif
	}
	controlPts->point.setValues(0, PARTICLE_COUNT + 1, particleAndAnchorPositions); // Update the control points of the nurbs curve
	indicatorTF->translation.setValue(-50, -30 + (windStrength*0.75) / 2, 0);
	strengthIndicator->height = windStrength * 0.75;
	change_material_color(indicatorMaterial, { windStrength / 100, 1 - windStrength / 100, 0 });
#ifdef PRINT_PARTICLE_POS
		cout << endl;
#endif
}

int main(int, char** argv)
{
	srand(1); // Initialize the random number generator with a seed

	particleAndAnchorPositions[0] = anchorPoint; // First element of the array is the anchor position(This array is used for drawing the nurbs curve).
	
	for (int i = 0; i < PARTICLE_COUNT; i++) {// Initialize the positions
		particlePositions[i].setValue(0, 49 - i, 0); // 
		particleAndAnchorPositions[i + 1] = particlePositions[i];
	}

	for (int i = 0; i < PARTICLE_COUNT + 5; i++) { // Fill the knot vector like: [0,0,0,0, ........... ,n,n,n,n] where n is PARTICLE_COUNT - 2. This makes sure the curve passes through control points
		if (i < 4)						knots[i] = 0;
		else if (i > PARTICLE_COUNT)	knots[i] = PARTICLE_COUNT - 2;
		else							knots[i] = i - 3;
	}
		
#ifdef PRINT_PARTICLE_POS
	for (int i = 0; i < PARTICLE_COUNT; i++) {
		float a = 0, b = 0, c = 0;
		particlePositions[i].getValue(a, b, c);
		printf_s("%.2f ", b);
	}
	cout << endl;
#endif

	HWND window = SoWin::init(argv[0]); // Initialize the library
	if (window == NULL)
		exit(1);
	SoWinExaminerViewer* viewer = new SoWinExaminerViewer(window);

	SoSeparator* rope = new SoSeparator(); // Parent-most object in the hierarchy

	SoSeparator* particleSeperator[PARTICLE_COUNT]; // Create an individual seperator for each sphere so that the transformations do not add up
	SoSphere* particleSpheres[PARTICLE_COUNT]; // Array that holds the particle visuals
	
	SoSeparator* nurbsSeperator = make_curve(); // Create the nurbs curve

	// Set up the anchor, its material and transformation
	SoSeparator* anchorSeperator = new SoSeparator;
	SoMaterial* anchorMaterial = new SoMaterial;
	anchorMaterial->ambientColor.setValue(.33, .22, .27);
	anchorMaterial->diffuseColor.setValue(.7, .1, .6);
	anchorMaterial->specularColor.setValue(.82, .5, .71);
	anchorMaterial->shininess = .5;
	SoCube* anchorBox = new SoCube;
	anchorBox->height = 2;
	anchorBox->width = 80;
	anchorBox->depth = 25;
	SoTransform* anchorTF = new SoTransform;
	anchorTF->translation.setValue(0, anchorPoint.getValue()[1], 0);
	anchorSeperator->addChild(anchorTF);
	anchorSeperator->addChild(anchorMaterial);
	anchorSeperator->addChild(anchorBox);

	indicatorTF = new SoTransform;
	indicatorTF->translation.setValue(-50, -30 + (windStrength * 0.75) / 2, 0);
	SoSeparator* indicatorSeperator = new SoSeparator;
	indicatorMaterial = new SoMaterial;
	strengthIndicator = new SoCylinder;
	strengthIndicator->radius = 5;
	strengthIndicator->height = windStrength;
	indicatorSeperator->addChild(indicatorTF);
	indicatorSeperator->addChild(indicatorMaterial);
	indicatorSeperator->addChild(strengthIndicator);

	rope->addChild(indicatorSeperator);
	rope->addChild(nurbsSeperator); // Add the rope
	rope->addChild(anchorSeperator); // Add the anchor


	for (int i = 0; i < PARTICLE_COUNT; i++) {
		particleSpheres[i] = new SoSphere;    // Create the particle visual
		particleSpheres[i] -> radius = radius;// Set particle radius

		particleSeperator[i] = new SoSeparator; // Create the seperator which will have the sphere and the transform as its children
		particleTransforms[i] = new SoTransform; // Create the particle transformation
		particleMaterials[i] = new SoMaterial; // Create the material for each particle

		particleSeperator[i]->addChild(particleTransforms[i]); // Add the transform of a particle
		particleSeperator[i]->addChild(particleMaterials[i]); // Make the particles same material as the anchor
		particleSeperator[i]->addChild(particleSpheres[i]); // Add the sphere visual
		rope->addChild(particleSeperator[i]); // Add the whole particle seperator to the rope
	}
	change_material_color(particleMaterials[selectedParticle], {0.85, 0, 0}); // Paint the selected particle to red
	
	SoEventCallback* myEventCB = new SoEventCallback;	// Keyboard event listener
	myEventCB->addEventCallback(SoKeyboardEvent::getClassTypeId(), key_press_callback);
	rope->addChild(myEventCB); // Add the keyboard listener as a child

	timer->setFunction(update_frame_callback);
	timer->setInterval(updateInterval);
	if (!timer->isScheduled()) timer->schedule();
	
	viewer->setSceneGraph(rope);
	viewer->setTitle("MECH 534 - Project 1-Final");
	viewer->show();
	SoWin::show(window);
	SoWin::mainLoop();
	return 0;
}