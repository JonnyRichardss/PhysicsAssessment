#pragma once

#include "BasicActors.h"
#include "Brick.h"
#include "BowlingBall.h"
#include "Extras/Renderer.h"
#include "MyPhysicsEngine.h"
#include "VehicleActor.h"
#include "SnippetVehicleSceneQuery.h"
#include <iostream>
#include <iomanip>
#define NUM_BRICKS_WIDTH  20
#define NUM_BRICKS_HEIGHT  5
#define NUM_BRICKS_DEPTH  1
///Custom scene class
namespace PhysicsEngine{
class MyScene : public Scene
{
	Plane* plane;
	Brick* bricks[NUM_BRICKS_HEIGHT][NUM_BRICKS_WIDTH][NUM_BRICKS_DEPTH];
	BowlingBall* ball;
	BowlingBall* smallBall;
	MySimulationEventCallback* my_callback;
	VehicleActor* vehicleActor;
	PxVehicleDriveTank* vehicle;
	PxVehicleDriveTankRawInputData vehicleInput = PxVehicleDriveTankRawInputData(PxVehicleDriveTankControlModel::Enum::eSPECIAL);
	PxVehicleDrivableSurfaceToTireFrictionPairs* FrictionPairs;
	snippetvehicle::VehicleSceneQueryData* vehicleSceneQueryData;
	PxBatchQuery* batchQuery;
	float InputL;
	float InputR;
	float InputThrottle;
	bool InputBrake;
public:
	//specify your custom filter shader here
	//PxDefaultSimulationFilterShader by default
	MyScene();

	void AddWall();

	void SetVisualisation();

	//Custom scene initialisation
	virtual void CustomInit();

	//Custom udpate function
	virtual void CustomUpdate(PxReal deltaTime) override;
	virtual void CustomRender();
	/// An example use of key release handling
	void ExampleKeyReleaseHandler();

	/// An example use of key presse handling
	void ExampleKeyPressHandler();

	void PressBrake();
	void ReleaseBrake();

	void IncrementThrottle();
	void DecrementThrottle();
	void IncrementL();
	void DecrementL();
	void IncrementR();
	void DecrementR();

	PxVehicleDrivableSurfaceToTireFrictionPairs* createFrictionPairs(const PxMaterial* defaultMaterial);
};
}

