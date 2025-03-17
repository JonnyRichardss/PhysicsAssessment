#include "MyScene.h"

using namespace PhysicsEngine;

MyScene::MyScene() : Scene() {}

void PhysicsEngine::MyScene::AddWall()
{
	const PxVec3 StartPos = { NUM_BRICKS_WIDTH * BrickDimensions.x * -1 ,BrickDimensions.y,0 };
	const PxReal horizOffset = BrickDimensions.x;
	for (int i = 0; i < NUM_BRICKS_HEIGHT; i++) {
		for (int j = 0; j < NUM_BRICKS_WIDTH; j++) {
			for (int k = 0; k < NUM_BRICKS_DEPTH; k++) {
				PxVec3 offset(j * 2 * BrickDimensions.x, i * 2 * BrickDimensions.y, k * 2 * BrickDimensions.z);
				PxTransform newBrickTransform(StartPos + offset);
				//TODO these get leaked -- scene needs to handle deleting them
				Brick* b = new Brick(newBrickTransform);
				Add(b);
				bricks[i][j][k] = b;
			}
		}
	}
	int numJoints = 0;
	for (int i = 0; i < NUM_BRICKS_HEIGHT; i++) {
		for (int j = 0; j < NUM_BRICKS_WIDTH; j++) {
			for (int k = 0; k < NUM_BRICKS_DEPTH; k++) {
				Brick* brick = bricks[i][j][k];
				Brick* toJoin[3] = { nullptr,nullptr,nullptr };

				if (i < (NUM_BRICKS_HEIGHT - 1)) {
					toJoin[0] = bricks[i + 1][j][k];
				}
				if (j < (NUM_BRICKS_WIDTH - 1)) {
					toJoin[1] = bricks[i][j + 1][k];
				}
				if (k < (NUM_BRICKS_DEPTH - 1)) {
					toJoin[2] = bricks[i][j][k + 1];
				}
				for (int i = 0; i < 3; i++) {
					Brick* otherBrick = toJoin[i];
					if (otherBrick == nullptr) {
						continue;
					}
					PxTransform brickTransform = brick->GetRigidBody()->getGlobalPose();
					PxTransform otherTransform = otherBrick->GetRigidBody()->getGlobalPose();
					PxVec3 brickPos = brickTransform.p;
					PxVec3 otherPos = otherTransform.p;
					PxFixedJoint* joint = PxFixedJointCreate(*GetPhysics(), brick->GetRigidBody(), PxTransform((otherPos - brickPos) / 2.0), otherBrick->GetRigidBody(), PxTransform((brickPos - otherPos) / 2.0));
					PxReal JOINT_BREAK_FORCE = 100000.0f;
					joint->setBreakForce(JOINT_BREAK_FORCE, JOINT_BREAK_FORCE);
					joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION, true);
					PxConstraintFlags test = joint->getConstraintFlags();
					numJoints++;
				}
			}
		}
	}
	printf("Created %i joints!\n", numJoints);
}

void MyScene::SetVisualisation()
{
	px_scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
	px_scene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);

	px_scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES, 1.0f);
	px_scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS, 1.0f);
}

//Custom scene initialisation
void MyScene::CustomInit()
{
	SetVisualisation();

	GetMaterial()->setDynamicFriction(.2f);

	///Initialise and set the customised event callback
	my_callback = new MySimulationEventCallback();
	px_scene->setSimulationEventCallback(my_callback);


	PxVehicleSetBasisVectors(PxVec3(0, 1, 0), PxVec3(0, 0, 1));//tell vehicle SDK about our coordinate space
	PxVehicleSetUpdateMode(PxVehicleUpdateMode::Enum::eVELOCITY_CHANGE);//set vehicle SDK update mode

	vehicleSceneQueryData = snippetvehicle::VehicleSceneQueryData::allocate(1, PX_MAX_NB_WHEELS, 1, 1, snippetvehicle::WheelSceneQueryPreFilterBlocking, NULL, PxDefaultAllocator());
	batchQuery = snippetvehicle::VehicleSceneQueryData::setUpBatchedSceneQuery(0, *vehicleSceneQueryData, px_scene);
	{
		PxMaterial* planeMat = CreateMaterial(0.5, 0.5, 0.5);
		FrictionPairs = createFrictionPairs(planeMat);
		PxFilterData groundPlaneSimFilterData(1, 28, 0, 0);
		plane = new Plane();
		plane->Color(PxVec3(210.f / 255.f, 210.f / 255.f, 210.f / 255.f));
		plane->Material(planeMat);

		//Get the plane shape so we can set query and simulation filter data.
		PxShape* shapes[1];
		plane->GetRigidBody()->getShapes(shapes, 1);

		//Set the query filter data of the ground plane so that the vehicle raycasts can hit the ground.
		PxFilterData qryFilterData;
		qryFilterData.word3 = static_cast<PxU32>(-65536);
		shapes[0]->setQueryFilterData(qryFilterData);

		//Set the simulation filter data of the ground plane so that it collides with the chassis of a vehicle but not the wheels.
		shapes[0]->setSimulationFilterData(groundPlaneSimFilterData);
		Add(plane);
	}

	AddWall();

	ball = new BowlingBall(PxTransform(PxVec3(0, BowlingBallDimensions/2.0f, 75.0)));
	smallBall = new BowlingBall(PxTransform(PxVec3(0, BowlingBallDimensions / 6.0f, -25.0)), BowlingBallDimensions / 3.0f, BowlingBallDensity);
	Add(ball);
	Add(smallBall);
	PxReal BallForce = ball->GetRigidBody()->getMass() * 25;
	PxReal SmallBallForce = smallBall->GetRigidBody()->getMass() * 15;
	ball->GetRigidBody()->addForce(PxVec3(0, 0, -1 * BallForce), PxForceMode::eIMPULSE);
	smallBall->GetRigidBody()->addForce(PxVec3(0, 0, SmallBallForce), PxForceMode::eIMPULSE);




	vehicleActor = new VehicleActor(PxTransform(PxVec3(10, 2, 20)));
	vehicle = vehicleActor->tank;
	Add(vehicleActor);
	vehicle->setToRestState();
	vehicle->mDriveDynData.forceGearChange(PxVehicleGearsData::eFIRST);
	vehicle->mDriveDynData.setUseAutoGears(true);
	vehicle->setDriveModel(PxVehicleDriveTankControlModel::eSTANDARD);

}
//snippednestedScene.cpp
static PxVehiclePadSmoothingData gPadSmoothingData =
{
	{
		6.0f,	//rise rate eANALOG_INPUT_ACCEL=0,
		6.0f,	//rise rate eANALOG_INPUT_BRAKE,
		6.0f,	//rise rate eANALOG_INPUT_HANDBRAKE,
		2.5f,	//rise rate eANALOG_INPUT_STEER_LEFT,
		2.5f,	//rise rate eANALOG_INPUT_STEER_RIGHT,
	},
	{
		10.0f,	//fall rate eANALOG_INPUT_ACCEL=0
		10.0f,	//fall rate eANALOG_INPUT_BRAKE_LEFT
		10.0f,	//fall rate eANALOG_INPUT_BRAKE_RIGHT
		5.0f,	//fall rate eANALOG_INPUT_THRUST_LEFT
		5.0f	//fall rate eANALOG_INPUT_THRUST_RIGHT
	}
};
static int test = 0;
//Custom udpate function
void MyScene::CustomUpdate(PxReal deltaTime)
{
	vehicleInput.setAnalogAccel((abs(InputL) + abs(InputR))/2.0f);
	//vehicleInput.setAnalogAccel(.2);
	vehicleInput.setAnalogLeftThrust(InputL);
	vehicleInput.setAnalogRightThrust(InputR);
	//vehicleInput.setAnalogLeftBrake(-InputL);
	//vehicleInput.setAnalogRightBrake(-InputR);
	PxVehicleDriveTankSmoothAnalogRawInputsAndSetAnalogInputs(gPadSmoothingData, vehicleInput, deltaTime, *vehicle);
	PxVehicleWheels* vehicles[1] = { vehicle };

	const PxU32 raycastQueryResultsSize = vehicleSceneQueryData->getQueryResultBufferSize();
	PxRaycastQueryResult* raycastQueryResults = vehicleSceneQueryData->getRaycastQueryResultBuffer(0);
	PxVehicleSuspensionRaycasts(batchQuery, 1, vehicles, raycastQueryResultsSize, raycastQueryResults);

	const PxVec3 gravity = px_scene->getGravity();
	PxWheelQueryResult wheelQueryResults[PX_MAX_NB_WHEELS];
	PxVehicleWheelQueryResult vehicleQueryResults[1] = { {wheelQueryResults, vehicle->mWheelsSimData.getNbWheels()} };
	PxVehicleUpdates(deltaTime, gravity, *FrictionPairs, 1, vehicles, vehicleQueryResults);
	test = vehicleQueryResults[0].wheelQueryResults->isInAir;
}

static bool currentGear = true;
void MyScene::CustomRender() {
	PxVec3 BallLocation = ball->GetRigidBody()->getGlobalPose().p;
	char buf[100];
	snprintf(buf, (size_t)100, "Ball Pos: x:%f y:%f z:%f          L:%f          R:%f     Gear: %i\0", BallLocation.x, BallLocation.y, BallLocation.z,InputL,InputR,currentGear);
	string s = buf;
	PxVec2 location = PxVec2(0, 0);
	PxVec3 color(1, 0, 0);
	VisualDebugger::Renderer::RenderText(s, location, color, 0.015f);
}

/// An example use of key release handling
void MyScene::ExampleKeyReleaseHandler()
{
	cerr << "I am realeased!" << endl;
}

/// An example use of key presse handling
void MyScene::ExampleKeyPressHandler()
{
	currentGear = currentGear ? false : true;

	vehicle->mDriveDynData.forceGearChange(currentGear ? PxVehicleGearsData::eFIRST : PxVehicleGearsData::eREVERSE);
}

static float ChangeSize = 0.005;
void PhysicsEngine::MyScene::IncrementL()
{
	InputL += ChangeSize;
	if (InputL >= 1.0f) {
		InputL = 1.0f;
	}
}
void PhysicsEngine::MyScene::DecrementL()
{
	InputL -= ChangeSize;
	if (InputL <= -1.0f) {
		InputL = -1.0f;
	}
}
void PhysicsEngine::MyScene::IncrementR()
{
	InputR += ChangeSize;
	if (InputR >= 1.0f) {
		InputR = 1.0f;
	}
}
void PhysicsEngine::MyScene::DecrementR()
{
	InputR -= ChangeSize;
	if (InputR <= -1.0f) {
		InputR = -1.0f;
	}
}

//snippetvehicletyrefriction.cpp
PxVehicleDrivableSurfaceToTireFrictionPairs* PhysicsEngine::MyScene::createFrictionPairs(const PxMaterial* defaultMaterial)
{
	PxVehicleDrivableSurfaceType surfaceTypes[1];
	surfaceTypes[0].mType = 0;

	const PxMaterial* surfaceMaterials[1];
	surfaceMaterials[0] = defaultMaterial;

	PxVehicleDrivableSurfaceToTireFrictionPairs* surfaceTirePairs =
		PxVehicleDrivableSurfaceToTireFrictionPairs::allocate(1, 1);

	surfaceTirePairs->setup(1, 1, surfaceMaterials, surfaceTypes);

	for (PxU32 i = 0; i < 1; i++)
	{
		for (PxU32 j = 0; j < 1; j++)
		{
			surfaceTirePairs->setTypePairFriction(i, j, 1.0f);
		}
	}
	return surfaceTirePairs;
}
