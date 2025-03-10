#pragma once

#include "BasicActors.h"
#include "Brick.h"
#include "BowlingBall.h"
#include "Extras/Renderer.h"
#include <iostream>
#include <iomanip>

namespace PhysicsEngine
{
	using namespace std;

	//a list of colours: Circus Palette
	static const PxVec3 color_palette[] = {PxVec3(46.f/255.f,9.f/255.f,39.f/255.f),PxVec3(217.f/255.f,0.f/255.f,0.f/255.f),
		PxVec3(255.f/255.f,45.f/255.f,0.f/255.f),PxVec3(255.f/255.f,140.f/255.f,54.f/255.f),PxVec3(4.f/255.f,117.f/255.f,111.f/255.f)};

	struct FilterGroup
	{
		enum Enum
		{
			ACTOR0		= (1 << 0),
			ACTOR1		= (1 << 1),
			ACTOR2		= (1 << 2)
			//add more if you need
		};
	};

	///A customised collision class, implemneting various callbacks
	class MySimulationEventCallback : public PxSimulationEventCallback
	{
	public:
		//an example variable that will be checked in the main simulation loop
		bool trigger;

		MySimulationEventCallback() : trigger(false) {}

		///Method called when the contact with the trigger object is detected.
		virtual void onTrigger(PxTriggerPair* pairs, PxU32 count) 
		{
			//you can read the trigger information here
			for (PxU32 i = 0; i < count; i++)
			{
				//filter out contact with the planes
				if (pairs[i].otherShape->getGeometryType() != PxGeometryType::ePLANE)
				{
					//check if eNOTIFY_TOUCH_FOUND trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_FOUND)
					{
						cerr << "onTrigger::eNOTIFY_TOUCH_FOUND" << endl;
						trigger = true;
					}
					//check if eNOTIFY_TOUCH_LOST trigger
					if (pairs[i].status & PxPairFlag::eNOTIFY_TOUCH_LOST)
					{
						cerr << "onTrigger::eNOTIFY_TOUCH_LOST" << endl;
						trigger = false;
					}
				}
			}
		}

		///Method called when the contact by the filter shader is detected.
		virtual void onContact(const PxContactPairHeader &pairHeader, const PxContactPair *pairs, PxU32 nbPairs) 
		{
			cerr << "Contact found between " << pairHeader.actors[0]->getName() << " " << pairHeader.actors[1]->getName() << endl;

			//check all pairs
			for (PxU32 i = 0; i < nbPairs; i++)
			{
				//check eNOTIFY_TOUCH_FOUND
				if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_FOUND)
				{
					cerr << "onContact::eNOTIFY_TOUCH_FOUND" << endl;
				}
				//check eNOTIFY_TOUCH_LOST
				if (pairs[i].events & PxPairFlag::eNOTIFY_TOUCH_LOST)
				{
					cerr << "onContact::eNOTIFY_TOUCH_LOST" << endl;
				}
			}
		}

		virtual void onConstraintBreak(PxConstraintInfo *constraints, PxU32 count) {}
		virtual void onWake(PxActor **actors, PxU32 count) {}
		virtual void onSleep(PxActor **actors, PxU32 count) {}
#if PX_PHYSICS_VERSION >= 0x304000
		virtual void onAdvance(const PxRigidBody *const *bodyBuffer, const PxTransform *poseBuffer, const PxU32 count) {}
#endif
	};

	//A simple filter shader based on PxDefaultSimulationFilterShader - without group filtering
	static PxFilterFlags CustomFilterShader( PxFilterObjectAttributes attributes0,	PxFilterData filterData0,
		PxFilterObjectAttributes attributes1,	PxFilterData filterData1,
		PxPairFlags& pairFlags,	const void* constantBlock,	PxU32 constantBlockSize)
	{
		// let triggers through
		if(PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
		{
			pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
			return PxFilterFlags();
		}

		pairFlags = PxPairFlag::eCONTACT_DEFAULT;
		//enable continous collision detection
//		pairFlags |= PxPairFlag::eCCD_LINEAR;
		
		
		//customise collision filtering here
		//e.g.

		// trigger the contact callback for pairs (A,B) where 
		// the filtermask of A contains the ID of B and vice versa.
		if((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
		{
			//trigger onContact callback for this pair of objects
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_LOST;
//			pairFlags |= PxPairFlag::eNOTIFY_CONTACT_POINTS;
		}

		return PxFilterFlags();
	};
	#define NUM_BRICKS_WIDTH  20
	#define NUM_BRICKS_HEIGHT  5
	#define NUM_BRICKS_DEPTH  1
	///Custom scene class
	class MyScene : public Scene
	{
		Plane* plane;
		Brick* bricks[NUM_BRICKS_HEIGHT][NUM_BRICKS_WIDTH][NUM_BRICKS_DEPTH];
		BowlingBall* ball;
		BowlingBall* smallBall;
		MySimulationEventCallback* my_callback;
		
	public:
		//specify your custom filter shader here
		//PxDefaultSimulationFilterShader by default
		MyScene() : Scene() {};

		///A custom scene class
		void SetVisualisation()
		{
			px_scene->setVisualizationParameter(PxVisualizationParameter::eSCALE, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCOLLISION_SHAPES, 1.0f);

			//cloth visualisation
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCLOTH_HORIZONTAL, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCLOTH_VERTICAL, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCLOTH_BENDING, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eCLOTH_SHEARING, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LOCAL_FRAMES, 1.0f);
			px_scene->setVisualizationParameter(PxVisualizationParameter::eJOINT_LIMITS, 1.0f);
		}

		//Custom scene initialisation
		virtual void CustomInit() 
		{
			SetVisualisation();			

			GetMaterial()->setDynamicFriction(.2f);

			///Initialise and set the customised event callback
			my_callback = new MySimulationEventCallback();
			px_scene->setSimulationEventCallback(my_callback);

			plane = new Plane();
			plane->Color(PxVec3(210.f/255.f,210.f/255.f,210.f/255.f));
			Add(plane);
			
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
							toJoin[0] = bricks[i+1][j][k];
						}
						if (j < (NUM_BRICKS_WIDTH - 1)) {
							toJoin[1] = bricks[i][j+1][k];
						}
						if (k < (NUM_BRICKS_DEPTH - 1)) {
							toJoin[2] = bricks[i][j][k+1];
						}
						//for each brick [i+j+k] add a joint to the one above, to the right and behind it
						
						for (int i = 0; i < 3;i++) {
							Brick* otherBrick = toJoin[i];
							if (otherBrick == nullptr) {
								continue;
							}
							PxTransform brickTransform = brick->GetRigidBody()->getGlobalPose();
							PxTransform otherTransform = otherBrick->GetRigidBody()->getGlobalPose();
							PxVec3 brickPos = brickTransform.p;
							PxVec3 otherPos = otherTransform.p;
							PxFixedJoint* joint = PxFixedJointCreate(*GetPhysics(), brick->GetRigidBody(),PxTransform((otherPos - brickPos)/2.0), otherBrick->GetRigidBody(), PxTransform((brickPos - otherPos) / 2.0));
							PxReal JOINT_BREAK_FORCE = 100000.0f;
							joint->setBreakForce(JOINT_BREAK_FORCE,JOINT_BREAK_FORCE);
							joint->setConstraintFlag(PxConstraintFlag::eVISUALIZATION,true);
							joint->setConstraintFlag(PxConstraintFlag::ePROJECTION, true);
							PxConstraintFlags test = joint->getConstraintFlags();

							//joint->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, true);
							
							numJoints++;
						}
						
						
						 
					}
				}
			}
			printf("Created %i joints!\n", numJoints);
			PxTransform ballPos(PxVec3(0, 0, 75.0));
			PxTransform smallBallPos(PxVec3(0, 0, -25.0));
			ball = new BowlingBall(ballPos);
			smallBall = new BowlingBall(smallBallPos, BowlingBallDimensions / 3.0f, BowlingBallDensity);
			Add(ball);
			Add(smallBall);
			PxReal BallForce = ball->GetRigidBody()->getMass() * 25;
			PxReal SmallBallForce = smallBall->GetRigidBody()->getMass() * 15;
			ball->GetRigidBody()->addForce(PxVec3(0, 0, -1 * BallForce), PxForceMode::eIMPULSE);
			smallBall->GetRigidBody()->addForce(PxVec3(0, 0, SmallBallForce), PxForceMode::eIMPULSE);
			

			//setting custom cloth parameters
			//((PxCloth*)cloth->Get())->setStretchConfig(PxClothFabricPhaseType::eBENDING, PxClothStretchConfig(1.f));
		}

		//Custom udpate function
		virtual void CustomUpdate() 
		{
			
		}
		virtual void CustomRender() {
			PxVec3 BallLocation = ball->GetRigidBody()->getGlobalPose().p;
			char buf[100];
			snprintf(buf, (size_t)100, "Ball Pos: x:%f y:%f z:%f\0", BallLocation.x, BallLocation.y, BallLocation.z);
			string s = buf;
			PxVec2 location = PxVec2(0, 0);
			PxVec3 color(1, 0, 0);
			VisualDebugger::Renderer::RenderText(s, location, color, 0.015f);
		}
		/// An example use of key release handling
		void ExampleKeyReleaseHandler()
		{
			cerr << "I am realeased!" << endl;
		}

		/// An example use of key presse handling
		void ExampleKeyPressHandler()
		{
			cerr << "I am pressed!" << endl;
		}
	};
}
