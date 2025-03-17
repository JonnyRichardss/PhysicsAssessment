#include "VehicleActor.h"
using namespace PhysicsEngine;
VehicleActor::VehicleActor(const PxTransform& pose) : DynamicActor(pose)
{
	//need wheelSimFilterData , chassisSimFilterData, chassisData, wheelMaterial, chassisMaterial, wheelGeometry, ChassisGeometry

	const int numWheels = 8;
	//SnippetVehicleTank :: initTankDesc
	const PxF32 chassisMass = 15000.0f;
	const PxVec3 chassisDims(3.5f, 2.0f, 9.0f);
	const PxVec3 chassisMOI
	((chassisDims.y * chassisDims.y + chassisDims.z * chassisDims.z) * chassisMass / 12.0f,
		(chassisDims.x * chassisDims.x + chassisDims.z * chassisDims.z) * 0.8f * chassisMass / 12.0f,
		(chassisDims.x * chassisDims.x + chassisDims.y * chassisDims.y) * chassisMass / 12.0f);
	const PxVec3 chassisCMOffset(0.0f, -chassisDims.y * 0.5f + 0.65f, 0.25f);

	const PxF32 wheelMass = 20.0f;
	const PxF32 wheelRadius = 0.75f;
	const PxF32 wheelWidth = 0.5f;
	const PxF32 wheelMOI = 0.5f * wheelMass * wheelRadius * wheelWidth;
	const PxU32 nbWheels = 8;

	PxFilterData chassisSimFilterData = PxFilterData(4, 32, 0, 0);
	PxFilterData wheelSimFilterData = PxFilterData(2, 14, 0, 0);

	TyreMaterial = CreateMaterial(0.5, 0.5, 0.5);
	ChassisMaterial = CreateMaterial(0.5, 0.5, 0.5);

	PxSphereGeometry WheelGeometry = PxSphereGeometry(0.5f);
	PxBoxGeometry ChassisGeometry = PxBoxGeometry(chassisDims / 2);



	//manually setting wheel and chassis as undrivable surfaces
	PxFilterData wheelQryFilterData;
	PxFilterData chassisQryFilterData;
	wheelQryFilterData.word3 = 0x0000ffff; 
	chassisQryFilterData.word3 = 0x0000ffff;

	//Add all the wheel shapes to the actor.
	for (PxU32 i = 0; i < numWheels; i++)
	{
		//TODO set geometry (look at wheelConvexMeshes) 
		//TODO setup wheelSimFilterData
		CreateShape(WheelGeometry,1, wheelQryFilterData,wheelSimFilterData,PxVec3(0), TyreMaterial);
	}

	//Add the chassis shapes to the actor.
	//TODO set geometry (look at chassisConvexMeshes)
	//TODO setup chassisSimFilterData
	CreateShape(ChassisGeometry, 4, chassisQryFilterData, chassisSimFilterData, PxVec3(0, 0, 1), ChassisMaterial);

	//TODO setup chassisData
	GetRigidBody()->setMass(chassisMass);
	GetRigidBody()->setMassSpaceInertiaTensor(chassisMOI);
	GetRigidBody()->setCMassLocalPose(PxTransform(chassisCMOffset, PxQuat(PxIdentity)));

	//Set up the sim data for the wheels.
	PxVehicleWheelsSimData* wheelsSimData = PxVehicleWheelsSimData::allocate(numWheels);
	{
		//Compute the wheel center offsets from the origin.
		PxVec3 wheelCentreActorOffsets[PX_MAX_NB_WHEELS];
		const PxF32 wheelFrontZ = chassisDims.z * 0.35f;
		const PxF32 wheelRearZ = -chassisDims.z * 0.35f;
		//tank::computeWheelCenterActorOffsets(frontZ, rearZ, chassisDims, wheelWidth, wheelRadius, numWheels, wheelCentreActorOffsets);
		
		//chassisDims.z is the distance from the rear of the chassis to the front of the chassis.
		//The front has z = 0.5*chassisDims.z and the rear has z = -0.5*chassisDims.z.
		//Compute a position for the front wheel and the rear wheel along the z-axis.
		//Compute the separation between each wheel along the z-axis.
		const PxF32 numLeftWheels = numWheels / 2.0f;
		const PxF32 deltaZ = (wheelFrontZ - wheelRearZ) / (numLeftWheels - 1.0f);
		//Set the outside of the left and right wheels to be flush with the chassis.
		//Set the top of the wheel to be just touching the underside of the chassis.
		for (PxU32 i = 0; i < numWheels; i += 2)
		{
			//Left wheel offset from origin.
			wheelCentreActorOffsets[i + 0] = PxVec3((-chassisDims.x + wheelWidth) * 0.5f, -(chassisDims.y / 2 + wheelRadius), wheelRearZ + i * deltaZ * 0.5f);
			//Right wheel offsets from origin.
			wheelCentreActorOffsets[i + 1] = PxVec3((+chassisDims.x - wheelWidth) * 0.5f, -(chassisDims.y / 2 + wheelRadius), wheelRearZ + i * deltaZ * 0.5f);
		}

		setupWheelsSimulationData
		(wheelMass, wheelMOI, wheelRadius, wheelWidth,
			numWheels, wheelCentreActorOffsets,
			chassisCMOffset, chassisMass,
			wheelsSimData);
	}

	//Set up the sim data for the tank drive model.
	PxVehicleDriveSimData driveSimData;
	{
		//Set up the engine to be more powerful but also more damped than the default engine.
		PxVehicleEngineData engineData = driveSimData.getEngineData();
		engineData.mPeakTorque *= 2.0f;
		engineData.mDampingRateZeroThrottleClutchEngaged = 2.0f;
		engineData.mDampingRateZeroThrottleClutchDisengaged = 0.5f;
		engineData.mDampingRateFullThrottle = 0.5f;
		driveSimData.setEngineData(engineData);
	}
	
	//Create a tank from the wheels and drive sim data.
	tank = PxVehicleDriveTank::allocate(numWheels);
	tank->setup(GetPhysics(), GetRigidBody(), *wheelsSimData, driveSimData, numWheels);

	//Configure the userdata
	//configureUserData(vehDriveTank, tankDesc.actorUserData, tankDesc.shapeUserDatas);

	//Free the sim data because we don't need that any more.
	wheelsSimData->free();

	return;



	/*
	PxVehicleWheelsSimData* wheelsSimData = PxVehicleWheelsSimData::allocate(4);
	PxVehicleDriveSimData4W driveSimData;
	//setupWheels
	//setupDrive
	//setupActor

	drive = PxVehicleDrive4W::allocate(4);
	drive->setup(GetPhysics(), GetRigidBody(), *wheelsSimData, driveSimData, 0);
	wheelsSimData->free();
	*/
}

PhysicsEngine::VehicleActor::~VehicleActor()
{
	if (tank)
		tank->free();
}

/*
void setupWheelsSimulationData(const PxF32 wheelMass, const PxF32 wheelMOI,
const PxF32 wheelRadius, const PxF32 wheelWidth, const PxU32 numWheels,
const PxVec3* wheelCenterActorOffsets, const PxVec3& chassisCMOffset,
const PxF32 chassisMass, PxVehicleWheelsSimData* wheelsSimData);
void setupDriveSimulationData(const PxVehicleDifferential4WData::Enum diffType,const PxReal enginePeakTorque,
const PxReal engineMaxOmega,const PxReal gearSwitchTime,const PxReal clutchStrength,PxVehicleDriveSimData4W& driveSimData);
void setupVehicleActor(PxRigidDynamic* vehicleActor);
*/
void VehicleActor::CreateShape(const PxGeometry& geometry, PxReal density, const PxFilterData& queryFilterData, const PxFilterData& simulationFilterData, const PxVec3& color,  const PxMaterial* material)
{
	PxShape* shape = ((PxRigidDynamic*)actor)->createShape(geometry, *material);
	PxRigidBodyExt::updateMassAndInertia(*(PxRigidDynamic*)actor, density);
	colors.push_back(color);
	//pass the color pointers to the renderer
	shape->userData = new UserData();
	shape->setQueryFilterData(queryFilterData);
	shape->setSimulationFilterData(simulationFilterData);
	shape->setLocalPose(PxTransform(PxIdentity));
	for (unsigned int i = 0; i < colors.size(); i++)
		((UserData*)GetShape(i)->userData)->color = &colors[i];
}

inline void PhysicsEngine::VehicleActor::setupWheelsSimulationData(const PxF32 wheelMass, const PxF32 wheelMOI, const PxF32 wheelRadius, const PxF32 wheelWidth, const PxU32 numWheels, const PxVec3* wheelCenterActorOffsets, const PxVec3& chassisCMOffset, const PxF32 chassisMass, PxVehicleWheelsSimData* wheelsSimData)
{

	//Set up the wheels.
	PxVehicleWheelData wheels[PX_MAX_NB_WHEELS];
	{
		//Set up the wheel data structures with mass, moi, radius, width.
		//Increase the damping on the wheel.
		for (PxU32 i = 0; i < numWheels; i++)
		{
			wheels[i].mMass = wheelMass;
			wheels[i].mMOI = wheelMOI;
			wheels[i].mRadius = wheelRadius;
			wheels[i].mWidth = wheelWidth;
			wheels[i].mDampingRate = 2.0f;
		}
	}

	//Set up the tires.
	PxVehicleTireData tires[PX_MAX_NB_WHEELS];
	{
		//Set all tire types to "normal" type.
		for (PxU32 i = 0; i < numWheels; i++)
		{
			tires[i].mType = 0;
		}
	}

	//Set up the suspensions
	PxVehicleSuspensionData suspensions[PX_MAX_NB_WHEELS];
	{
		//Compute the mass supported by each suspension spring.
		PxF32 suspSprungMasses[PX_MAX_NB_WHEELS];
		PxVehicleComputeSprungMasses(numWheels, wheelCenterActorOffsets, chassisCMOffset, chassisMass, 1, suspSprungMasses);

		//Set the suspension data.
		for (PxU32 i = 0; i < numWheels; i++)
		{
			suspensions[i].mMaxCompression = 0.3f;
			suspensions[i].mMaxDroop = 0.1f;
			suspensions[i].mSpringStrength = 10000.0f;
			suspensions[i].mSpringDamperRate = 1500.0f;
			suspensions[i].mSprungMass = suspSprungMasses[i];
		}
	}

	//Set up the wheel geometry.
	PxVec3 suspTravelDirections[PX_MAX_NB_WHEELS];
	PxVec3 wheelCentreCMOffsets[PX_MAX_NB_WHEELS];
	PxVec3 suspForceAppCMOffsets[PX_MAX_NB_WHEELS];
	PxVec3 tireForceAppCMOffsets[PX_MAX_NB_WHEELS];
	{
		for (PxU32 i = 0; i < numWheels; i++)
		{
			//Vertical suspension travel.
			suspTravelDirections[i] = PxVec3(0, -1, 0);

			//Wheel center offset is offset from rigid body center of mass.
			wheelCentreCMOffsets[i] = wheelCenterActorOffsets[i] - chassisCMOffset;

			//Suspension force application point 0.3 metres below rigid body center of mass.
			suspForceAppCMOffsets[i] = PxVec3(wheelCentreCMOffsets[i].x, -0.3f, wheelCentreCMOffsets[i].z);

			//Tire force application point 0.3 metres below rigid body center of mass.
			tireForceAppCMOffsets[i] = PxVec3(wheelCentreCMOffsets[i].x, -0.3f, wheelCentreCMOffsets[i].z);
		}
	}

	//Set up the filter data of the raycast that will be issued by each suspension.
	PxFilterData qryFilterData;
	qryFilterData.word3 = 0x0000ffff;

	//Set the wheel, tire and suspension data.
	//Set the geometry data.
	//Set the query filter data
	for (PxU32 i = 0; i < numWheels; i++)
	{
		wheelsSimData->setWheelData(i, wheels[i]);
		wheelsSimData->setTireData(i, tires[i]);
		wheelsSimData->setSuspensionData(i, suspensions[i]);
		wheelsSimData->setSuspTravelDirection(i, suspTravelDirections[i]);
		wheelsSimData->setWheelCentreOffset(i, wheelCentreCMOffsets[i]);
		wheelsSimData->setSuspForceAppPointOffset(i, suspForceAppCMOffsets[i]);
		wheelsSimData->setTireForceAppPointOffset(i, tireForceAppCMOffsets[i]);
		wheelsSimData->setSceneQueryFilterData(i, qryFilterData);
		wheelsSimData->setWheelShapeMapping(i, PxI32(i));
	}
}
