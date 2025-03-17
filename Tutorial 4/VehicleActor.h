#pragma once
#include "BasicActors.h"
namespace PhysicsEngine {
	class VehicleActor : public DynamicActor
	{
	public:
		VehicleActor(const PxTransform& pose);
		virtual ~VehicleActor();
		PxVehicleDriveTank* tank;
		PxMaterial* TyreMaterial;
		PxMaterial* ChassisMaterial;
	protected:
		/*
		void setupWheelsSimulationData(const PxF32 wheelMass, const PxF32 wheelMOI,
			const PxF32 wheelRadius, const PxF32 wheelWidth, const PxU32 numWheels,
			const PxVec3* wheelCenterActorOffsets, const PxVec3& chassisCMOffset,
			const PxF32 chassisMass, PxVehicleWheelsSimData* wheelsSimData);
		void setupDriveSimulationData(const PxVehicleDifferential4WData::Enum diffType,const PxReal enginePeakTorque,
			const PxReal engineMaxOmega,const PxReal gearSwitchTime,const PxReal clutchStrength,PxVehicleDriveSimData4W& driveSimData);
		void setupVehicleActor(PxRigidDynamic* vehicleActor);
		*/
		void CreateShape(const PxGeometry& geometry, PxReal density, const PxFilterData& queryFilterData, const PxFilterData& simulationFilterData, const PxVec3& color, const PxMaterial* material);
		void setupWheelsSimulationData
		(const PxF32 wheelMass, const PxF32 wheelMOI, const PxF32 wheelRadius, const PxF32 wheelWidth,
			const PxU32 numWheels, const PxVec3* wheelCenterActorOffsets,
			const PxVec3& chassisCMOffset, const PxF32 chassisMass,
			PxVehicleWheelsSimData* wheelsSimData);
		
	};
}

