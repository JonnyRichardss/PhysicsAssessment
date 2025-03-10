#pragma once
#include "BasicActors.h"
namespace PhysicsEngine{
	static const PxReal BowlingBallDimensions = 3.0;
	static const PxReal BowlingBallDensity = 785.0; //solid steel
	static const PxVec3 BowlingBallMaterialParams = { 0.2,0.1,1.0 };
	static const PxVec3 BowlingBallColour = { 0.05,0.6,0.05 };
	class BowlingBall : public Sphere
	{
	public:
		BowlingBall(const PxTransform& pose) : BowlingBall(pose, BowlingBallDimensions, BowlingBallDensity)
		{}
		BowlingBall(const PxTransform& pose, PxReal radius, PxReal density): Sphere(pose, radius, density) {
			PxMaterial* BallMat = PhysicsEngine::CreateMaterial(BowlingBallMaterialParams);
			Material(BallMat);
			Color(BowlingBallColour, -1);
		}
	};
}

