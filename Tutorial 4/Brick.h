#pragma once
#include "BasicActors.h"
namespace PhysicsEngine {
	static const PxVec3 BrickDimensions = {01.075,00.5125,00.325};
	static const PxReal BrickDensity = 1845;
	static const PxVec3 BrickMaterialParams = { 0.5,0.5,1.0 };
	static const PxVec3 BrickColour = { 0.8,0.05,0.05 };
	static PxMaterial* BrickMat = nullptr;
	class Brick : public Box
	{
	public:	
		Brick(const PxTransform& pose) : Box(pose, BrickDimensions, BrickDensity) 
		{
			if (BrickMat == nullptr) {
				BrickMat = PhysicsEngine::CreateMaterial(BrickMaterialParams);
			}
			Material(BrickMat);
			Color(BrickColour, -1);
		}
	};
}

