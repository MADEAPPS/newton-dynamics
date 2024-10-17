// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "NewtonCollision.h"
#include "NewtonCollisionLandscape.generated.h"

class ndShapeInstance;
class UNewtonRigidBody;
class ULandscapeHeightfieldCollisionComponent;

/**
 * 
 */

//UCLASS(Abstract, meta=(BlueprintSpawnableComponent))
UCLASS(ClassGroup= NewtonLandScape, meta=(BlueprintSpawnableComponent))
class UNewtonCollisionLandscape : public UNewtonCollision
{
	GENERATED_BODY()

	public:
	// Sets default values for this component's properties
	UNewtonCollisionLandscape();

	private:
	virtual void ApplyPropertyChanges();
	virtual ndShape* CreateShape() const;
	virtual long long CalculateHash() const;
	virtual ndShapeInstance* CreateInstanceShape() const;
	virtual ndShapeInstance* CreateBodyInstanceShape(const ndMatrix& bodyMatrix) const;
};
