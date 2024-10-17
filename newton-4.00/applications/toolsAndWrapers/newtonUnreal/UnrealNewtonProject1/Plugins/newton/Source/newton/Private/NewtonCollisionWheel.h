// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "NewtonCollision.h"
#include "NewtonCollisionWheel.generated.h"

class ndShapeInstance;
class UNewtonRigidBody;

/**
 * 
 */
UCLASS(ClassGroup=(NewtonCollision), meta=(BlueprintSpawnableComponent))
class UNewtonCollisionWheel : public UNewtonCollision
{
	GENERATED_BODY()
	
	public:
	// Sets default values for this component's properties
	UNewtonCollisionWheel();

	protected:
	virtual void ApplyPropertyChanges();
	virtual ndShape* CreateShape() const;
	virtual long long CalculateHash() const;
	virtual ndShapeInstance* CreateInstanceShape() const override;
	virtual ndShapeInstance* CreateBodyInstanceShape(const ndMatrix& bodyMatrix) const override;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 2.0f))
	float Radio;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 2.0f))
	float Width;
};
