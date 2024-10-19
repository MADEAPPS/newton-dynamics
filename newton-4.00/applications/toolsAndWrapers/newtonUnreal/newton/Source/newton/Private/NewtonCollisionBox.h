// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "NewtonCollision.h"
#include "NewtonCollisionBox.generated.h"

class ndShapeInstance;
class UNewtonRigidBody;

/**
 * 
 */
UCLASS(ClassGroup=(NewtonCollision), meta=(BlueprintSpawnableComponent))
class UNewtonCollisionBox : public UNewtonCollision
{
	GENERATED_BODY()
	
	public:
	// Sets default values for this component's properties
	UNewtonCollisionBox();
	virtual void InitStaticMeshCompoment(const USceneComponent* const meshComponent) override;

	protected:
	virtual void ApplyPropertyChanges();
	virtual ndShape* CreateShape() const;
	virtual long long CalculateHash() const;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 2.0f))
	float SizeX;
	
	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 2.0f))
	float SizeY;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 2.0f))
	float SizeZ;
};
