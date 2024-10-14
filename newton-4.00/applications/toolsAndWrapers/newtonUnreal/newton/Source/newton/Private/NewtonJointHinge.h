// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "NewtonJoint.h"
#include "NewtonJointHinge.generated.h"

class ndWorld;
class ANewtonWorldActor;
/**
 * 
 */
UCLASS(ClassGroup=(NewtonJoints), meta=(BlueprintSpawnableComponent))
class UNewtonJointHinge : public UNewtonJoint
{
	GENERATED_BODY()
	
	public:
	// Sets default values for this component's properties
	UNewtonJointHinge();

	protected:
	virtual void DrawGizmo(float timestep) const override;
	virtual void CreateJoint(ANewtonWorldActor* const worldActor) override;

	UPROPERTY(EditAnywhere, Category = Newton)
	bool EnableLimits;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = -10000.0f, ClampMax = -1.0f))
	float MinAngle;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 1.0f, ClampMax = 10000.0f))
	float MaxAngle;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 0.0f, ClampMax = 10000.0f))
	float SpringConst;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 0.0f, ClampMax = 1000.0f))
	float DampingConst;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 0.0001f, ClampMax = 1.0f))
	float SpringDamperRegularizer;
};
