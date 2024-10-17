// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "NewtonJoint.h"
#include "NewtonJointSlider.generated.h"


class ndWorld;
class ANewtonWorldActor;
/**
 * 
 */

UCLASS(ClassGroup=(NewtonJoints), meta=(BlueprintSpawnableComponent))
class UNewtonJointSlider : public UNewtonJoint
{
	GENERATED_BODY()
	
	public:
	// Sets default values for this component's properties
	UNewtonJointSlider();

	protected:
	virtual void DrawGizmo(float timestep) const override;
	virtual void CreateJoint(ANewtonWorldActor* const worldActor) override;

	UPROPERTY(EditAnywhere, Category = Newton)
	bool EnableLimits;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = -10000.0f, ClampMax = -0.01f))
	float MinDistance;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 0.01f, ClampMax = 10000.0f))
	float MaxDistance;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 0.0f, ClampMax = 10000.0f))
	float SpringConst;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 0.0f, ClampMax = 1000.0f))
	float DampingConst;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 0.0001f, ClampMax = 1.0f))
	float SpringDamperRegularizer;
};
