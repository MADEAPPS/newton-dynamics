// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "NewtonJoint.h"
#include "NewtonJointCylinder.generated.h"

class ANewtonWorldActor;

/**
 * 
 */
UCLASS(ClassGroup=(NewtonJoints), meta=(BlueprintSpawnableComponent))
class UNewtonJointCylinder : public UNewtonJoint
{
	GENERATED_BODY()

	public:
	// Sets default values for this component's properties
	UNewtonJointCylinder();

	protected:
	void DrawLinearGizmo(float timestep) const;
	void DrawAngularGizmo(float timestep) const;
	virtual void DrawGizmo(float timestep) const override;
	virtual void CreateJoint(ANewtonWorldActor* const worldActor) override;

	UPROPERTY(EditAnywhere, Category = Newton)
	bool EnableAngularLimits;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = -10000.0f, ClampMax = -1.0f))
	float MinAngle;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 1.0f, ClampMax = 10000.0f))
	float MaxAngle;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 0.0f, ClampMax = 10000.0f))
	float SpringAngularConst;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 0.0f, ClampMax = 1000.0f))
	float DampingAngularConst;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 0.0001f, ClampMax = 1.0f))
	float SpringDamperAngularRegularizer;

	UPROPERTY(EditAnywhere, Category = Newton)
	bool EnableLinearLimits;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = -10000.0f, ClampMax = -0.01f))
	float MinDistance;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 0.01f, ClampMax = 10000.0f))
	float MaxDistance;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 0.0f, ClampMax = 10000.0f))
	float SpringLinearConst;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 0.0f, ClampMax = 1000.0f))
	float DampingLinearConst;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 0.0001f, ClampMax = 1.0f))
	float SpringDamperLinearRegularizer;
};
