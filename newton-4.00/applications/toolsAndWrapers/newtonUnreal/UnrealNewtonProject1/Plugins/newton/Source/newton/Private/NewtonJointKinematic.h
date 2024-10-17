// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "NewtonJoint.h"
#include "NewtonJointKinematic.generated.h"

class ndWorld;
class ANewtonWorldActor;
/**
 * 
 */
UCLASS(ClassGroup=(NewtonJoints), meta=(BlueprintSpawnableComponent))
class UNewtonJointKinematic : public UNewtonJoint
{
	GENERATED_BODY()
	
	public:
	// Sets default values for this component's properties
	UNewtonJointKinematic();

	protected:
	virtual void DrawGizmo(float timestep) const override;
	virtual void CreateJoint(ANewtonWorldActor* const worldActor) override;
};
