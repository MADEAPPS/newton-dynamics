// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "NewtonRigidBody.h"
#include "NewtonSceneRigidBody.generated.h"

/**
 * 
 */
UCLASS(ClassGroup = NewtonScene, meta=(BlueprintSpawnableComponent), HideCategories = (Physics, Collision), MinimalAPI)
class UNewtonSceneRigidBody : public UNewtonRigidBody
{
	GENERATED_BODY()

	public:
	UNewtonSceneRigidBody();

	void RemoveAllCollisions();
	virtual ndShapeInstance* CreateCollision(const ndMatrix& bodyMatrix) const override;
};
