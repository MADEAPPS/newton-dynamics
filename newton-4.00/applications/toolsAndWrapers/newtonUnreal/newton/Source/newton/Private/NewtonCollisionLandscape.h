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
	virtual void InitStaticMeshCompoment(const USceneComponent* const meshComponent) override;

	protected:
	virtual void Serialize(FArchive& Ar) override;
	virtual void ApplyPropertyChanges() override;
	virtual ndShape* CreateShape() const override;
	virtual long long CalculateHash() const override;
	virtual ndShapeInstance* CreateInstanceShape() const override;
	virtual ndShapeInstance* CreateBodyInstanceShape(const ndMatrix& bodyMatrix) const override;;

	TArray<char> m_materials;
	TArray<float> m_heightfield;
	float m_scale_x;
	float m_scale_y;
	float m_scale_z;
	int m_tileSize_x;
	int m_tileSize_y;
};
