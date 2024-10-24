// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "NewtonCollision.h"
#include "NewtonCollisionConvexHull.generated.h"

class ndHullOutput;
class ndShapeInstance;
class UNewtonRigidBody;

/**
 * 
 */
UCLASS(ClassGroup=(NewtonCollision), meta=(BlueprintSpawnableComponent))
class UNewtonCollisionConvexHull : public UNewtonCollision
{
	GENERATED_BODY()
	public:
	// Sets default values for this component's properties
	UNewtonCollisionConvexHull();

	virtual void InitStaticMeshCompoment(const USceneComponent* const meshComponent) override;
	
	protected:
	virtual void Serialize(FArchive& ar) override;

	virtual void ApplyPropertyChanges() override;
	virtual ndShape* CreateShape() const override;
	virtual long long CalculateHash() const override;

	void GenerateMesh(const USceneComponent* const meshComponent);
	virtual ndShapeInstance* CreateInstanceShape() const override;
	virtual ndShapeInstance* CreateBodyInstanceShape(const ndMatrix& bodyMatrix) const override;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 0.001f))
	float Tolerance;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 16))
	int MaxVertexCount;

	TArray<FVector3f> m_convexHullPoints;
};
	

