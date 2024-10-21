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
	void InitVhacdConvex(const ndHullOutput* const conveHullMesh);
	virtual void InitStaticMeshCompoment(const USceneComponent* const meshComponent) override;

	protected:
	virtual void Serialize(FArchive& Ar) override;

	virtual void ApplyPropertyChanges();
	virtual ndShape* CreateShape() const;
	virtual long long CalculateHash() const;
	virtual ndShapeInstance* CreateInstanceShape() const;
	virtual ndShapeInstance* CreateBodyInstanceShape(const ndMatrix& bodyMatrix) const;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 0.001f))
	float Tolerance;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 16))
	int MaxVertexCount;

	TArray<FVector3f> m_convexHullPoints;
};
	

