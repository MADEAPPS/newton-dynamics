// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

#include "NewtonCommons.h"
#include "NewtonCollision.h"
#include "NewtonCollisionConvexApproximate.generated.h"

/**
 * 
 */
UCLASS(ClassGroup=(NewtonCollision), meta=(BlueprintSpawnableComponent))
class UNewtonCollisionConvexApproximate : public UNewtonCollision
{
	GENERATED_BODY()
	class ConvexVhacdGenerator;

	public:
	// Sets default values for this component's properties
	UNewtonCollisionConvexApproximate();
	virtual void InitStaticMeshCompoment(const USceneComponent* const meshComponent) override;
	
	protected:
	virtual void ApplyPropertyChanges();
	virtual ndShape* CreateShape() const;
	virtual long long CalculateHash() const;
	virtual void Serialize(FArchive& ar) override;
	virtual ndShapeInstance* CreateInstanceShape() const;
	virtual ndShapeInstance* CreateBodyInstanceShape(const ndMatrix& bodyMatrix) const;

	long long CalculateStaticMeshHash() const;
	ndConvexHullSet* CreateConvexApproximationShapes() const;

	UPROPERTY(EditAnywhere, Category = Newton)
	bool Generate = false;

	UPROPERTY(EditAnywhere, Category = Newton)
	bool HighResolution = false;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 1, ClampMax = 128))
	int MaxVertexPerConvex = 32;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 1, ClampMax = 128))
	int MaxConvexes = 16;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 0.0, ClampMax = 1.0))
	float Tolerance = 0.0f;

	bool m_generateFlipFlop;
	TSharedPtr<ndConvexHullSet> m_convexHullSet;
};
