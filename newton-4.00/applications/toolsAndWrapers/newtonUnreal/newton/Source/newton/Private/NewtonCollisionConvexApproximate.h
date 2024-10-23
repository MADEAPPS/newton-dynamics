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
	virtual void ApplyPropertyChanges() override;
	virtual ndShape* CreateShape() const override;
	virtual long long CalculateHash() const override;
	virtual ndShapeInstance* CreateInstanceShape() const override;
	virtual ndVector GetVolumePosition(const ndMatrix& bodyMatrix) const override;
	virtual ndShapeInstance* CreateBodyInstanceShape(const ndMatrix& bodyMatrix) const override;
	
	long long CalculateStaticMeshHash() const;
	const FStaticMeshLODResources* GetRenderLOD() const;
	ndConvexHullSet* CreateConvexApproximationShapes() const;

	virtual void Serialize(FArchive& ar) override;
	void SerializeLoadRevision_firstVersion(FArchive& ar);

	UPROPERTY(EditAnywhere, Category = Newton)
	bool Generate;

	UPROPERTY(EditAnywhere, Category = Newton)
	bool HighResolution;

	UPROPERTY(VisibleAnywhere, Category = Newton)
	int NumberOfConvex;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 1, ClampMax = 128))
	int MaxVertexPerConvex;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 1, ClampMax = 128))
	int MaxConvexes;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 0.0, ClampMax = 1.0))
	float Tolerance;
	
	long long m_meshHash;
	TSharedPtr<ndConvexHullSet> m_convexHullSet;
	bool m_generateFlipFlop;
};
