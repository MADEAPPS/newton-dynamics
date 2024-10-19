// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "NewtonCollision.h"
#include "NewtonCollisionPolygonalMesh.generated.h"


/**
 * 
 */
UCLASS(ClassGroup=(NewtonCollision), meta=(BlueprintSpawnableComponent))
class UNewtonCollisionPolygonalMesh : public UNewtonCollision
{
	GENERATED_BODY()

	class ndShapeStatic;
	
	public:
	// Sets default values for this component's properties
	UNewtonCollisionPolygonalMesh();
	virtual void InitStaticMeshCompoment(const USceneComponent* const meshComponent) override;

	protected:
	virtual void Serialize(FArchive& Ar) override;

	virtual void ApplyPropertyChanges();
	virtual ndShape* CreateShape() const;
	virtual long long CalculateHash() const;
	virtual ndShapeInstance* CreateInstanceShape() const;
	virtual ndShapeInstance* CreateBodyInstanceShape(const ndMatrix& bodyMatrix) const;

	TArray<int> m_indexData;
	TArray<FVector3f> m_vetexData;
};
