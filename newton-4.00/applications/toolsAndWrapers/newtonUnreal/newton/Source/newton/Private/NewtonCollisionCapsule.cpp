// Fill out your copyright notice in the Description page of Project Settings.


#include "NewtonCollisionCapsule.h"

#include "Newton.h"
#include "NewtonRigidBody.h"
#include "ThirdParty/newtonLibrary/Public/dNewton/ndNewton.h"

UNewtonCollisionCapsule::UNewtonCollisionCapsule()
	:Super()
	,Radio0(50.0f)
	,Radio1(50.0f)
	,Length(100.0f)
{
}

ndShape* UNewtonCollisionCapsule::CreateShape() const
{
	return new ndShapeCapsule(Radio0 * UNREAL_INV_UNIT_SYSTEM, Radio1 * UNREAL_INV_UNIT_SYSTEM, Length * UNREAL_INV_UNIT_SYSTEM);
}

void UNewtonCollisionCapsule::InitStaticMeshCompoment(const USceneComponent* const meshComponent)
{
	const UStaticMeshComponent* const staticMeshComponent = Cast<UStaticMeshComponent>(meshComponent);
	check(staticMeshComponent);
	const UStaticMesh* const staticMesh = staticMeshComponent->GetStaticMesh().Get();
	const UBodySetup* const bodySetup = staticMesh->GetBodySetup();
	const FKAggregateGeom& aggGeom = bodySetup->AggGeom;
	check(aggGeom.SphylElems.Num() == 1);
	const FKSphylElem& element = aggGeom.SphylElems[0];

	SetTransform(meshComponent);
	const FTransform localTransformOffset(element.GetTransform());
	const FTransform globalTransform(localTransformOffset * GetComponentToWorld());
	SetComponentToWorld(globalTransform);

	const AActor* const owner = GetOwner();
	const FTransform bodyTransform(owner->GetRootComponent()->GetComponentToWorld());
	const FTransform localTransform(globalTransform * bodyTransform.Inverse());

	SetRelativeScale3D_Direct(localTransform.GetScale3D());
	SetRelativeRotation_Direct(FRotator(localTransform.GetRotation()));
	SetRelativeLocation_Direct(localTransform.GetLocation());

	Radio0 = element.Radius;
	Radio1 = element.Radius;
	Length = element.Length;
}

long long UNewtonCollisionCapsule::CalculateHash() const
{
	long long hash = ndCRC64(ndShapeCapsule::StaticClassName(), strlen(ndShapeCapsule::StaticClassName()), 0);
	hash = ndCRC64(&Radio0, sizeof(float), hash);
	hash = ndCRC64(&Radio1, sizeof(float), hash);
	hash = ndCRC64(&Length, sizeof(float), hash);
	return hash;
}

void UNewtonCollisionCapsule::ApplyPropertyChanges()
{
	if (BestFit)
	{
		for (USceneComponent* parent = GetAttachParent(); parent; parent = parent->GetAttachParent())
		{
			UStaticMeshComponent* const meshComp = Cast<UStaticMeshComponent>(parent);
			if (meshComp && meshComp->GetStaticMesh())
			{
				//FBoxSphereBounds bounds(meshComp->CalcBounds(FTransform()));
				//SizeX = ndMax(float(bounds.BoxExtent.X * 2.0f), 2.0f);
				//SizeY = ndMax(float(bounds.BoxExtent.Y * 2.0f), 2.0f);
				//SizeZ = ndMax(float(bounds.BoxExtent.Z * 2.0f), 2.0f);
				//SetRelativeTransform(FTransform());
				break;
			}
		}
	}
	BuildNewtonShape();

	Super::ApplyPropertyChanges();
}

ndShapeInstance* UNewtonCollisionCapsule::CreateInstanceShape() const
{
	ndShapeInstance* const  instance = Super::CreateInstanceShape();
	const ndMatrix aligment(ndYawMatrix(ndPi * 0.5f));
	instance->SetLocalMatrix(aligment * instance->GetLocalMatrix());
	return instance;
}

ndShapeInstance* UNewtonCollisionCapsule::CreateBodyInstanceShape(const ndMatrix& bodyMatrix) const
{
	ndShapeInstance* const instance = Super::CreateBodyInstanceShape(bodyMatrix);
	const ndMatrix aligment(ndYawMatrix(ndPi * 0.5f));
	instance->SetLocalMatrix(aligment * instance->GetLocalMatrix());
	return instance;
}
