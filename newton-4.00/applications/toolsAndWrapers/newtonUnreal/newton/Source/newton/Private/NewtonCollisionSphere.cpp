// Fill out your copyright notice in the Description page of Project Settings.


#include "NewtonCollisionSphere.h"

#include "Newton.h"
#include "NewtonRigidBody.h"
#include "ThirdParty/newtonLibrary/Public/dNewton/ndNewton.h"

UNewtonCollisionSphere::UNewtonCollisionSphere()
	:Super()
	,Radio(50.0f)
{
}

ndShape* UNewtonCollisionSphere::CreateShape() const
{
	return new ndShapeSphere(Radio * UNREAL_INV_UNIT_SYSTEM);
}

void UNewtonCollisionSphere::InitStaticMeshCompoment(const USceneComponent* const meshComponent)
{
	const UStaticMeshComponent* const staticMeshComponent = Cast<UStaticMeshComponent>(meshComponent);
	check(staticMeshComponent);
	const UStaticMesh* const staticMesh = staticMeshComponent->GetStaticMesh().Get();
	const UBodySetup* const bodySetup = staticMesh->GetBodySetup();
	const FKAggregateGeom& aggGeom = bodySetup->AggGeom;
	check(aggGeom.SphereElems.Num() == 1);
	const FKSphereElem& element = aggGeom.SphereElems[0];

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

	Radio = element.Radius;
}

long long UNewtonCollisionSphere::CalculateHash() const
{
	long long hash = ndCRC64(ndShapeSphere::StaticClassName(), strlen(ndShapeSphere::StaticClassName()), 0);
	hash = ndCRC64(&Radio, sizeof(float), hash);
	return hash;
}

void UNewtonCollisionSphere::ApplyPropertyChanges()
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
