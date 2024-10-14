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
