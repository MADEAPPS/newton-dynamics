// Fill out your copyright notice in the Description page of Project Settings.


#include "NewtonCollisionWheel.h"

#include "Newton.h"
#include "NewtonRigidBody.h"
#include "ThirdParty/newtonLibrary/Public/dNewton/ndNewton.h"

UNewtonCollisionWheel::UNewtonCollisionWheel()
	:Super()
	,Radio(50.0f)
	,Width(50.0f)
{
}

ndShape* UNewtonCollisionWheel::CreateShape() const
{
	return new ndShapeChamferCylinder(0.5f, 1.0f);
}

long long UNewtonCollisionWheel::CalculateHash() const
{
	long long hash = ndCRC64(ndShapeChamferCylinder::StaticClassName(), strlen(ndShapeChamferCylinder::StaticClassName()), 0);
	hash = ndCRC64(&Radio, sizeof(float), hash);
	hash = ndCRC64(&Width, sizeof(float), hash);
	return hash;
}

void UNewtonCollisionWheel::ApplyPropertyChanges()
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

ndShapeInstance* UNewtonCollisionWheel::CreateInstanceShape() const
{
	ndShapeInstance* const instance = Super::CreateInstanceShape();

	const ndVector scale(ndFloat32(Width * UNREAL_INV_UNIT_SYSTEM), ndFloat32(Radio * UNREAL_INV_UNIT_SYSTEM), ndFloat32(Radio * UNREAL_INV_UNIT_SYSTEM), ndFloat32 (0.0f));
	instance->SetScale(scale);

	const ndMatrix aligment(ndYawMatrix(ndPi * 0.5f));
	instance->SetLocalMatrix(aligment * instance->GetLocalMatrix());
	return instance;
}

ndShapeInstance* UNewtonCollisionWheel::CreateBodyInstanceShape(const ndMatrix& bodyMatrix) const
{
	ndShapeInstance* const instance = Super::CreateBodyInstanceShape(bodyMatrix);

	const FVector uScale(GetComponentTransform().GetScale3D());
	const ndVector scale1(ndFloat32(uScale.X), ndFloat32(uScale.Y), ndFloat32(uScale.Z), ndFloat32(1.0f));
	const ndVector scale2(ndFloat32(Width * UNREAL_INV_UNIT_SYSTEM), ndFloat32(Radio * UNREAL_INV_UNIT_SYSTEM), ndFloat32(Radio * UNREAL_INV_UNIT_SYSTEM), ndFloat32(0.0f));
	instance->SetScale(scale1 * scale2);

	const ndMatrix aligment(ndYawMatrix(ndPi * 0.5f));
	instance->SetLocalMatrix(aligment * instance->GetLocalMatrix());
	return instance;
}
