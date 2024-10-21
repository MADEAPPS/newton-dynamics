// Fill out your copyright notice in the Description page of Project Settings.


#include "NewtonSceneRigidBody.h"
#include "Landscape.h"
#include "LevelEditor.h"
#include "LandscapeProxy.h"

#include "Newton.h"
#include "NewtonWorldActor.h"
#include "NewtonCollisionLandscape.h"
#include "ThirdParty/newtonLibrary/Public/thirdParty/ndConvexApproximation.h"

UNewtonSceneRigidBody::UNewtonSceneRigidBody()
	:Super()
{
	const FTransform transform;
	SetComponentToWorld(transform);
	SetRelativeScale3D_Direct(transform.GetScale3D());
	SetRelativeRotation_Direct(FRotator(transform.GetRotation()));
	SetRelativeLocation_Direct(transform.GetLocation());
}

void UNewtonSceneRigidBody::RemoveAllCollisions()
{
	const TArray<TObjectPtr<USceneComponent>>& chidren = GetAttachChildren();
	for (ndInt32 i = chidren.Num() - 1; i >= 0; --i)
	{
		chidren[i]->DestroyComponent();
	}
}

ndShapeInstance* UNewtonSceneRigidBody::CreateCollision(const ndMatrix& bodyMatrix) const
{
	ndArray<const UNewtonCollision*> subShapes;
	const TArray<TObjectPtr<USceneComponent>>& children = GetAttachChildren();

	for (ndInt32 i = children.Num() - 1; i >= 0; --i)
	{
		const UNewtonCollision* const shape = Cast<UNewtonCollision>(children[i]);
		if (shape)
		{
			subShapes.PushBack(shape);
		}
	}

	if (subShapes.GetCount() == 0)
	{
		return new ndShapeInstance(new ndShapeNull());
	}
	
	if (subShapes.GetCount() == 1)
	{
		return subShapes[0]->CreateBodyInstanceShape(bodyMatrix);
	}
	
	ndShapeInstance* const compoundInstance = new ndShapeInstance(new ndShapeCompound());
	ndShapeCompound* const compound = compoundInstance->GetShape()->GetAsShapeCompound();
	compound->BeginAddRemove();
	for (ndInt32 i = subShapes.GetCount() - 1; i >= 0; --i)
	{
		ndShapeInstance* const shapeInstance = subShapes[i]->CreateBodyInstanceShape(bodyMatrix);
		compound->AddCollision(shapeInstance);
		delete shapeInstance;
	}
	compound->EndAddRemove();
	return compoundInstance;
}
