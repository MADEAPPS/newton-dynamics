// Fill out your copyright notice in the Description page of Project Settings.


#include "NewtonJoint.h"

#include "NewtonRigidBody.h"
#include "ThirdParty/newtonLibrary/Public/dNewton/ndNewton.h"

// Sets default values for this component's properties
UNewtonJoint::UNewtonJoint()
	:Super()
	,ShowDebug(false)
	,DebugScale(1.0f)
	//,LoopNewtonStaticMesh0()
	//,LoopNewtonStaticMesh1()
	,m_joint(nullptr)
	,m_propertyChanged(true)
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;
}

void UNewtonJoint::ClearDebug()
{
	ShowDebug = false;
	m_propertyChanged = true;
}

void UNewtonJoint::ActivateDebug()
{
	ShowDebug = true;
	m_propertyChanged = true;
}

void UNewtonJoint::CreateJoint(ANewtonWorldActor* const)
{
	m_transfrom = GetComponentTransform();
	ApplyPropertyChanges();
}

void UNewtonJoint::DestroyJoint()
{
	if (m_joint)
	{
		ndBodyKinematic* const body = m_joint->GetBody0();
		check(body);

		ndWorld* const world = body->GetScene()->GetWorld();
		check(world);

		world->RemoveJoint(m_joint);
		m_joint = nullptr;
	}
}

UNewtonRigidBody* UNewtonJoint::FindChild() const
{
	const TArray<TObjectPtr<USceneComponent>>& childArray = GetAttachChildren();
	if (childArray.Num())
	{
		check(childArray.Num() == 1);
		return Cast<UNewtonRigidBody>(childArray[0]);
	}
	return nullptr;
}

UNewtonRigidBody* UNewtonJoint::FindParent() const
{
	return Cast<UNewtonRigidBody>(GetAttachParent());
}

void UNewtonJoint::GetBodyPairs(ndWorld* const world, ndBodyKinematic** body0, ndBodyKinematic** body1) const
{
	*body0 = nullptr;
	*body1 = nullptr;

	UNewtonRigidBody* const child = FindChild();
	if (child)
	{
		check(child->m_body);
		*body0 = child->m_body->GetAsBodyKinematic();
	}

	UNewtonRigidBody* const parent = FindParent();
	if (parent)
	{
		check(parent->m_body);
		*body1 = parent->m_body->GetAsBodyKinematic();
	}
	else
	{
		*body1 = world->GetSentinelBody();
	}

	if (!body0)
	{
		UE_LOG(LogTemp, Warning, TEXT("The child of a: %s must be a UNewtonRigidBody. Failed to create joint"), *GetName());
	}

	if (!body1)
	{
		UE_LOG(LogTemp, Warning, TEXT("The parent of a: %s must be a UNewtonRigidBody. Failed to create joint"), *GetName());
	}
}

void UNewtonJoint::DrawGizmo(float timestep) const
{
	check(0);
}

void UNewtonJoint::UpdateTransform()
{
	UNewtonRigidBody* const parent = FindParent();
	if (parent)
	{
		m_transfrom = GetRelativeTransform() * parent->m_globalTransform;
	}
}

void UNewtonJoint::ApplyPropertyChanges()
{
	m_propertyChanged = false;
	const FTransform transform(GetComponentTransform());
	const UNewtonRigidBody* const child = FindChild();
	if (child)
	{
		const FTransform childTransform(child->GetComponentTransform());
		m_localChildTransfrom = transform * childTransform.Inverse();
	}
	
	const UNewtonRigidBody* const parent = FindParent();
	if (parent)
	{
		const FTransform parentTransform(parent->GetComponentTransform());
		m_localParentTransfrom = transform * parentTransform.Inverse();
	}
}

void UNewtonJoint::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);
	m_propertyChanged = true;
}

// Called every frame
void UNewtonJoint::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	SetComponentToWorld(m_transfrom);
	if (ShowDebug)
	{
		DrawGizmo(DeltaTime);
	}
}
