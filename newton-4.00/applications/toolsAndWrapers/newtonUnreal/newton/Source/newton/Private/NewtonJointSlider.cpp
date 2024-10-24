// Fill out your copyright notice in the Description page of Project Settings.


#include "NewtonJointSlider.h"

#include "Newton.h"
#include "NewtonRigidBody.h"
#include "NewtonWorldActor.h"
#include "ThirdParty/newtonLibrary/Public/dNewton/ndNewton.h"

UNewtonJointSlider::UNewtonJointSlider()
	:Super()
	,EnableLimits(false)
	,MinDistance(-100.0f)
	,MaxDistance(100.0f)
	,SpringConst(0.0f)
	,DampingConst(0.0f)
	,SpringDamperRegularizer(1.0e-3f)
{
}

void UNewtonJointSlider::DrawGizmo(float timestep) const
{
	ndFloat32 scale = DebugScale * UNREAL_UNIT_SYSTEM;
	const FTransform transform(GetComponentTransform());
	const ndMatrix matrix(ToNewtonMatrix(transform));
	const FColor pinColor(255.0f, 255.0f, 0.0f);
	const ndVector pinDir(matrix.m_front.Scale(scale * 0.9f));
	const FVector pingStart(transform.GetLocation());
	const FVector pingEnd(float(pingStart.X + pinDir.m_x), float(pingStart.Y + pinDir.m_y), float(pingStart.Z + pinDir.m_z));

	const UWorld* const world = GetWorld();
	DrawDebugLine(world, pingStart, pingEnd, pinColor, false, timestep);

	UNewtonRigidBody* const child = FindChild();
	if (EnableLimits && child)
	{
		const ndVector maxLimit(matrix.m_front.Scale(MaxDistance));
		const FVector maxConeStart(pingStart.X + maxLimit.m_x, pingStart.Y + maxLimit.m_y, pingStart.Z + maxLimit.m_z);
		const ndVector minLimit(matrix.m_front.Scale(MinDistance));
		const FVector minConeStart(pingStart.X + minLimit.m_x, pingStart.Y + minLimit.m_y, pingStart.Z + minLimit.m_z);

		const FVector coneDir(matrix.m_front.m_x, matrix.m_front.m_y, matrix.m_front.m_z);
		DrawDebugLine(world, minConeStart, maxConeStart, pinColor, false, timestep);
		DrawDebugCone(world, maxConeStart, coneDir, scale * 0.125f, 15.0f * ndDegreeToRad, 15.0f * ndDegreeToRad, 8, pinColor, false, timestep);
		DrawDebugCone(world, minConeStart, coneDir, -scale * 0.125f, 15.0f * ndDegreeToRad, 15.0f * ndDegreeToRad, 8, pinColor, false, timestep);
	}
}

// Called when the game starts
void UNewtonJointSlider::CreateJoint(ANewtonWorldActor* const newtonWorldActor)
{
	Super::CreateJoint(newtonWorldActor);

	check(!m_joint);
	ndBodyKinematic* body0;
	ndBodyKinematic* body1;
	ndWorld* const world = newtonWorldActor->GetNewtonWorld();
	GetBodyPairs(world, &body0, &body1);
	if (body0 && body1)
	{
		const FTransform transform(GetRelativeTransform());
		const ndMatrix matrix(ToNewtonMatrix(transform) * body1->GetMatrix());

		ndJointSlider* const joint = new ndJointSlider(matrix, body0, body1);

		joint->SetLimitState(EnableLimits);
		joint->SetLimits(ndFloat32(MinDistance * UNREAL_INV_UNIT_SYSTEM), ndFloat32(MaxDistance * UNREAL_INV_UNIT_SYSTEM));
		joint->SetAsSpringDamper(SpringDamperRegularizer, SpringConst, DampingConst);

		m_joint = joint;
		world->AddJoint(m_joint);
	}
}
