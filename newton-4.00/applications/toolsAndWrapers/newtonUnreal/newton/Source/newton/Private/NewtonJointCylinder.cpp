// Fill out your copyright notice in the Description page of Project Settings.


#include "NewtonJointCylinder.h"

#include "Newton.h"
#include "NewtonRigidBody.h"
#include "NewtonWorldActor.h"
#include "ThirdParty/newtonLibrary/Public/dNewton/ndNewton.h"

UNewtonJointCylinder::UNewtonJointCylinder()
	:Super()
	,EnableAngularLimits(false)
	,MinAngle(-45.0f)
	,MaxAngle(45.0f)
	,SpringAngularConst(0.0f)
	,DampingAngularConst(0.0f)
	,SpringDamperAngularRegularizer(1.0e-3f)
	,EnableLinearLimits(false)
	,MinDistance(-100.0f)
	,MaxDistance(100.0f)
	,SpringLinearConst(0.0f)
	,DampingLinearConst(0.0f)
	,SpringDamperLinearRegularizer(1.0e-3f)
{
}

void UNewtonJointCylinder::DrawAngularGizmo(float timestep) const
{
	ndFloat32 scale = DebugScale * UNREAL_UNIT_SYSTEM;
	const FTransform transform(GetComponentTransform());
	const ndMatrix matrix(ToNewtonMatrix(transform));
	const FColor pinColor(255.0f, 255.0f, 0.0f);
	const ndVector pinDir(matrix.m_front.Scale(scale * 0.9f));
	const FVector coneDir(matrix.m_front.m_x, matrix.m_front.m_y, matrix.m_front.m_z);
	const FVector pingStart(transform.GetLocation());
	const FVector pingEnd(float(pingStart.X + pinDir.m_x), float(pingStart.Y + pinDir.m_y), float(pingStart.Z + pinDir.m_z));

	const UWorld* const world = GetWorld();
	DrawDebugLine(world, pingStart, pingEnd, pinColor, false, timestep);
	DrawDebugCone(world, pingEnd, coneDir, -scale * 0.125f, 15.0f * ndDegreeToRad, 15.0f * ndDegreeToRad, 8, pinColor, false, timestep);

	UNewtonRigidBody* const child = FindChild();
	if (EnableAngularLimits && child)
	{
		TArray<int32> indices;
		TArray<FVector> verts;
		float deltaTwist = MaxAngle - MinAngle;
		if ((deltaTwist > 1.0e-3f) && (deltaTwist < 360.0f))
		{
			const ndVector point(ndFloat32(0.0f), scale, ndFloat32(0.0f), ndFloat32(0.0f));
			const ndInt32 subdiv = 12;
			ndFloat32 angle0 = MinAngle;
			ndFloat32 angleStep = ndMin(deltaTwist, 360.0f) / subdiv;

			const FVector parentOrigin(transform.GetLocation());
			const ndMatrix parentMatrix(ToNewtonMatrix(transform));
			verts.Push(parentOrigin);
			for (ndInt32 i = 0; i <= subdiv; ++i)
			{
				const ndVector p(parentMatrix.RotateVector(ndPitchMatrix(angle0 * ndDegreeToRad).RotateVector(point)));
				const FVector p1(float(p.m_x + parentOrigin.X), float(p.m_y + parentOrigin.Y), float(p.m_z + parentOrigin.Z));
				angle0 += angleStep;
				verts.Push(p1);
			}
			for (ndInt32 i = 0; i < subdiv; ++i)
			{
				indices.Push(0);
				indices.Push(i + 1);
				indices.Push(i + 2);
			}
			const FColor meshColor(255.0f, 0.0f, 0.0f, 32.0f);
			DrawDebugMesh(world, verts, indices, meshColor, false, timestep);
		}
	}
}

void UNewtonJointCylinder::DrawLinearGizmo(float timestep) const
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
	if (EnableLinearLimits && child)
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

void UNewtonJointCylinder::DrawGizmo(float timestep) const
{
	UNewtonRigidBody* const child = FindChild();
	if (child)
	{
		DrawLinearGizmo(timestep);
		DrawAngularGizmo(timestep);
	}
}

// Called when the game starts
void UNewtonJointCylinder::CreateJoint(ANewtonWorldActor* const newtonWorldActor)
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
		ndJointCylinder* const joint = new ndJointCylinder(matrix, body0, body1);

		joint->SetLimitStateAngle(EnableAngularLimits);
		joint->SetLimitsAngle(ndFloat32(MinAngle * ndDegreeToRad), ndFloat32(MaxAngle * ndDegreeToRad));
		joint->SetAsSpringDamperAngle(SpringDamperAngularRegularizer, SpringAngularConst, DampingAngularConst);
		
		joint->SetLimitStatePosit(EnableLinearLimits);
		joint->SetLimitsPosit(ndFloat32(MinDistance * UNREAL_INV_UNIT_SYSTEM), ndFloat32(MaxDistance * UNREAL_INV_UNIT_SYSTEM));
		joint->SetAsSpringDamperPosit(SpringDamperLinearRegularizer, SpringLinearConst, DampingLinearConst);
		
		m_joint = joint;
		world->AddJoint(m_joint);
	}
}
