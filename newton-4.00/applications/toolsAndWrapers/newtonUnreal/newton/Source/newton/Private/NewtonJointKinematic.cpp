// Fill out your copyright notice in the Description page of Project Settings.


#include "NewtonJointKinematic.h"

#include "Newton.h"
#include "NewtonRigidBody.h"
#include "NewtonWorldActor.h"
#include "ThirdParty/newtonLibrary/Public/dNewton/ndNewton.h"

UNewtonJointKinematic::UNewtonJointKinematic()
	:Super()
{
}

void UNewtonJointKinematic::DrawGizmo(float timestep) const
{
	//ndFloat32 scale = DebugScale * UNREAL_UNIT_SYSTEM;
	//const FTransform transform(GetComponentTransform());
	//const ndMatrix matrix(UNewtonRigidBody::ToNewtonMatrix(transform));
	//const FColor pinColor(255.0f, 255.0f, 0.0f);
	//const ndVector pinDir(matrix.m_front.Scale(scale * 0.9f));
	//const FVector coneDir(matrix.m_front.m_x, matrix.m_front.m_y, matrix.m_front.m_z);
	//const FVector pingStart(transform.GetLocation());
	//const FVector pingEnd(float(pingStart.X + pinDir.m_x), float(pingStart.Y + pinDir.m_y), float(pingStart.Z + pinDir.m_z));
	//
	//const UWorld* const world = GetWorld();
	//DrawDebugLine(world, pingStart, pingEnd, pinColor, false, timestep);
	//DrawDebugCone(world, pingEnd, coneDir, -scale * 0.125f, 15.0f * ndDegreeToRad, 15.0f * ndDegreeToRad, 8, pinColor, false, timestep);
	//
	//UNewtonRigidBody* const child = FindChild();
	//if (EnableLimits && child)
	//{
	//	TArray<int32> indices;
	//	TArray<FVector> verts;
	//	float deltaTwist = MaxAngle - MinAngle;
	//	if ((deltaTwist > 1.0e-3f) && (deltaTwist < 360.0f))
	//	{
	//		const ndVector point(ndFloat32(0.0f), scale, ndFloat32(0.0f), ndFloat32(0.0f));
	//		const ndInt32 subdiv = 12;
	//		ndFloat32 angle0 = MinAngle;
	//		ndFloat32 angleStep = ndMin(deltaTwist, 360.0f) / subdiv;
	//
	//		const FVector parentOrigin(transform.GetLocation());
	//		const ndMatrix parentMatrix(UNewtonRigidBody::ToNewtonMatrix(transform));
	//		verts.Push(parentOrigin);
	//		for (ndInt32 i = 0; i <= subdiv; ++i)
	//		{
	//			const ndVector p(parentMatrix.RotateVector(ndPitchMatrix(angle0 * ndDegreeToRad).RotateVector(point)));
	//			const FVector p1(float(p.m_x + parentOrigin.X), float(p.m_y + parentOrigin.Y), float(p.m_z + parentOrigin.Z));
	//			angle0 += angleStep;
	//			verts.Push(p1);
	//		}
	//		for (ndInt32 i = 0; i < subdiv; ++i)
	//		{
	//			indices.Push(0);
	//			indices.Push(i + 1);
	//			indices.Push(i + 2);
	//		}
	//		const FColor meshColor(255.0f, 0.0f, 0.0f, 32.0f);
	//		DrawDebugMesh(world, verts, indices, meshColor, false, timestep);
	//	}
	//}
}

// Called when the game starts
void UNewtonJointKinematic::CreateJoint(ANewtonWorldActor* const newtonWorldActor)
{
	Super::CreateJoint(newtonWorldActor);

	check(0);
	//check(!m_joint);
	//ndBodyKinematic* body0;
	//ndBodyKinematic* body1;
	//ndWorld* const world = newtonWorldActor->GetNewtonWorld();
	//GetBodyPairs(world, &body0, &body1);
	//
	//if (body0 && body1)
	//{
	//	const FTransform transform(GetRelativeTransform());
	//	const ndMatrix matrix(UNewtonRigidBody::ToNewtonMatrix(transform) * body1->GetMatrix());
	//	ndJointHinge* const joint = new ndJointHinge(matrix, body0, body1);
	//
	//	joint->SetLimitState(EnableLimits);
	//	joint->SetLimits(ndFloat32(MinAngle * ndDegreeToRad), ndFloat32(MaxAngle * ndDegreeToRad));
	//	joint->SetAsSpringDamper(SpringDamperRegularizer, SpringConst, DampingConst);
	//
	//	m_joint = joint;
	//	world->AddJoint(m_joint);
	//}
}


