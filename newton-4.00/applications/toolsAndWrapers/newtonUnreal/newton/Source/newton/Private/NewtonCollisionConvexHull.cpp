// Fill out your copyright notice in the Description page of Project Settings.


#include "NewtonCollisionConvexHull.h"

#include "Newton.h"
#include "NewtonRigidBody.h"
#include "NewtonSceneActor.h"
#include "ThirdParty/newtonLibrary/Public/dNewton/ndNewton.h"
#include "ThirdParty/newtonLibrary/Public/thirdParty/ndConvexApproximation.h"

UNewtonCollisionConvexHull::UNewtonCollisionConvexHull()
	:Super()
	,Tolerance(1.0e-3f)
	,MaxVertexCount(128)
{
}

void UNewtonCollisionConvexHull::Serialize(FArchive& ar)
{
	Super::Serialize(ar);
	ar << m_convexHullPoints;
}

long long UNewtonCollisionConvexHull::CalculateHash() const
{
	long long hash = ndCRC64(ndShapeConvexHull::StaticClassName(), strlen(ndShapeConvexHull::StaticClassName()), 0);
	if (m_convexHullPoints.Num())
	{
		const FVector3f* const vexterBuffer = &m_convexHullPoints[0];
		hash = ndCRC64(vexterBuffer, m_convexHullPoints.Num() * sizeof(FVector3f), hash);
	}
	return hash;
}

ndShape* UNewtonCollisionConvexHull::CreateShape() const
{
	if (m_convexHullPoints.Num())
	{
		ndArray<ndVector> points;
		for (ndInt32 i = m_convexHullPoints.Num() - 1; i >= 0; --i)
		{
			const ndVector p(ndFloat32(m_convexHullPoints[i].X), ndFloat32(m_convexHullPoints[i].Y), ndFloat32(m_convexHullPoints[i].Z), ndFloat32(0.0f));
			points.PushBack(p);
		}
		ndShape* const shape = new ndShapeConvexHull(m_convexHullPoints.Num(), sizeof(ndVector), Tolerance, &points[0].m_x, MaxVertexCount);
		return shape;
	}
	return new ndShapeNull();
}

void UNewtonCollisionConvexHull::GenerateMesh(const USceneComponent* const meshComponent)
{
	const UStaticMeshComponent* const staticMeshComponent = Cast<UStaticMeshComponent>(meshComponent);
	check(staticMeshComponent);
	UStaticMesh* const staticMesh = staticMeshComponent->GetStaticMesh().Get();

	const FStaticMeshRenderData* const renderData = staticMesh->GetRenderData();
	const FStaticMeshLODResourcesArray& renderResource = renderData->LODResources;

	const FVector uScale(meshComponent->GetComponentTransform().GetScale3D());
	const ndVector scale(ndFloat32(uScale.X), ndFloat32(uScale.Y), ndFloat32(uScale.Z), ndFloat32(0.0f));
	const ndVector bakedScale(scale.Scale(UNREAL_INV_UNIT_SYSTEM));

	const FStaticMeshLODResources& renderLOD = renderResource[0];
	const FStaticMeshVertexBuffers& staticMeshVertexBuffer = renderLOD.VertexBuffers;;
	const FPositionVertexBuffer& positBuffer = staticMeshVertexBuffer.PositionVertexBuffer;

	ndArray<ndBigVector> points;
	for (ndInt32 i = positBuffer.GetNumVertices() - 1; i >= 0; --i)
	{
		const FVector3f p(positBuffer.VertexPosition(i));
		const ndVector q(ndFloat32(p.X), ndFloat32(p.Y), ndFloat32(p.Z), ndFloat32(1.0f));
		points.PushBack(q * bakedScale);
	}
	ndConvexHull3d convexHull(&points[0].m_x, sizeof(ndBigVector), points.GetCount(), Tolerance, MaxVertexCount);
	const ndArray<ndBigVector>& convexVertex = convexHull.GetVertexPool();

	for (ndInt32 i = convexVertex.GetCount() - 1; i >= 0; --i)
	{
		FVector3f p(convexVertex[i].m_x, convexVertex[i].m_y, convexVertex[i].m_z);
		m_convexHullPoints.Push(p);
	}
}

void UNewtonCollisionConvexHull::SetTransform(const USceneComponent* const meshComponent)
{
	//AActor* xxx0 = GetOwner();
	//AActor* xxx1 = meshComponent->GetOwner();
	FTransform bodyTransform(GetComponentTransform());
	for (USceneComponent* parent = GetAttachParent(); parent; parent = parent->GetAttachParent())
	{
		bodyTransform = parent->GetComponentTransform();
		if (Cast<UNewtonRigidBody>(parent))
		{
			break;
		}
	}
	FTransform meshGlobalTransform(meshComponent->GetComponentToWorld());
	meshGlobalTransform.SetScale3D(FVector(1.0f, 1.0f, 1.0f));
	const FTransform localTransform(meshGlobalTransform * bodyTransform.Inverse());

	SetComponentToWorld(meshGlobalTransform);

	// for some reason, this does not work in the unreal editor
	SetRelativeScale3D_Direct(localTransform.GetScale3D());
	SetRelativeRotation_Direct(FRotator(localTransform.GetRotation()));
	SetRelativeLocation_Direct(localTransform.GetLocation());
}

void UNewtonCollisionConvexHull::InitStaticMeshCompoment(const USceneComponent* const meshComponent)
{
	SetTransform(meshComponent);
	GenerateMesh(meshComponent);
}

void UNewtonCollisionConvexHull::ApplyPropertyChanges()
{
	if (!m_convexHullPoints.Num())
	{
		if (!Cast<ANewtonSceneActor>(GetOwner()))
		{
			const USceneComponent* const parent = Cast<UStaticMeshComponent>(GetAttachParent());
			if (parent)
			{
				const FTransform localTransform;
				SetComponentToWorld(parent->GetComponentToWorld());
				SetRelativeScale3D_Direct(localTransform.GetScale3D());
				SetRelativeRotation_Direct(FRotator(localTransform.GetRotation()));
				SetRelativeLocation_Direct(localTransform.GetLocation());
				GenerateMesh(parent);
			}
		}
	}
	m_debugVisualIsDirty = true;

	BuildNewtonShape();
	Super::ApplyPropertyChanges();
}

ndShapeInstance* UNewtonCollisionConvexHull::CreateInstanceShape() const
{
	ndShapeInstance* const instance = new ndShapeInstance(m_shape);
	instance->SetScale(ndVector(ndFloat32(1.0f)));
	instance->SetLocalMatrix(ndGetIdentityMatrix());
	return instance;
}

ndShapeInstance* UNewtonCollisionConvexHull::CreateBodyInstanceShape(const ndMatrix& bodyMatrix) const
{
	ndShapeInstance* const instance = new ndShapeInstance(m_shape);

	const FTransform transform(GetComponentToWorld());
	const ndMatrix matrix(ToNewtonMatrix(transform) * bodyMatrix.OrthoInverse());

	instance->SetLocalMatrix(matrix);
	instance->SetScale(ndVector(ndFloat32(1.0f)));
	return instance;
}
