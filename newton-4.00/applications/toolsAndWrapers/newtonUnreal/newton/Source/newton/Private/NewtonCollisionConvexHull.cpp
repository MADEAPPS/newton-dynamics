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

	// this actually sucks big time in unreal
	//ar.UsingCustomVersion(FnewtonModule::m_guiID);
	//int xxxx0 = ar.CustomVer(FnewtonModule::m_guiID);
	
	ar << m_proceduralData;
}

void UNewtonCollisionConvexHull::SetProceduralData(const ndHullOutput& pointCloud)
{
	m_proceduralData.Empty();
	for (ndInt32 i = pointCloud.GetCount() - 1; i >= 0; --i)
	{
		FVector3f p(pointCloud[i].m_x, pointCloud[i].m_y, pointCloud[i].m_z);
		m_proceduralData.Push(p);
	}
}

long long UNewtonCollisionConvexHull::CalculateHash() const
{
	long long hash = ndCRC64(ndShapeConvexHull::StaticClassName(), strlen(ndShapeConvexHull::StaticClassName()), 0);
	if (m_proceduralData.Num())
	{
		const FVector3f* const vexterBuffer = &m_proceduralData[0];
		hash = ndCRC64(vexterBuffer, m_proceduralData.Num() * sizeof(FVector3f), hash);
	}
	return hash;
}

ndShape* UNewtonCollisionConvexHull::CreateShape() const
{
	if (m_proceduralData.Num())
	{
		ndArray<ndVector> points;
		for (int i = m_proceduralData.Num() - 1; i >= 0; --i)
		{
			const ndVector p(ndFloat32(m_proceduralData[i].X), ndFloat32(m_proceduralData[i].Y), ndFloat32(m_proceduralData[i].Z), ndFloat32(0.0f));
			points.PushBack(p);
		}
		ndShape* const shape = new ndShapeConvexHull(m_proceduralData.Num(), sizeof(ndVector), Tolerance, &points[0].m_x, MaxVertexCount);
		return shape;
	}
	return new ndShapeNull();
}

void UNewtonCollisionConvexHull::ApplyPropertyChanges()
{
	check(0);
	//if (m_proceduralData.Num() == 0)
	//{
	//	m_debugVisualIsDirty = true;
	//
	//	const UStaticMesh* const staticMesh = FindStaticMesh();
	//	if (staticMesh)
	//	{
	//		const ANewtonSceneActor* const owner = Cast<ANewtonSceneActor>(GetOwner());
	//		if (owner && m_savedMeshComponent)
	//		{
	//			const FTransform bodyTransform(owner->GetRootComponent()->GetComponentToWorld());
	//			const FTransform meshTransform(m_savedMeshComponent->GetComponentToWorld());
	//			const FTransform transform(meshTransform * bodyTransform.Inverse());
	//			SetComponentToWorld(transform);
	//		}
	//
	//		const FStaticMeshRenderData* const renderData = staticMesh->GetRenderData();
	//		const FStaticMeshLODResourcesArray& renderResource = renderData->LODResources;
	//
	//		const FVector uScale(GetComponentTransform().GetScale3D());
	//		const ndVector scale(ndFloat32(uScale.X), ndFloat32(uScale.Y), ndFloat32(uScale.Z), ndFloat32(0.0f));
	//		const ndVector bakedScale(scale.Scale(UNREAL_INV_UNIT_SYSTEM));
	//
	//		const FStaticMeshLODResources& renderLOD = renderResource[0];
	//		const FStaticMeshVertexBuffers& staticMeshVertexBuffer = renderLOD.VertexBuffers;;
	//		const FPositionVertexBuffer& positBuffer = staticMeshVertexBuffer.PositionVertexBuffer;
	//
	//		ndArray<ndBigVector> points;
	//		for (int i = positBuffer.GetNumVertices() - 1; i >= 0; --i)
	//		{
	//			const FVector3f p(positBuffer.VertexPosition(i));
	//			const ndVector q(ndFloat32(p.X), ndFloat32(p.Y), ndFloat32(p.Z), ndFloat32(0.0f));
	//			points.PushBack(q * bakedScale);
	//		}
	//		ndConvexHull3d convexHull(&points[0].m_x, sizeof(ndBigVector), points.GetCount(), Tolerance, MaxVertexCount);
	//		const ndArray<ndBigVector>& convexVertex = convexHull.GetVertexPool();
	//
	//		ndHullOutput hullMesh;
	//		for (ndInt32 i = convexVertex.GetCount() - 1; i >= 0; --i)
	//		{
	//			ndHullPoint p;
	//			p.m_x = convexVertex[i].m_x;
	//			p.m_y = convexVertex[i].m_y;
	//			p.m_z = convexVertex[i].m_z;
	//			hullMesh.PushBack(p);
	//		}
	//		SetProceduralData(hullMesh);
	//	}
	//}

	BuildNewtonShape();
	Super::ApplyPropertyChanges();
}

ndShapeInstance* UNewtonCollisionConvexHull::CreateBodyInstanceShape(const ndMatrix& bodyMatrix) const
{
	ndShapeInstance* const instance = CreateInstanceShape();

	const ndVector scale(ndFloat32(1.0f));
	const FTransform transform(GetComponentToWorld());
	const ndMatrix matrix(UNewtonRigidBody::ToNewtonMatrix(transform) * bodyMatrix.OrthoInverse());

	instance->SetScale(scale);
	instance->SetLocalMatrix(matrix);
	return instance;
}

ndShapeInstance* UNewtonCollisionConvexHull::CreateInstanceShape() const
{
	ndShapeInstance* const instance = new ndShapeInstance(m_shape);
	const FVector uScale(GetComponentTransform().GetScale3D());
	const ndVector scale(ndFloat32(1.0f / uScale.X), ndFloat32(1.0f / uScale.Y), ndFloat32(1.0f / uScale.Z), ndFloat32(0.0f));
	instance->SetScale(scale);
	return instance;
}
