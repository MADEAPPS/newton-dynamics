// Fill out your copyright notice in the Description page of Project Settings.


#include "NewtonCollisionPolygonalMesh.h"

#include "Newton.h"
#include "NewtonRigidBody.h"
#include "ThirdParty/newtonLibrary/Public/dNewton/ndNewton.h"

UNewtonCollisionPolygonalMesh::UNewtonCollisionPolygonalMesh()
	:Super()
{
}

long long UNewtonCollisionPolygonalMesh::CalculateHash() const
{
	auto FindStaticMesh = [this]()
	{
		UStaticMeshComponent* const mesh = Cast<UStaticMeshComponent>(GetAttachParent());
		if (mesh && mesh->GetStaticMesh().Get())
		{
			return mesh->GetStaticMesh().Get();
		}
		return (UStaticMesh*) nullptr;
	};

	long long hash = ndCRC64(ndShapeHeightfield::StaticClassName(), strlen(ndShapeHeightfield::StaticClassName()), 0);

	UStaticMesh* const staticMesh = FindStaticMesh();
	if (staticMesh)
	{
		#if 1
		FTriMeshCollisionData collisionData;
		bool data = staticMesh->GetPhysicsTriMeshData(&collisionData, true);
		if (data)
		{ 
			const FVector uScale(GetComponentTransform().GetScale3D());
			const ndVector scale(ndFloat32(uScale.X), ndFloat32(uScale.Y), ndFloat32(uScale.Z), ndFloat32(0.0f));
			const ndVector bakedScale(scale.Scale(UNREAL_INV_UNIT_SYSTEM));

			for (int i = collisionData.Vertices.Num() - 1; i >= 0; --i)
			{
				const FVector3f p(collisionData.Vertices[i]);
				const ndVector q(ndFloat32(p.X), ndFloat32(p.Y), ndFloat32(p.Z), ndFloat32(0.0f));
				const ndVector hashPoint(bakedScale * q);
				hash = ndCRC64(&hashPoint, sizeof(hashPoint), hash);
			}
		}
		#else	
		//using actual render LOD0 geometry, maybe not a good idea.
		const FStaticMeshRenderData* const renderData = staticMesh->GetRenderData();
		const FStaticMeshLODResourcesArray& renderResource = renderData->LODResources;

		const FVector uScale(GetComponentTransform().GetScale3D());
		const ndVector scale(ndFloat32(uScale.X), ndFloat32(uScale.Y), ndFloat32(uScale.Z), ndFloat32(0.0f));
		const ndVector bakedScale(scale.Scale(UNREAL_INV_UNIT_SYSTEM));

		const FStaticMeshLODResources& renderLOD = renderResource[0];
		const FStaticMeshVertexBuffers& staticMeshVertexBuffer = renderLOD.VertexBuffers;;
		const FPositionVertexBuffer& positBuffer = staticMeshVertexBuffer.PositionVertexBuffer;
		for (int i = positBuffer.GetNumVertices() - 1; i >= 0; --i)
		{
			const FVector3f p(positBuffer.VertexPosition(i));
			const ndVector q(ndFloat32(p.X), ndFloat32(p.Y), ndFloat32(p.Z), ndFloat32(0.0f));
			const ndVector hashPoint(bakedScale * q);

			hash = ndCRC64(&hashPoint, sizeof(hashPoint), hash);
		}
		#endif
	}

	return hash;
}

ndShape* UNewtonCollisionPolygonalMesh::CreateShape() const
{
	auto FindStaticMesh = [this]()
	{
		UStaticMeshComponent* const mesh = Cast<UStaticMeshComponent>(GetAttachParent());
		if (mesh && mesh->GetStaticMesh().Get())
		{
			return mesh->GetStaticMesh().Get();
		}
		return (UStaticMesh*) nullptr;
	};

	UStaticMesh* const staticMesh = FindStaticMesh();
	if (staticMesh)
	{
		ndPolygonSoupBuilder meshBuilder;
		meshBuilder.Begin();
		ndVector face[8];

		const FVector uScale(GetComponentTransform().GetScale3D());
		const ndVector scale(ndFloat32(uScale.X), ndFloat32(uScale.Y), ndFloat32(uScale.Z), ndFloat32(0.0f));
		const ndVector bakedScale(scale.Scale(UNREAL_INV_UNIT_SYSTEM));

		#if 1
		FTriMeshCollisionData collisionData;
		bool data = staticMesh->GetPhysicsTriMeshData(&collisionData, true);
		if (data)
		{
			for (int i = collisionData.Indices.Num() - 1; i >= 0; --i)
			{
				ndInt32 i0 = collisionData.Indices[i].v0;
				ndInt32 i1 = collisionData.Indices[i].v1;
				ndInt32 i2 = collisionData.Indices[i].v2;
				const FVector3f p0(collisionData.Vertices[i0]);
				const FVector3f p1(collisionData.Vertices[i1]);
				const FVector3f p2(collisionData.Vertices[i2]);
				const ndVector q0(ndFloat32(p0.X), ndFloat32(p0.Y), ndFloat32(p0.Z), ndFloat32(0.0f));
				const ndVector q1(ndFloat32(p1.X), ndFloat32(p1.Y), ndFloat32(p1.Z), ndFloat32(0.0f));
				const ndVector q2(ndFloat32(p2.X), ndFloat32(p2.Y), ndFloat32(p2.Z), ndFloat32(0.0f));

				face[0] = q0 * bakedScale;
				face[2] = q1 * bakedScale;
				face[1] = q2 * bakedScale;

				//for now MaterialIndex = 0
				ndInt32 materialIndex = 0;
				meshBuilder.AddFace(&face[0].m_x, sizeof(ndVector), 3, materialIndex);
			}
		}

		#else	
		const FStaticMeshRenderData* const renderData = staticMesh->GetRenderData();
		const FStaticMeshLODResourcesArray& renderResource = renderData->LODResources;

		const FStaticMeshLODResources& renderLOD = renderResource[0];
		const FRawStaticIndexBuffer& staticMeshIndexBuffer = renderLOD.IndexBuffer;
		const FStaticMeshVertexBuffers& staticMeshVertexBuffer = renderLOD.VertexBuffers;;
		const FPositionVertexBuffer& positBuffer = staticMeshVertexBuffer.PositionVertexBuffer;

		int32 indexCount = staticMeshIndexBuffer.GetNumIndices();
		for (ndInt32 i = 0; i < indexCount; i += 3)
		{
			ndInt32 i0 = staticMeshIndexBuffer.GetIndex(i + 0);
			ndInt32 i1 = staticMeshIndexBuffer.GetIndex(i + 1);
			ndInt32 i2 = staticMeshIndexBuffer.GetIndex(i + 2);
			const FVector3f p0(positBuffer.VertexPosition(i0));
			const FVector3f p1(positBuffer.VertexPosition(i1));
			const FVector3f p2(positBuffer.VertexPosition(i2));
			const ndVector q0(ndFloat32(p0.X), ndFloat32(p0.Y), ndFloat32(p0.Z), ndFloat32(0.0f));
			const ndVector q1(ndFloat32(p1.X), ndFloat32(p1.Y), ndFloat32(p1.Z), ndFloat32(0.0f));
			const ndVector q2(ndFloat32(p2.X), ndFloat32(p2.Y), ndFloat32(p2.Z), ndFloat32(0.0f));

			face[0] = q0 * bakedScale;
			face[1] = q1 * bakedScale;
			face[2] = q2 * bakedScale;

			//for now MaterialIndex = 0
			ndInt32 materialIndex = 0;
			meshBuilder.AddFace(&face[0].m_x, sizeof(ndVector), 3, materialIndex);
		}
		#endif

		meshBuilder.End(true);
		ndShape* const shape = new ndShapeStatic_bvh(meshBuilder);
		return shape;
	}
	return new ndShapeNull();
}

void UNewtonCollisionPolygonalMesh::ApplyPropertyChanges()
{
	BuildNewtonShape();

	Super::ApplyPropertyChanges();
}

ndShapeInstance* UNewtonCollisionPolygonalMesh::CreateInstanceShape() const
{
	ndShapeInstance* const instance = new ndShapeInstance(m_shape);
	const FVector uScale(GetComponentTransform().GetScale3D());
	const ndVector scale(ndFloat32(1.0f / uScale.X), ndFloat32(1.0f / uScale.Y), ndFloat32(1.0f / uScale.Z), ndFloat32(0.0f));
	instance->SetScale(scale);
	return instance;
}

ndShapeInstance* UNewtonCollisionPolygonalMesh::CreateBodyInstanceShape(const ndMatrix& bodyMatrix) const
{
	ndShapeInstance* const instance = CreateInstanceShape();

	const ndVector scale(ndFloat32(1.0f));
	const FTransform transform(GetComponentToWorld());
	const ndMatrix matrix(UNewtonRigidBody::ToNewtonMatrix(transform) * bodyMatrix.OrthoInverse());

	instance->SetScale(scale);
	instance->SetLocalMatrix(matrix);
	return instance;
}
