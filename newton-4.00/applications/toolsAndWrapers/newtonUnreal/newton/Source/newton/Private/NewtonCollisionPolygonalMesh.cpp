// Fill out your copyright notice in the Description page of Project Settings.


#include "NewtonCollisionPolygonalMesh.h"

#include "Newton.h"
#include "NewtonRigidBody.h"
#include "NewtonSceneActor.h"
#include "ThirdParty/newtonLibrary/Public/dNewton/ndNewton.h"

class UNewtonCollisionPolygonalMesh::ndShapeStatic : public ndShapeStatic_bvh
{
	public:
	ndShapeStatic(const ndPolygonSoupBuilder& meshBuilder, UNewtonCollisionPolygonalMesh* const self)
		:ndShapeStatic_bvh(meshBuilder)
		,m_self(self)
		,m_vertexCount(0)
	{
	}

	static ndIntersectStatus AaabbIntersectCallback(
		void* const context,
		const ndFloat32* const polygon, ndInt32 strideInBytes,
		const ndInt32* const indexArray, ndInt32 indexCount, ndFloat32 hitDistance)
	{
		ndShapeStatic* const self = (ndShapeStatic*)context;
		UNewtonCollisionPolygonalMesh* const polyMesh = self->m_self;

		for (ndInt32 i = indexCount - 1; i >= 0; --i)
		{
			self->m_vertexCount = ndMax(self->m_vertexCount, indexArray[i]);
		}

		for (ndInt32 i = 2; i < indexCount; ++i)
		{
			polyMesh->m_indexData.Push(indexArray[0]);
			polyMesh->m_indexData.Push(indexArray[i - 0]);
			polyMesh->m_indexData.Push(indexArray[i - 1]);
		}
		return m_continueSearh;
	}

	void ScanMesh()
	{
		ndInt32 vCount = GetVertexCount();
		ndInt32 stride = GetStrideInBytes() / sizeof(ndFloat32);
		const ndFloat32* const points = GetLocalVertexPool();

		ndVector p0(-1.0e10f, -1.0e10f, -1.0e10f, 0.0f);
		ndVector p1(1.0e10f, 1.0e10f, 1.0e10f, 0.0f);
		const ndFastAabb aabb(p0, p1);
		m_vertexCount = -1;
		ForAllSectors(aabb, ndVector(0.0f), 1.0f, AaabbIntersectCallback, this);
		m_vertexCount++;
		for (ndInt32 j = 0; j < m_vertexCount; ++j)
		{
			ndInt32 i =  j * stride;
			FVector3f p(points[i + 0], points[i + 1], points[i + 2]);
			m_self->m_vetexData.Push(p);
		}
	}

	UNewtonCollisionPolygonalMesh* m_self;
	int m_vertexCount;
};

UNewtonCollisionPolygonalMesh::UNewtonCollisionPolygonalMesh()
	:Super()
{
}

void UNewtonCollisionPolygonalMesh::Serialize(FArchive& ar)
{
	Super::Serialize(ar);

	ar << m_vetexData;
	ar << m_indexData;
}

long long UNewtonCollisionPolygonalMesh::CalculateHash() const
{
	long long hash = ndCRC64(ndShapeHeightfield::StaticClassName(), strlen(ndShapeHeightfield::StaticClassName()), 0);
	if (m_indexData.Num())
	{
		const int* const indexBuffer = &m_indexData[0];
		const FVector3f* const vexterBuffer = &m_vetexData[0];
		hash = ndCRC64(indexBuffer, m_indexData.Num() * sizeof(int), hash);
		hash = ndCRC64(vexterBuffer, m_vetexData.Num() * sizeof(FVector3f), hash);
	}
	return hash;
}

ndShape* UNewtonCollisionPolygonalMesh::CreateShape() const
{
	if (!m_indexData.Num())
	{
		return new ndShapeNull();
	}
	ndPolygonSoupBuilder meshBuilder;
	meshBuilder.Begin();
	ndVector face[8];
	for (ndInt32 i =  m_indexData.Num() - 3; i >= 0; i -= 3)
	{
		ndInt32 i0 = m_indexData[i + 0];
		ndInt32 i1 = m_indexData[i + 1];
		ndInt32 i2 = m_indexData[i + 2];
		const FVector3f p0(m_vetexData[i0]);
		const FVector3f p1(m_vetexData[i1]);
		const FVector3f p2(m_vetexData[i2]);
		const ndVector q0(ndFloat32(p0.X), ndFloat32(p0.Y), ndFloat32(p0.Z), ndFloat32(0.0f));
		const ndVector q1(ndFloat32(p1.X), ndFloat32(p1.Y), ndFloat32(p1.Z), ndFloat32(0.0f));
		const ndVector q2(ndFloat32(p2.X), ndFloat32(p2.Y), ndFloat32(p2.Z), ndFloat32(0.0f));
		face[0] = q0;
		face[1] = q1;
		face[2] = q2;
		//for now MaterialIndex = 0
		ndInt32 materialIndex = 0;
		meshBuilder.AddFace(&face[0].m_x, sizeof(ndVector), 3, materialIndex);
	}
	meshBuilder.End(true);
	ndShape* const shape = new ndShapeStatic_bvh(meshBuilder);
	return shape;
}

void UNewtonCollisionPolygonalMesh::InitStaticMeshCompoment(const USceneComponent* const meshComponent)
{
	SetTransform(meshComponent);

	const UStaticMeshComponent* const staticMeshComponent = Cast<UStaticMeshComponent>(meshComponent);
	check(staticMeshComponent);
	UStaticMesh* const staticMesh = staticMeshComponent->GetStaticMesh().Get();

	ndVector face[8];
	
	ndPolygonSoupBuilder meshBuilder;
	meshBuilder.Begin();
	
	const FVector uScale(GetComponentTransform().GetScale3D());
	const ndVector scale(ndFloat32(uScale.X), ndFloat32(uScale.Y), ndFloat32(uScale.Z), ndFloat32(0.0f));
	const ndVector bakedScale(scale.Scale(UNREAL_INV_UNIT_SYSTEM));
	
	FTriMeshCollisionData collisionData;
	bool data = staticMesh->GetPhysicsTriMeshData(&collisionData, true);
	if (data)
	{
		for (ndInt32 i =  collisionData.Indices.Num() - 1; i >= 0; --i)
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
			face[1] = q1 * bakedScale;
			face[2] = q2 * bakedScale;
	
			//for now MaterialIndex = 0
			ndInt32 materialIndex = 0;
			meshBuilder.AddFace(&face[0].m_x, sizeof(ndVector), 3, materialIndex);
		}
	}
	meshBuilder.End(true);
	ndShapeStatic* const polyTree= new ndShapeStatic(meshBuilder, this);
	polyTree->ScanMesh();
	delete polyTree;
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
	const ndMatrix matrix(ToNewtonMatrix(transform) * bodyMatrix.OrthoInverse());

	instance->SetScale(scale);
	instance->SetLocalMatrix(matrix);
	return instance;
}
