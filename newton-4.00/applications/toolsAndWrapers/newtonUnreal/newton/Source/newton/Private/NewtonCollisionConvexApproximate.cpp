// Fill out your copyright notice in the Description page of Project Settings.


#include "NewtonCollisionConvexApproximate.h"

#include "Newton.h"
#include "NewtonRigidBody.h"
#include "NewtonSceneActor.h"
#include "ThirdParty/newtonLibrary/Public/dNewton/ndNewton.h"
#include "ThirdParty/newtonLibrary/Public/thirdParty/ndConvexApproximation.h"

//#define SHOW_VHACD_PROGRESS_BAR

class UNewtonCollisionConvexApproximate::ConvexVhacdGenerator : public ndConvexApproximation
{
	public:
	ConvexVhacdGenerator(ndInt32 maxConvexes, bool quality)
		:ndConvexApproximation(maxConvexes, quality)
		,m_progressBar(nullptr)
		,m_acc(0.0f)
	{
		#ifdef SHOW_VHACD_PROGRESS_BAR  
		// for some reason the progress bar invalidate some UClasses
		// I need to report this some day to unreal.
		// for now just do not report progress.
		m_progressBar = new FScopedSlowTask(100.0f, NSLOCTEXT("Newton", "Newton", "Generation Convex Approximation"));
		m_progressBar->MakeDialog();
		#endif
	}

	~ConvexVhacdGenerator()
	{
		if (m_progressBar)
		{
			delete m_progressBar;
			m_progressBar = nullptr;
		}
	}

	virtual void ShowProgress() override
	{
		if (m_progressBar)
		{
			m_acc += 1.0f;
			if (m_acc < 99.0f)
			{
				m_progressBar->EnterProgressFrame();
			}
		}
	}

	FScopedSlowTask* m_progressBar;
	float m_acc;
};

UNewtonCollisionConvexApproximate::UNewtonCollisionConvexApproximate()
	:Super()
{
	m_meshHash = 0x15263412;
	m_generateFlipFlop = false;
	m_convexHullSet = TSharedPtr<ndConvexHullSet>(nullptr);

	Generate = false;
	MaxConvexes = 16;
	Tolerance = 0.0f;
	NumberOfConvex = 0;
	HighResolution = false;
	MaxVertexPerConvex = 32;
}

void UNewtonCollisionConvexApproximate::SerializeLoadRevision_firstVersion(FArchive& ar)
{
	FnewtonModule* const plugin = FnewtonModule::GetPlugin();
	check(plugin);
	TSharedPtr<ndConvexHullSet> serializedSet(plugin->FindConvexHull(m_hash));

	int numOfConvex = 0;
	ar << numOfConvex;
	if (serializedSet == nullptr)
	{
		if (numOfConvex)
		{
			serializedSet = TSharedPtr<ndConvexHullSet>(new ndConvexHullSet);
			for (int j = 0; j < numOfConvex; ++j)
			{
				ndHullPoints* const newHull = new ndHullPoints;
				ndHullPoints& hull = *newHull;
				ar << hull;
				serializedSet->Push(newHull);
			}
			plugin->AddConvexHull(serializedSet, m_meshHash);
		}
	}
	else if (numOfConvex)
	{
		// only parse this data
		ndConvexHullSet convexHullSet;
		for (int j = 0; j < numOfConvex; ++j)
		{
			ndHullPoints* const newHull = new ndHullPoints;
			ndHullPoints& hull = *newHull;
			ar << hull;
			convexHullSet.Push(newHull);
		}
	}
	m_convexHullSet = serializedSet;
}

void UNewtonCollisionConvexApproximate::Serialize(FArchive& ar)
{
	Super::Serialize(ar);

	ar << m_meshHash;
	if (ar.IsSaving())
	{
		int numOfConvex = m_convexHullSet ? m_convexHullSet->Num() : 0;
		ar << numOfConvex;
		for (int j = 0; j < numOfConvex; ++j)
		{
			ndHullPoints& hull = *(*m_convexHullSet)[j];
			ar << hull;
		}
	}
	else
	{
		// this actually sucks big time in unreal
		ar.UsingCustomVersion(FnewtonModule::m_guiID);
		int version = ar.CustomVer(FnewtonModule::m_guiID);
		check (version <= ndPluginVersion::m_firstVersion)
		switch (version)
		{
			case ndPluginVersion::m_firstVersion:
			default:
				SerializeLoadRevision_firstVersion(ar);
		}
	}
}

const FStaticMeshLODResources* UNewtonCollisionConvexApproximate::GetRenderLOD() const
{
	const UStaticMeshComponent* const staticComponent = Cast<UStaticMeshComponent>(GetAttachParent());
	if (staticComponent)
	{
		const UStaticMesh* const staticMesh = staticComponent->GetStaticMesh().Get();
		if (staticMesh)
		{
			const FStaticMeshRenderData* const renderData = staticMesh->GetRenderData();
			if (renderData)
			{
				return &renderData->LODResources[0];
			}
		}
	}
	return nullptr;
}

long long UNewtonCollisionConvexApproximate::CalculateStaticMeshHash() const
{
	long long hash = ndCRC64(ndShapeCompound::StaticClassName(), strlen(ndShapeConvexHull::StaticClassName()), 0);
	const FStaticMeshLODResources* const renderLOD = GetRenderLOD();
	if (renderLOD)
	{
		const FVector uScale(GetComponentTransform().GetScale3D());
		const ndVector scale(ndFloat32(uScale.X), ndFloat32(uScale.Y), ndFloat32(uScale.Z), ndFloat32(0.0f));
		const ndVector bakedScale(scale.Scale(UNREAL_INV_UNIT_SYSTEM));

		const FStaticMeshVertexBuffers& staticMeshVertexBuffer = renderLOD->VertexBuffers;;
		const FPositionVertexBuffer& positBuffer = staticMeshVertexBuffer.PositionVertexBuffer;
		for (ndInt32 i = 0; i < ndInt32(positBuffer.GetNumVertices()); ++i)
		{
			const FVector3f p(positBuffer.VertexPosition(i));
			FVector3f q;
			q.X = ndReal(p.X * bakedScale.m_x);
			q.Y = ndReal(p.Y * bakedScale.m_y);
			q.Z = ndReal(p.Z * bakedScale.m_z);
			hash = ndCRC64(&q, sizeof(FVector3f), hash);
		}
		const FRawStaticIndexBuffer& indexBuffer = renderLOD->IndexBuffer;
		for (ndInt32 i = 0; i < ndInt32(indexBuffer.GetNumIndices()); i += 3)
		{
			ndInt32 j = indexBuffer.GetIndex(i);
			hash = ndCRC64(&j, sizeof(ndInt32), hash);
		}
		hash = ndCRC64(&HighResolution, sizeof(bool), hash);
		hash = ndCRC64(&MaxVertexPerConvex, sizeof(int), hash);
		hash = ndCRC64(&MaxConvexes, sizeof(int), hash);
		hash = ndCRC64(&Tolerance, sizeof(float), hash);
	}
	return hash;
}

ndConvexHullSet* UNewtonCollisionConvexApproximate::CreateConvexApproximationShapes() const
{
	const UStaticMeshComponent* const staticComponent = Cast<UStaticMeshComponent>(GetAttachParent());
	if (!staticComponent)
	{
		check(0);
		return nullptr;
	}

	ConvexVhacdGenerator* const vhacdHullSet = new ConvexVhacdGenerator(MaxConvexes, HighResolution);
	vhacdHullSet->m_tolerance = 1.0e-3f + Tolerance * 0.1f;
	vhacdHullSet->m_maxPointPerHull = MaxVertexPerConvex;

	const FStaticMeshLODResources* const renderLOD = GetRenderLOD();
	check(renderLOD);

	const FVector uScale(GetComponentTransform().GetScale3D());
	const ndVector scale(ndFloat32(uScale.X), ndFloat32(uScale.Y), ndFloat32(uScale.Z), ndFloat32(0.0f));
	const ndVector bakedScale(scale.Scale(UNREAL_INV_UNIT_SYSTEM));

	const FStaticMeshVertexBuffers& staticMeshVertexBuffer = renderLOD->VertexBuffers;
	const FPositionVertexBuffer& positBuffer = staticMeshVertexBuffer.PositionVertexBuffer;
	
	ndHullInputMesh& inputMesh = vhacdHullSet->m_inputMesh;
	for (ndInt32 i = 0; i < ndInt32(positBuffer.GetNumVertices()); ++i)
	{
		ndHullPoint q;
		const FVector3f p(positBuffer.VertexPosition(i));
	
		q.m_x = ndReal(p.X * bakedScale.m_x);
		q.m_y = ndReal(p.Y * bakedScale.m_y);
		q.m_z = ndReal(p.Z * bakedScale.m_z);
		inputMesh.m_points.PushBack(q);
	}
	
	const FRawStaticIndexBuffer& indexBuffer = renderLOD->IndexBuffer;
	for (ndInt32 i = 0; i < ndInt32(indexBuffer.GetNumIndices()); i += 3)
	{
		ndHullInputMesh::ndFace face;
		face.m_i0 = indexBuffer.GetIndex(i + 0);
		face.m_i1 = indexBuffer.GetIndex(i + 1);
		face.m_i2 = indexBuffer.GetIndex(i + 2);
		check(face.m_i0 != face.m_i1);
		check(face.m_i0 != face.m_i2);
		check(face.m_i1 != face.m_i2);
		inputMesh.m_faces.PushBack(face);
	}
	
	vhacdHullSet->Execute();

	ndConvexHullSet* const hullSet = new ndConvexHullSet;
	ndArray<ndHullOutput*>& hullArray = vhacdHullSet->m_ouputHulls;
	for (ndInt32 i = hullArray.GetCount() - 1; i >= 0; --i)
	//for (ndInt32 i = 4; i < 6; ++i)
	{
		ndHullPoints* const pointsSet = new ndHullPoints;
		const ndHullOutput* const convexHull = hullArray[i];
		
		ndHullPoints& dstPoints = *pointsSet;
		const ndArray<ndHullPoint>& srcPoints = *convexHull;
		for (ndInt32 j = ndInt32(srcPoints.GetCount()) - 1; j >= 0; --j)
		{
			FVector3f p(srcPoints[j].m_x, srcPoints[j].m_y, srcPoints[j].m_z);
			dstPoints.Push(p);
		}
		hullSet->Push(pointsSet);
	}

	UE_LOG(LogTemp, Display, TEXT("number of vhcd convex generated: %d"), hullArray.GetCount());
	delete vhacdHullSet;
	return hullSet;
}

long long UNewtonCollisionConvexApproximate::CalculateHash() const
{
	long long hash = ndCRC64(ndShapeCompound::StaticClassName(), strlen(ndShapeConvexHull::StaticClassName()), 0);
	if (m_convexHullSet && m_convexHullSet->Num())
	{
		for (ndInt32 i = 0; i < m_convexHullSet->Num(); ++i)
		{
			const ndHullPoints& hull = *(*m_convexHullSet)[i];
			const FVector3f* const vexterBuffer = &hull[0];
			hash = ndCRC64(vexterBuffer, hull.Num() * sizeof(FVector3f), hash);
		}
	}
	return hash;
}

ndShape* UNewtonCollisionConvexApproximate::CreateShape() const
{
	if (m_convexHullSet && m_convexHullSet->Num())
	{
		//const FVector uScale(GetComponentTransform().GetScale3D());
		//const ndVector scale(ndFloat32(1.0f / uScale.X), ndFloat32(1.0f / uScale.Y), ndFloat32(1.0f / uScale.Z), ndFloat32(0.0f));

		ndShapeCompound* const compound = new ndShapeCompound();
		compound->BeginAddRemove();
		for (ndInt32 i = m_convexHullSet->Num() - 1; i >= 0; --i)
		{
			const ndHullPoints& hullPoints = *((*m_convexHullSet)[i]);
			const FVector3f* const vexterBuffer = &hullPoints[0];
			ndShape* const shape = new ndShapeConvexHull(hullPoints.Num(), sizeof(FVector3f), Tolerance, &vexterBuffer[0].X);
			ndShapeInstance* const subShape = new ndShapeInstance(shape);
			//new ndShapeConvexHull(hullPoints, sizeof(ndVector), Tolerance, &points[0].m_x, MaxVertexCount);
			//subShape->SetScale(scale);
			compound->AddCollision(subShape);
			delete subShape;
		}
		compound->EndAddRemove();
		return compound;
	}
	return new ndShapeNull();
}

void UNewtonCollisionConvexApproximate::InitStaticMeshCompoment(const USceneComponent* const meshComponent)
{
	check(0);
}

ndShapeInstance* UNewtonCollisionConvexApproximate::CreateBodyInstanceShape(const ndMatrix& bodyMatrix) const
{
	ndShapeInstance* const instance = CreateInstanceShape();

	const ndVector scale(ndFloat32(1.0f));
	const FTransform transform(GetComponentToWorld());
	const ndMatrix matrix(ToNewtonMatrix(transform) * bodyMatrix.OrthoInverse());

	instance->SetScale(scale);
	instance->SetLocalMatrix(matrix);
	return instance;
}

ndShapeInstance* UNewtonCollisionConvexApproximate::CreateInstanceShape() const
{
	ndShapeInstance* const instance = new ndShapeInstance(m_shape);
	const FVector uScale(GetComponentTransform().GetScale3D());
	const ndVector scale(ndFloat32(1.0f / uScale.X), ndFloat32(1.0f / uScale.Y), ndFloat32(1.0f / uScale.Z), ndFloat32(0.0f));
	instance->SetScale(scale);
	return instance;
}

ndVector UNewtonCollisionConvexApproximate::GetVolumePosition(const ndMatrix& bodyMatrix) const
{
	ndVector posit(0.0f);
	ndShapeInstance* const instance = CreateInstanceShape();
	if (instance)
	{
		instance->SetScale(ndVector(1.0f));

		const ndMatrix inertia(instance->CalculateInertia());
		posit = inertia.m_posit;

		posit.m_w = instance->GetVolume();
		delete instance;
	}
	return posit;
}

void UNewtonCollisionConvexApproximate::ApplyPropertyChanges()
{
	FnewtonModule* const plugin = FnewtonModule::GetPlugin();
	check(plugin);
	long long meshHash = CalculateStaticMeshHash();
	if ((m_meshHash != meshHash) || Generate && !m_generateFlipFlop)
	{
		TSharedPtr<ndConvexHullSet> convexHullSet(plugin->FindConvexHull(meshHash));
		if (convexHullSet == nullptr)
		{
			convexHullSet = TSharedPtr<ndConvexHullSet>(CreateConvexApproximationShapes());
			plugin->AddConvexHull(convexHullSet, meshHash);
		}
	}
	if (m_convexHullSet == nullptr)
	{
		m_convexHullSet = plugin->FindConvexHull(meshHash);
		m_debugVisualIsDirty = true;
	}
	NumberOfConvex = m_convexHullSet ? m_convexHullSet->Num() : 0;
	MarkRenderDynamicDataDirty();
	NotifyMeshUpdated();

	m_meshHash = meshHash;
	m_generateFlipFlop = Generate;
	Generate = false;
	BuildNewtonShape();
	Super::ApplyPropertyChanges();
}