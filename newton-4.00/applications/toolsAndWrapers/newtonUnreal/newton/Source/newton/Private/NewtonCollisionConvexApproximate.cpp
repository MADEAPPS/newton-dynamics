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
	Generate = false;
	m_generateFlipFlop = false;
	HighResolution = false;
	MaxConvexes = 16;
	Tolerance = 0.0f;
	MaxVertexPerConvex = 32;
}

void UNewtonCollisionConvexApproximate::Serialize(FArchive& ar)
{
	Super::Serialize(ar);

	// this actually sucks big time in unreal
	//ar.UsingCustomVersion(FnewtonModule::m_guiID);
	//int version = ar.CustomVer(FnewtonModule::m_guiID);
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
		FnewtonModule* const plugin = FnewtonModule::GetPlugin();
		check(plugin);
		long long meshHash = CalculateStaticMeshHash();
		TSharedPtr<ndConvexHullSet> serializedSet(plugin->FindConvexHull(m_hash));
		if (serializedSet == nullptr)
		{
			int numOfConvex = m_convexHullSet ? m_convexHullSet->Num() : 0;
			ar << numOfConvex;
			if (numOfConvex)
			{
				TSharedPtr<ndConvexHullSet> convexHullSet(new ndConvexHullSet);
				for (int j = 0; j < numOfConvex; ++j)
				{
					ndHullPoints* const newHull = new ndHullPoints;
					ndHullPoints& hull = *newHull;
					ar << hull;
					convexHullSet->Push(newHull);
				}
				m_convexHullSet = convexHullSet;
				plugin->AddConvexHull(convexHullSet, meshHash);
			}
			
		}
		else
		{
			int numOfConvex = m_convexHullSet ? m_convexHullSet->Num() : 0;
			ar << numOfConvex;
			if (numOfConvex)
			{
				ndConvexHullSet convexHullSet;
				for (int j = 0; j < numOfConvex; ++j)
				{
					ndHullPoints* const newHull = new ndHullPoints;
					ndHullPoints& hull = *newHull;
					ar << hull;
					convexHullSet.Push(newHull);
				}
			}
		}
	}
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
	
	const UStaticMesh* const staticMesh = staticComponent->GetStaticMesh().Get();
	check(staticMesh);
	const FStaticMeshRenderData* const renderData = staticMesh->GetRenderData();
	check(renderData);
	const FStaticMeshLODResourcesArray& renderResource = renderData->LODResources;
	
	const FVector uScale(GetComponentTransform().GetScale3D());
	const ndVector scale(ndFloat32(uScale.X), ndFloat32(uScale.Y), ndFloat32(uScale.Z), ndFloat32(0.0f));
	const ndVector bakedScale(scale.Scale(UNREAL_INV_UNIT_SYSTEM));
	
	const FStaticMeshLODResources& renderLOD = renderResource[0];
	const FStaticMeshVertexBuffers& staticMeshVertexBuffer = renderLOD.VertexBuffers;;
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
	
	const FRawStaticIndexBuffer& indexBuffer = renderLOD.IndexBuffer;
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

long long UNewtonCollisionConvexApproximate::CalculateStaticMeshHash() const
{
	long long hash = ndCRC64(ndShapeCompound::StaticClassName(), strlen(ndShapeConvexHull::StaticClassName()), 0);
	const UStaticMeshComponent* const staticComponent = Cast<UStaticMeshComponent>(GetAttachParent());
	if (staticComponent)
	{
		ConvexVhacdGenerator* const convexHullSet = new ConvexVhacdGenerator(MaxConvexes, HighResolution);
		convexHullSet->m_tolerance = 1.0e-3f + Tolerance * 0.1f;
		convexHullSet->m_maxPointPerHull = MaxVertexPerConvex;

		const UStaticMesh* const staticMesh = staticComponent->GetStaticMesh().Get();
		check(staticMesh);
		const FStaticMeshRenderData* const renderData = staticMesh->GetRenderData();
		check(renderData);
		const FStaticMeshLODResourcesArray& renderResource = renderData->LODResources;

		//const FVector uScale(GetComponentTransform().GetScale3D());
		//const ndVector scale(ndFloat32(uScale.X), ndFloat32(uScale.Y), ndFloat32(uScale.Z), ndFloat32(0.0f));
		//const ndVector bakedScale(scale.Scale(UNREAL_INV_UNIT_SYSTEM));

		const FStaticMeshLODResources& renderLOD = renderResource[0];
		const FStaticMeshVertexBuffers& staticMeshVertexBuffer = renderLOD.VertexBuffers;;
		const FPositionVertexBuffer& positBuffer = staticMeshVertexBuffer.PositionVertexBuffer;
		for (ndInt32 i = 0; i < ndInt32(positBuffer.GetNumVertices()); ++i)
		{
			const FVector3f p(positBuffer.VertexPosition(i));
			hash = ndCRC64(&p, sizeof(FVector3f), hash);
		}
		const FRawStaticIndexBuffer& indexBuffer = renderLOD.IndexBuffer;
		for (ndInt32 i = 0; i < ndInt32(indexBuffer.GetNumIndices()); i += 3)
		{
			ndInt32 j = indexBuffer.GetIndex(i);
			hash = ndCRC64(&j, sizeof(ndInt32), hash);
		}
	}
	return hash;
}

void UNewtonCollisionConvexApproximate::ApplyPropertyChanges()
{
	FnewtonModule* const plugin = FnewtonModule::GetPlugin();
	check(plugin);
	long long meshHash = CalculateStaticMeshHash();
	if (Generate && !m_generateFlipFlop)
	{
		TSharedPtr<ndConvexHullSet> convexHullSet (plugin->FindConvexHull(m_hash));
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
	MarkRenderDynamicDataDirty();
	NotifyMeshUpdated();

	m_generateFlipFlop = Generate;
	Generate = false;
	BuildNewtonShape();
	Super::ApplyPropertyChanges();
}

ndShapeInstance* UNewtonCollisionConvexApproximate::CreateBodyInstanceShape(const ndMatrix& bodyMatrix) const
{
	ndShapeInstance* const instance = CreateInstanceShape();

	const ndVector scale(ndFloat32(1.0f));
	const FTransform transform(GetComponentToWorld());
	const ndMatrix matrix(UNewtonRigidBody::ToNewtonMatrix(transform) * bodyMatrix.OrthoInverse());

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