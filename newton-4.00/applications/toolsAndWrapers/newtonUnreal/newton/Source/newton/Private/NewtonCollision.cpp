// Fill out your copyright notice in the Description page of Project Settings.


#include "NewtonCollision.h"
#include "Kismet/GameplayStatics.h"

#include "Newton.h"
#include "NewtonRigidBody.h"
#include "NewtonWorldActor.h"
#include "NewtonSceneActor.h"
#include "ThirdParty/newtonLibrary/Public/dNewton/ndNewton.h"

// Sets default values for this component's properties
UNewtonCollision::UNewtonCollision()
	:Super()
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	//PrimaryComponentTick.bCanEverTick = true;

	m_hash = 0;
	m_shape = nullptr;
	m_showDebug = false;
	m_propertyChanged = true;
	m_debugVisualIsDirty = false;

	BestFit = false;
	CastShadow = 0;
	bExplicitShowWireframe = true;
	m_geometryMesh = TObjectPtr<USceneComponent>(nullptr);
	m_visualMesh = TSharedPtr<UE::Geometry::FDynamicMesh3>(nullptr);

	ConstructorHelpers::FObjectFinder<UMaterial> TexObj(TEXT("/newton/NewtonTransparentMaterial"));
	m_transparentMaterial = Cast<UMaterial>(TexObj.Object);
	m_transparentMaterial->OpacityMaskClipValue = 0.0f;
}

void UNewtonCollision::OnRegister()
{
	Super::OnRegister();
	m_propertyChanged = true;
}

void UNewtonCollision::OnUnregister()
{
	Super::OnUnregister();
	if (m_shape)
	{
		m_shape->Release();
		m_shape = nullptr;
	}
	m_hash = 0;
	m_visualMesh = TSharedPtr<UE::Geometry::FDynamicMesh3>(nullptr);
}

//USceneComponent* UNewtonCollision::GetGeometryMesh() const
TObjectPtr<USceneComponent> UNewtonCollision::GetGeometryMesh() const
{
	return m_geometryMesh;
}

//void UNewtonCollision::SetGeometryMesh(USceneComponent* const geometry)
void UNewtonCollision::SetGeometryMesh(const TObjectPtr<USceneComponent>& geometry)
{
	m_geometryMesh = geometry;
}

void UNewtonCollision::Serialize(FArchive& ar)
{
	Super::Serialize(ar);
	ar << m_geometryMesh;
}

void UNewtonCollision::PostLoad()
{
	Super::PostLoad();

	m_propertyChanged = true;

	UDynamicMesh* mesh = GetDynamicMesh();
	check(mesh);
	UE::Geometry::FDynamicMesh3* const triangleMesh = mesh->GetMeshPtr();
	triangleMesh->Clear();
}

void UNewtonCollision::OnAttachmentChanged()
{
	Super::OnAttachmentChanged();
	m_propertyChanged = true;
}

void UNewtonCollision::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);
	m_propertyChanged = true;
	if (BestFit)
	{
		BestFit = false;
	}
}

void UNewtonCollision::BeginDestroy()
{
	Super::BeginDestroy();
	if (m_shape)
	{
		m_shape->Release();
		m_shape = nullptr;
	}
	m_hash = 0;
	m_visualMesh = TSharedPtr<UE::Geometry::FDynamicMesh3>(nullptr);
}

bool UNewtonCollision::ShouldCreatePhysicsState() const
{
	Super::ShouldCreatePhysicsState();
	return false;
}

long long UNewtonCollision::CalculateHash() const
{
	check(0);
	return 0;
}

ndShape* UNewtonCollision::CreateShape() const
{
	check(0);
	return new ndShapeNull();
}

void UNewtonCollision::BuildNewtonShape()
{
	const ANewtonSceneActor* const owner = Cast<ANewtonSceneActor>(GetOwner());
	if (owner)
	{
		const FTransform bodyTransform(owner->GetRootComponent()->GetComponentToWorld());
		const FTransform meshTransform(m_geometryMesh->GetComponentToWorld());
		const FTransform transform(meshTransform * bodyTransform.Inverse());
		SetComponentToWorld(transform);
	}

	long long hash = CalculateHash();
	if (m_hash != hash)
	{
		m_hash = hash;
		m_debugVisualIsDirty = true;
		if (m_shape)
		{
			m_shape->Release();
		}

		FnewtonModule* const plugin = FnewtonModule::GetPlugin();
		check(plugin);
		ndShape* shape = plugin->FindShape(m_hash);
		if (!shape)
		{
			shape = CreateShape();
			plugin->AddShape(shape, m_hash);
		}
		m_shape = (ndShape*)shape->AddRef();
	}
}

void UNewtonCollision::SetWireFrameColor(const FLinearColor& color)
{
	WireframeColor = color;
	OnRenderingStateChanged(false);
	MarkRenderDynamicDataDirty();
	NotifyMeshUpdated();
}

ndShapeInstance* UNewtonCollision::CreateInstanceShape() const
{
	ndShapeInstance* const instance = new ndShapeInstance(m_shape);
	return instance;
}

ndShapeInstance* UNewtonCollision::CreateBodyInstanceShape(const ndMatrix& bodyMatrix) const
{
	ndShapeInstance* const instance = CreateInstanceShape();

	const FVector uScale(GetComponentTransform().GetScale3D());
	const ndVector scale(ndFloat32(uScale.X), ndFloat32(uScale.Y), ndFloat32(uScale.Z), ndFloat32(1.0f));

	const FTransform transform(GetComponentToWorld());
	const ndMatrix matrix(UNewtonRigidBody::ToNewtonMatrix(transform) * bodyMatrix.OrthoInverse());

	instance->SetScale(scale);
	instance->SetLocalMatrix(matrix);
	return instance;
}

void UNewtonCollision::ApplyPropertyChanges()
{
	class PolygonizeMesh : public ndShapeDebugNotify
	{
		public:
		PolygonizeMesh()
			:ndShapeDebugNotify()
			,m_scale(UNREAL_UNIT_SYSTEM)
		{
		}

		void DrawPolygon(ndInt32 vertexCount, const ndVector* const faceArray, const ndEdgeType* const)
		{
			ndInt32 baseIndex = m_points.GetCount();
			for (ndInt32 i = 0; i < vertexCount; ++i)
			{
				m_points.PushBack(faceArray[i] * m_scale);
			}

			for (ndInt32 i = 2; i < vertexCount; ++i)
			{
				m_index.PushBack(baseIndex);
				m_index.PushBack(baseIndex + i - 0);
				m_index.PushBack(baseIndex + i - 1);
			}
		}

		TSharedPtr<UE::Geometry::FDynamicMesh3> CreateDynamicMesh3()
		{
			ndArray<ndInt32> remapIndex;
			remapIndex.SetCount(m_points.GetCount());
			ndInt32 vertexCount = ndVertexListToIndexList(&m_points[0].m_x, sizeof(ndVector), 3, ndInt32(m_points.GetCount()), &remapIndex[0], 1.0e-5f);

			ndArray<ndInt32> unrealVertexIndex;
			unrealVertexIndex.SetCount(m_points.GetCount());

			TSharedPtr<UE::Geometry::FDynamicMesh3> mesh3(new UE::Geometry::FDynamicMesh3());
			UE::Geometry::FDynamicMesh3* const triangleMesh = mesh3.Get();

			triangleMesh->Clear();
			for (ndInt32 i = 0; i < vertexCount; ++i)
			{
				FVector3d p(m_points[i].m_x, m_points[i].m_y, m_points[i].m_z);
				unrealVertexIndex[i] = triangleMesh->AppendVertex(p);
			}

			for (ndInt32 i = 0; i < m_index.GetCount(); i += 3)
			{
				int k0 = remapIndex[m_index[i + 0]];
				int k1 = remapIndex[m_index[i + 1]];
				int k2 = remapIndex[m_index[i + 2]];

				int j0 = unrealVertexIndex[k0];
				int j1 = unrealVertexIndex[k1];
				int j2 = unrealVertexIndex[k2];
				triangleMesh->AppendTriangle(j0, j1, j2);
			}
			return 	mesh3;
		}

		const ndVector m_scale;
		ndArray<ndInt32> m_index;
		ndArray<ndVector> m_points;
	};

	if (m_geometryMesh.Get())
	{
		m_debugVisualIsDirty = true;
	}

	m_propertyChanged = false;

	if (m_shape)
	{
		auto GetDebugMode = [this]()
		{
			for (USceneComponent* component = GetAttachParent(); component; component = component->GetAttachParent())
			{
				const UNewtonRigidBody* const rigidBody = Cast<UNewtonRigidBody>(component);
				if (rigidBody)
				{
					return rigidBody->ShowDebug;
				}
			}
			return false;
		};

		bool showDebug = GetDebugMode();
		if (showDebug)
		{
			if (!m_showDebug && m_debugVisualIsDirty)
			{
				FnewtonModule* const plugin = FnewtonModule::GetPlugin();
				check(plugin);
				TSharedPtr<UE::Geometry::FDynamicMesh3> visualMesh(plugin->FindDynamicMesh(m_hash));
				if (visualMesh.Get() == nullptr)
				{
					//UE_LOG(LogTemp, Warning, TEXT("Rebuild Mesh"));
					PolygonizeMesh wireMesh;
					ndShapeInstance* const instanceShape = CreateInstanceShape();
					instanceShape->DebugShape(ndGetIdentityMatrix(), wireMesh);
					visualMesh = wireMesh.CreateDynamicMesh3();
					plugin->AddDynamicMesh(visualMesh, m_hash);

					delete instanceShape;
				}

				m_visualMesh = visualMesh;
				UDynamicMesh* const dynMesh = GetDynamicMesh();
				dynMesh->SetMesh(UE::Geometry::FDynamicMesh3(*visualMesh.Get()));

				UMaterialInstanceDynamic* const transparentMaterialInstance = UMaterialInstanceDynamic::Create(m_transparentMaterial, nullptr);
				SetMaterial(0, transparentMaterialInstance);
			}
		}
		else
		{
			// remove the show debug mesh
			UE::Geometry::FDynamicMesh3* const triangleMesh = GetDynamicMesh()->GetMeshPtr();
			triangleMesh->Clear();
		}

		m_showDebug = showDebug;
		m_debugVisualIsDirty = false;
		OnRenderingStateChanged(false);
		MarkRenderDynamicDataDirty();
		NotifyMeshUpdated();
	}
}

ndVector UNewtonCollision::GetVolumePosition() const
{
	ndVector posit(0.0f);
	const ndShapeInstance* const instance = CreateInstanceShape();
	if (instance)
	{
		posit = instance->GetLocalMatrix().m_posit;
		posit.m_w = instance->GetVolume();
		delete instance;
	}
	return posit;
}