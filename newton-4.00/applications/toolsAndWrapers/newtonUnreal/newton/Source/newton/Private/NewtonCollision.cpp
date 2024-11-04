// Fill out your copyright notice in the Description page of Project Settings.


#include "NewtonCollision.h"
#include "Kismet/GameplayStatics.h"

#include "Newton.h"
#include "NewtonRigidBody.h"
#include "NewtonWorldActor.h"
#include "ThirdParty/newtonLibrary/Public/dNewton/ndNewton.h"

class UNewtonCollision::PolygonizeMesh : public ndShapeDebugNotify
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

		ndArray<ndInt32> triangles;
		ndTriangulatePolygon(&m_points[baseIndex], vertexCount, triangles);
		check((vertexCount - 2) == ndInt32 (triangles.GetCount() / 3));

		for (ndInt32 i = 0; i < ndInt32(triangles.GetCount()); i += 3)
		{
			m_index.PushBack(baseIndex + triangles[i + 0]);
			m_index.PushBack(baseIndex + triangles[i + 2]);
			m_index.PushBack(baseIndex + triangles[i + 1]);
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

// Sets default values for this component's properties
UNewtonCollision::UNewtonCollision()
	:Super()
{
	m_hash = 0;
	m_shape = nullptr;
	m_showDebug = false;
	m_propertyChanged = true;
	m_debugVisualIsDirty = false;

	BestFit = false;
	CastShadow = 0;
	bExplicitShowWireframe = true;
	m_visualMesh = TSharedPtr<UE::Geometry::FDynamicMesh3>(nullptr);

	ConstructorHelpers::FObjectFinder<UMaterial> TexObj(TEXT("/newton/NewtonTransparentMaterial"));
	m_debugMaterial = Cast<UMaterial>(TexObj.Object);
	m_debugMaterial->OpacityMaskClipValue = 0.0f;

	//UMaterialInstanceDynamic* const debugMaterialInstance = UMaterialInstanceDynamic::Create(m_debugMaterial, nullptr);
	//debugMaterialInstance->OpacityMaskClipValue = 0.0f;
	//SetMaterial(0, debugMaterialInstance);
	SetMaterial(0, m_debugMaterial);
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

void UNewtonCollision::SetTransform(const USceneComponent* const meshComponent)
{
	FTransform bodyTransform(GetComponentTransform());
	for (USceneComponent* parent = GetAttachParent(); parent; parent = parent->GetAttachParent())
	{
		bodyTransform = parent->GetComponentTransform();
		if (Cast<UNewtonRigidBody>(parent))
		{
			break;
		}
	}
	const FTransform globalTransform(meshComponent->GetComponentToWorld());
	const FTransform localTransform(globalTransform * bodyTransform.Inverse());

	SetComponentToWorld(globalTransform);

	// for some reason, this does not work in the unreal editor
	SetRelativeScale3D_Direct(localTransform.GetScale3D());
	SetRelativeRotation_Direct(FRotator(localTransform.GetRotation()));
	SetRelativeLocation_Direct(localTransform.GetLocation());
}

void UNewtonCollision::InitStaticMeshCompoment(const USceneComponent* const meshComponent)
{
	check(0);
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
	// implicit shape does use unreal scale, for rendering
	ndShapeInstance* const instance = new ndShapeInstance(m_shape);
	return instance;
}

ndShapeInstance* UNewtonCollision::CreateBodyInstanceShape(const ndMatrix& bodyMatrix) const
{
	ndShapeInstance* const instance = CreateInstanceShape();

	// implicit set the scale and local transform to the instance
	const FVector uScale(GetComponentTransform().GetScale3D());
	const ndVector scale(ndFloat32(uScale.X), ndFloat32(uScale.Y), ndFloat32(uScale.Z), ndFloat32(1.0f));

	const FTransform transform(GetComponentToWorld());
	const ndMatrix matrix(ToNewtonMatrix(transform) * bodyMatrix.OrthoInverse());

	instance->SetScale(scale);
	instance->SetLocalMatrix(matrix);
	return instance;
}

ndVector UNewtonCollision::GetVolumePosition(const ndMatrix& bodyMatrix) const
{
	ndVector posit(0.0f);
	ndShapeInstance* const instance = CreateBodyInstanceShape(bodyMatrix);
	if (instance)
	{
		const ndMatrix inertia(instance->CalculateInertia());
		posit = inertia.m_posit;
		
		posit.m_w = instance->GetVolume();
		delete instance;
	}
	return posit;
}

bool UNewtonCollision::GetDebugMode() const
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
}

void UNewtonCollision::ApplyPropertyChanges()
{
	m_propertyChanged = false;

	if (m_shape)
	{
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
					ndShapeInstance* const instance = CreateInstanceShape();
					instance->DebugShape(ndGetIdentityMatrix(), wireMesh);
					visualMesh = wireMesh.CreateDynamicMesh3();
					plugin->AddDynamicMesh(visualMesh, m_hash);

					delete instance;
				}

				m_visualMesh = visualMesh;
				UDynamicMesh* const dynMesh = GetDynamicMesh();
				dynMesh->SetMesh(UE::Geometry::FDynamicMesh3(*visualMesh.Get()));
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