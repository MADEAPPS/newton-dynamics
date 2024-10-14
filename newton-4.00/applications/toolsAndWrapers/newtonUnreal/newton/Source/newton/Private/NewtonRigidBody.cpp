// Fill out your copyright notice in the Description page of Project Settings.


#include "NewtonRigidBody.h"
#include "Selection.h"
#include "LevelEditor.h"
#include "ContentBrowserModule.h"
#include "Modules/ModuleManager.h"
#include "Kismet/GameplayStatics.h"
#include "PhysicsEngine/BodySetup.h"

#include "Newton.h"
#include "NewtonJoint.h"
#include "NewtonCollision.h"
#include "NewtonWorldActor.h"
#include "ThirdParty/newtonLibrary/Public/dNewton/ndNewton.h"
#include "ThirdParty/newtonLibrary/Public/thirdParty/ndConvexApproximation.h"

//FLinearColor UNewtonRigidBody::m_awakeColor(1.0f, 0.0f, 0.f);
//FLinearColor UNewtonRigidBody::m_sleepingColor(0.0f, 1.0f, 0.f);

FLinearColor UNewtonRigidBody::m_awakeColor(0.0f, 0.5f, 1.0f);
FLinearColor UNewtonRigidBody::m_sleepingColor(0.0f, 0.125f, 0.25f);

class UNewtonRigidBody::NotifyCallback : public ndBodyNotify
{
	public:
	NotifyCallback(UNewtonRigidBody* const owner, const ndVector& gravity)
		:ndBodyNotify(gravity)
		,m_owner(owner)
		,m_sleepState(false)
	{
		const FTransform transform(owner->GetComponentTransform());
		const FVector location(transform.GetLocation());
		m_posit0.m_x = ndFloat32(location.X * UNREAL_INV_UNIT_SYSTEM);
		m_posit0.m_y = ndFloat32(location.Y * UNREAL_INV_UNIT_SYSTEM);
		m_posit0.m_z = ndFloat32(location.Z * UNREAL_INV_UNIT_SYSTEM);
		m_posit0.m_w = ndFloat32(1.0f);
		m_posit1 = m_posit0;

		const FQuat rot (transform.Rotator().Quaternion());
		m_rotation0.m_x = ndFloat32(rot.X);
		m_rotation0.m_y = ndFloat32(rot.Y);
		m_rotation0.m_z = ndFloat32(rot.Z);
		m_rotation0.m_w = ndFloat32(rot.W);
	}

	NotifyCallback(const NotifyCallback& src)
		:ndBodyNotify(src)
		,m_posit0(src.m_posit0)
		,m_posit1(src.m_posit1)
		,m_rotation0(src.m_rotation0)
		,m_rotation1(src.m_rotation1)
		,m_owner(src.m_owner)
	{
	}

	~NotifyCallback()
	{
	}

	ndBodyNotify* Clone() const
	{
		return new NotifyCallback(*this);
	}

	virtual bool OnSceneAabbOverlap(const ndBody* const otherBody) const
	{
		return true;
	}

	virtual void* GetUserData() const
	{
		return m_owner;
	}

	void UpdateTransform()
	{
		m_posit0 = m_posit1;
		m_rotation0 = m_rotation1;
		m_posit1 = GetBody()->GetMatrix().m_posit;
		m_rotation1 = GetBody()->GetRotation();

		if (m_rotation0.DotProduct(m_rotation1).GetScalar() < 0.0f)
		{
			m_rotation0 = m_rotation0.Scale(-1.0f);
		}
	}

	FTransform InteptolateTransform(ndFloat32 param)
	{
		const ndVector posit(m_posit0 + (m_posit1 - m_posit0).Scale(param));
		const ndQuaternion rotation(m_rotation0.Slerp(m_rotation1, param));
		const FQuat uRot(rotation.m_x, rotation.m_y, rotation.m_z, rotation.m_w);
		const FVector uPosit(posit.m_x * UNREAL_UNIT_SYSTEM, posit.m_y * UNREAL_UNIT_SYSTEM, posit.m_z * UNREAL_UNIT_SYSTEM);

		FTransform transform;
		transform.SetRotation(uRot);
		transform.SetLocation(uPosit);
		return transform;
	}

	virtual void OnApplyExternalForce(ndInt32 threadIndex, ndFloat32 timestep)
	{
		ndBodyDynamic* const body = GetBody()->GetAsBodyDynamic();
		const ndVector force(GetGravity().Scale(body->GetMassMatrix().m_w));
		body->SetForce(force);
	}

	ndVector m_posit0;
	ndVector m_posit1;
	ndQuaternion m_rotation0;
	ndQuaternion m_rotation1;
	UNewtonRigidBody* m_owner;
	bool m_sleepState;
};

// Sets default values for this component's properties
UNewtonRigidBody::UNewtonRigidBody()
	:Super()
	,ShowDebug(false)
	,ShowCenterOfMass(false)
	,AutoSleepMode(true)
	,DebugScale(1.0f)
	,Mass(0.0f)
	,LinearDamp(0.0f)
	,AngularDamp(0.0f)
	,InitialVeloc(0.0f, 0.0f, 0.0f)
	,InitialOmega(0.0f, 0.0f, 0.0f)
	,CenterOfMass(0.0f, 0.0f, 0.0f)
	,Gravity(0.0f, 0.0f, -980.0f)
	,m_localScale(1.0f, 1.0f, 1.0f)
	,m_globalScale(1.0f, 1.0f, 1.0f)
	,m_localTransform()
	,m_globalTransform()
	,m_body(nullptr)
	,m_newtonWorld(nullptr)
	,m_sleeping(true)
	,m_propertyChanged(true)
{
	// Set this component to be initialized when the game starts, and to be ticked every frame.  You can turn these features
	// off to improve performance if you don't need them.
	PrimaryComponentTick.bCanEverTick = true;
	SetMobility(EComponentMobility::Movable);
}

FTransform UNewtonRigidBody::ToUnRealTransform(const ndMatrix& matrix)
{
	const ndQuaternion rotation(matrix);
	const ndVector posit(matrix.m_posit.Scale(UNREAL_UNIT_SYSTEM));
	const FVector uPosit(posit.m_x, posit.m_y, posit.m_z);
	const FQuat uRot(rotation.m_x, rotation.m_y, rotation.m_z, rotation.m_w);

	FTransform transform;
	transform.SetRotation(uRot);
	transform.SetLocation(uPosit);
	return transform;
}

ndMatrix UNewtonRigidBody::ToNewtonMatrix(const FTransform& tranform)
{
	const FVector location(tranform.GetLocation());
	const FQuat rotation(tranform.Rotator().Quaternion());

	const ndQuaternion quat(ndFloat32(rotation.X), ndFloat32(rotation.Y), ndFloat32(rotation.Z), ndFloat32(rotation.W));
	const ndVector posit(UNREAL_INV_UNIT_SYSTEM * ndFloat32(location.X), UNREAL_INV_UNIT_SYSTEM * ndFloat32(location.Y), UNREAL_INV_UNIT_SYSTEM * ndFloat32(location.Z), ndFloat32(1.0f));
	const ndMatrix matrix(ndCalculateMatrix(quat, posit));
	return matrix;
}

void UNewtonRigidBody::BeginDestroy()
{
	Super::BeginDestroy();
}

void UNewtonRigidBody::DestroyRigidBody()
{
	if (m_body)
	{
		ndWorld* const world = m_body->GetScene()->GetWorld();
		if (world)
		{
			world->RemoveBody(m_body);
		}
		m_body = nullptr;
	}
}

void UNewtonRigidBody::PostLoad()
{
	Super::PostLoad();
	m_propertyChanged = true;
}

void UNewtonRigidBody::OnRegister()
{
	Super::OnRegister();
	m_propertyChanged = true;
}

void UNewtonRigidBody::OnUnregister()
{
	Super::OnUnregister();
}

void UNewtonRigidBody::OnChildAttached(USceneComponent* component)
{
	Super::OnChildAttached(component);
	m_propertyChanged = true;
}

void UNewtonRigidBody::OnChildDetached(USceneComponent* component)
{
	Super::OnChildDetached(component);
	m_propertyChanged = true;
}

void UNewtonRigidBody::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);
	m_propertyChanged = true;
}

void UNewtonRigidBody::ClearDebug()
{
	ShowDebug = false;
	ShowCenterOfMass = false;
	m_propertyChanged = true;
	Inertia.ShowPrincipalAxis = false;
}

ndMatrix UNewtonRigidBody::CalculateInertiaMatrix() const
{
	ndMatrix inertia(ndGetZeroMatrix());
	ndArray<ndShapeInstance*> instances;
	ndFixSizeArray<const USceneComponent*, 1024> stack;
	stack.PushBack(this);

	bool isDynamics = true;
	FVector invScale(GetComponentToWorld().GetScale3D());
	const ndMatrix bodyMatrix(ToNewtonMatrix(GetComponentToWorld()).OrthoInverse());
	invScale.X = 1.0f / invScale.X;
	invScale.Y = 1.0f / invScale.Y;
	invScale.Z = 1.0f / invScale.Z;
	while (stack.GetCount())
	{
		const USceneComponent* const component = stack.Pop();
		const UNewtonCollision* const collisionComponent = Cast<UNewtonCollision>(component);
		if (collisionComponent)
		{
			check(collisionComponent->m_shape);
			isDynamics = isDynamics || (!collisionComponent->m_shape->GetAsShapeStaticMesh()) ? true : false;

			const FTransform transform(collisionComponent->GetComponentToWorld());
			const ndMatrix localMatrix(ToNewtonMatrix(transform) * bodyMatrix);
			const FVector localScale(transform.GetScale3D() * invScale);

			ndShapeInstance* const instance = collisionComponent->CreateInstanceShape();
			instance->SetScale(ndVector(ndFloat32 (localScale.X), ndFloat32(localScale.Y), ndFloat32(localScale.Z), ndFloat32 (1.0f)));
			instance->SetLocalMatrix(instance->GetLocalMatrix() * localMatrix);
			instances.PushBack(instance);
		}
		const TArray<TObjectPtr<USceneComponent>>& childrenComp = component->GetAttachChildren();
		for (int i = childrenComp.Num() - 1; i >= 0; --i)
		{
			stack.PushBack(childrenComp[i].Get());
		}
	}
	if (instances.GetCount() == 1)
	{
		if (isDynamics)
		{
			inertia = instances[0]->CalculateInertia();
		}
		delete instances[0];
	}
	else if (instances.GetCount())
	{
		ndShapeInstance compoundInstance(new ndShapeCompound());
		ndShapeCompound* const compound = compoundInstance.GetShape()->GetAsShapeCompound();
		compound->BeginAddRemove();
		for (ndInt32 i = ndInt32(instances.GetCount()) - 1; i >= 0; --i)
		{
			ndShapeInstance* const subShape = instances[i];
			if (isDynamics)
			{
				compound->AddCollision(subShape);
			}
			delete subShape;
		}
		compound->EndAddRemove();
		if (isDynamics)
		{
			inertia = compoundInstance.CalculateInertia();
		}
	}

	return inertia;
}

void UNewtonRigidBody::DrawGizmo(float timestep)
{
	if (Inertia.ShowPrincipalAxis)
	{
		ndMatrix inertiaMatrix(CalculateInertiaMatrix());
		inertiaMatrix.EigenVectors();

		const FTransform tranform(GetComponentToWorld());
		const ndMatrix matrix(ToNewtonMatrix(tranform));

		FTransform offsetInertia;
		offsetInertia.SetRotation(Inertia.PrincipalInertiaAxis.Quaternion());
		const ndMatrix offsetMatrix(ToNewtonMatrix(offsetInertia));
		//const FRotator axisRot(tranform.GetRotation());
		//const FVector axisLoc(axisMatrix.m_posit.m_x * UNREAL_UNIT_SYSTEM, axisMatrix.m_posit.m_y * UNREAL_UNIT_SYSTEM, axisMatrix.m_posit.m_z * UNREAL_UNIT_SYSTEM);
		//DrawDebugCoordinateSystem(GetWorld(), axisLoc, axisRot, DebugScale * UNREAL_UNIT_SYSTEM, false, timestep);

		const ndMatrix axisMatrix(offsetMatrix * inertiaMatrix * matrix);
		const FTransform inertiaAxisTransform (ToUnRealTransform(axisMatrix));
		const FVector axisLoc(inertiaAxisTransform.GetLocation());
		const FRotator axisRot(inertiaAxisTransform.GetRotation());
		DrawDebugCoordinateSystem(GetWorld(), axisLoc, axisRot, DebugScale * UNREAL_UNIT_SYSTEM, false, timestep);
	}

	if (ShowCenterOfMass)
	{
		ndVector positVolume(ndFloat32(0.0f));
		if (m_body)
		{
			positVolume = m_body->GetCentreOfMass();
		}
		else
		{
			ndFixSizeArray<USceneComponent*, 1024> stack;
			stack.PushBack(this);

			ndFloat32 volume = ndFloat32(1.0e-3f);
			while (stack.GetCount())
			{
				const USceneComponent* const component = stack.Pop();
				const UNewtonCollision* const shape = Cast<UNewtonCollision>(component);
				if (shape)
				{
					const ndVector pv(shape->GetVolumePosition());
					volume += pv.m_w;
					positVolume += pv;
				}

				const TArray<TObjectPtr<USceneComponent>>& childrenComp = component->GetAttachChildren();
				for (int i = childrenComp.Num() - 1; i >= 0; --i)
				{
					stack.PushBack(childrenComp[i].Get());
				}
			}
			positVolume = positVolume.Scale(ndFloat32(1.0f) / volume);

			const ndVector centerOfMass(
				ndFloat32(CenterOfMass.X * UNREAL_INV_UNIT_SYSTEM), 
				ndFloat32(CenterOfMass.Y * UNREAL_INV_UNIT_SYSTEM), 
				ndFloat32(CenterOfMass.Z * UNREAL_INV_UNIT_SYSTEM),
				ndFloat32(0.0f));
			positVolume += centerOfMass;
			positVolume.m_w = ndFloat32(1.0f);
		}

		const FTransform tranform(GetComponentToWorld());
		const ndMatrix matrix(ToNewtonMatrix(tranform));
		positVolume = matrix.TransformVector(positVolume);

		const FRotator axisRot(tranform.GetRotation());
		const FVector axisLoc(positVolume.m_x * UNREAL_UNIT_SYSTEM, positVolume.m_y * UNREAL_UNIT_SYSTEM, positVolume.m_z * UNREAL_UNIT_SYSTEM);
		DrawDebugCoordinateSystem(GetWorld(), axisLoc, axisRot, DebugScale * UNREAL_UNIT_SYSTEM, false, timestep);
	}

	if (ShowDebug && m_body)
	{
		bool sleepState = m_body->GetSleepState();
		if (m_sleeping && !sleepState)
		{
			ndFixSizeArray<USceneComponent*, 1024> stack;
			stack.PushBack(this);
			while (stack.GetCount())
			{
				USceneComponent* const component = stack.Pop();
				UNewtonCollision* const shape = Cast<UNewtonCollision>(component);
				if (shape)
				{
					shape->SetWireFrameColor(m_awakeColor);
				}

				const TArray<TObjectPtr<USceneComponent>>& childrenComp = component->GetAttachChildren();
				for (int i = childrenComp.Num() - 1; i >= 0; --i)
				{
					stack.PushBack(childrenComp[i].Get());
				}
			}
		}
		else if (!m_sleeping && sleepState)
		{
			ndFixSizeArray<USceneComponent*, 1024> stack;
			stack.PushBack(this);
			while (stack.GetCount())
			{
				USceneComponent* const component = stack.Pop();
				UNewtonCollision* const shape = Cast<UNewtonCollision>(component);
				if (shape)
				{
					shape->SetWireFrameColor(m_sleepingColor);
				}

				const TArray<TObjectPtr<USceneComponent>>& childrenComp = component->GetAttachChildren();
				for (int i = childrenComp.Num() - 1; i >= 0; --i)
				{
					stack.PushBack(childrenComp[i].Get());
				}
			}
		}
		m_sleeping = sleepState;
	}
}

void UNewtonRigidBody::UpdateTransform()
{
	check(m_body);
	NotifyCallback* const notify = (NotifyCallback*)m_body->GetNotifyCallback();
	notify->UpdateTransform();
}

void UNewtonRigidBody::InterpolateTransform(float param)
{
	check(m_body);
	NotifyCallback* const notify = (NotifyCallback*)m_body->GetNotifyCallback();
	m_globalTransform = notify->InteptolateTransform(ndFloat32 (param));
	m_globalTransform.SetScale3D(m_globalScale);
}

void UNewtonRigidBody::CalculateLocalTransform()
{
	check(m_body);
	FTransform parentTransform;
	const USceneComponent* const parent = GetAttachParent();
	if (parent)
	{
		parentTransform = parentTransform = parent->GetComponentTransform();;
	}

	m_localTransform = m_globalTransform * parentTransform.Inverse();
	m_localTransform.SetScale3D(m_localScale);
}

void UNewtonRigidBody::ApplyPropertyChanges()
{
	m_propertyChanged = false;

	m_localTransform = GetRelativeTransform();
	m_globalTransform = GetComponentTransform();

	m_localScale = m_localTransform.GetScale3D();
	m_globalScale = m_globalTransform.GetScale3D();

	const ndMatrix inertiaMatrix(CalculateInertiaMatrix());
	//float scale = UNREAL_UNIT_SYSTEM * UNREAL_UNIT_SYSTEM * Mass;
	//show it in MKS units 
	//(not in centimeters because it is usually too big number)
	float scale = Mass;
	const FVector inertia(inertiaMatrix[0][0] * scale, inertiaMatrix[1][1] * scale, inertiaMatrix[2][2] * scale);
	Inertia.PrincipalInertia = inertia * Inertia.PrincipalInertiaScaler;

	if (ConvexApproximate.Generate)
	{
		const AActor* const owner = GetOwner();
		const TArray<TObjectPtr<USceneComponent>>& children = GetAttachChildren();
		for (int j = children.Num() - 1; j >= 0; --j)
		{
			UStaticMeshComponent* const staticMeshComponent = Cast<UStaticMeshComponent>(children[j]);

			if (staticMeshComponent && staticMeshComponent->GetOwner() && staticMeshComponent->GetStaticMesh().Get())
			{
				bool hasCollision = false;
				const TArray<TObjectPtr<USceneComponent>>& childrenComp = staticMeshComponent->GetAttachChildren();
				for (int i = childrenComp.Num() - 1; i >= 0; --i)
				{
					hasCollision = hasCollision || (Cast<UNewtonCollision>(childrenComp[i]) ? true : false);
				}

				if (hasCollision)
				{
					UE_LOG(LogTemp, Warning, TEXT("static mesh: %s, has one or more child collision shape alreary. You must delete all the UNewtonCollision children first"), *staticMeshComponent->GetName());
				}
				else
				{
					CreateConvexApproximationShapes(staticMeshComponent);
				}
			}
		}

		ConvexApproximate.Generate = false;
		FLevelEditorModule& levelEditor = FModuleManager::LoadModuleChecked<FLevelEditorModule>("LevelEditor");
		levelEditor.BroadcastComponentsEdited();
		levelEditor.BroadcastRedrawViewports(false);
	}
}

// Called every frame
void UNewtonRigidBody::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
	if (m_newtonWorld)
	{
		SetRelativeTransform(m_localTransform);
		SetComponentToWorld(m_globalTransform);

		DrawGizmo(DeltaTime);
	}
}

void UNewtonRigidBody::CreateConvexApproximationShapes(UStaticMeshComponent* const staticComponent)
{
	class ConveHullGenerator : public ndConvexApproximation
	{
		public:
		ConveHullGenerator(ndInt32 maxConvexes, bool quality)
			:ndConvexApproximation(maxConvexes, quality)
			,m_progressBar(nullptr)
			,m_acc(0.0f)
		{
		}
		
		~ConveHullGenerator()
		{
			check(!m_progressBar);
		}

		// for some reason the progress bar invalidate some UClasses
		// I need to report this some day to unreal.
		// for now just do not report progress.
		//void InitProgress()
		//{
		//	m_progressBar = new FScopedSlowTask(100.0f, NSLOCTEXT("Newton", "Newton", "Generation Convex Approximation"));
		//	m_progressBar->MakeDialog();
		//}
		//void EndProgress()
		//{
		//	check(m_progressBar);
		//	delete m_progressBar;
		//	m_progressBar = nullptr;
		//}

		virtual void Progress() override
		{
			//m_acc += 1.0f;
			//if (m_acc < 99.9f)
			//{
			//	m_progressBar->EnterProgressFrame();
			//}
		}

		FScopedSlowTask* m_progressBar;
		float m_acc;
	};

	ConveHullGenerator convexHullSet(ConvexApproximate.MaxConvexes, ConvexApproximate.HighResolution);

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

	ndHullInputMesh& inputMesh = convexHullSet.m_inputMesh;
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

	convexHullSet.Execute();

	AActor* const actor = staticComponent->GetOwner();
	check(actor);
	ndArray<ndHullOutput*>& hullArray = convexHullSet.m_ouputHulls;
	for (ndInt32 i = hullArray.GetCount() - 1; i >= 0; --i)
	{
		UNewtonCollisionConvexHull* const childConvex = Cast<UNewtonCollisionConvexHull>(actor->AddComponentByClass(UNewtonCollisionConvexHull::StaticClass(), false, FTransform(), true));
		actor->FinishAddComponent(childConvex, false, FTransform());
		actor->AddInstanceComponent(childConvex);
		childConvex->AttachToComponent(staticComponent, FAttachmentTransformRules::KeepRelativeTransform);
		childConvex->MarkRenderDynamicDataDirty();
		childConvex->NotifyMeshUpdated();

		const ndHullOutput* const convexHull = hullArray[i];
		childConvex->SetProceduralData(*convexHull);
	}
}

void UNewtonRigidBody::CreateRigidBody(ANewtonWorldActor* const worldActor, bool overrideAutoSleep)
{
	m_newtonWorld = worldActor;
	const ndMatrix matrix(ToNewtonMatrix(m_globalTransform));
	m_body = new ndBodyDynamic();
	m_body->SetMatrix(matrix);

	ndShapeInstance* const shape = CreateCollision(matrix);
	m_body->SetCollisionShape(*shape);
	m_body->SetMassMatrix(Mass, *shape);
	m_body->SetAutoSleep(AutoSleepMode && overrideAutoSleep);
	m_body->SetNotifyCallback(new NotifyCallback(this, ndVector(ndFloat32(Gravity.X * UNREAL_INV_UNIT_SYSTEM), ndFloat32(Gravity.Y * UNREAL_INV_UNIT_SYSTEM), ndFloat32(Gravity.Z * UNREAL_INV_UNIT_SYSTEM), ndFloat32(0.0f))));

	m_body->SetLinearDamping(LinearDamp);
	m_body->SetAngularDamping(ndVector(AngularDamp));
	m_body->SetOmega(ndVector(ndFloat32(InitialOmega.X), ndFloat32(InitialOmega.Y), ndFloat32(InitialOmega.Z), ndFloat32(0.0f)));
	m_body->SetVelocity(ndVector(ndFloat32(InitialVeloc.X * UNREAL_INV_UNIT_SYSTEM), ndFloat32(InitialVeloc.Y * UNREAL_INV_UNIT_SYSTEM), ndFloat32(InitialVeloc.Z * UNREAL_INV_UNIT_SYSTEM), ndFloat32(0.0f)));

	ndVector centerOfGravity(m_body->GetCentreOfMass());
	centerOfGravity += ndVector(ndFloat32(CenterOfMass.X * UNREAL_INV_UNIT_SYSTEM), ndFloat32(CenterOfMass.Y * UNREAL_INV_UNIT_SYSTEM), ndFloat32(CenterOfMass.Z * UNREAL_INV_UNIT_SYSTEM), ndFloat32(0.0f));
	m_body->SetCentreOfMass(centerOfGravity);

	ndWorld* world = m_newtonWorld->GetNewtonWorld();
	world->AddBody(m_body);
	delete shape;

	AActor* const actor = GetOwner();
	m_sleeping = false;
	ndFixSizeArray<USceneComponent*, 1024> stack;
	stack.PushBack(this);
	while (stack.GetCount())
	{
		USceneComponent* const component = stack.Pop();
		UNewtonCollision* const collision = Cast<UNewtonCollision>(component);
		if (collision)
		{
			collision->SetWireFrameColor(m_awakeColor);
		}

		const TArray<TObjectPtr<USceneComponent>>& childrenComp = component->GetAttachChildren();
		for (int i = childrenComp.Num() - 1; i >= 0; --i)
		{
			stack.PushBack(childrenComp[i].Get());
		}
	}
}

ndShapeInstance* UNewtonRigidBody::CreateCollision(const ndMatrix& bodyMatrix) const
{
	ndArray<UNewtonCollision*> collisionShapes;
	ndFixSizeArray<USceneComponent*, 1024> stack;
	stack.PushBack((USceneComponent*)this);
	while (stack.GetCount())
	{
		USceneComponent* const component = stack.Pop();
		UNewtonCollision* const shape = Cast<UNewtonCollision>(component);
		if (shape)
		{
			shape->ApplyPropertyChanges();
			collisionShapes.PushBack(shape);
		}
		const TArray<TObjectPtr<USceneComponent>>& childrenComp = component->GetAttachChildren();
		for (int i = childrenComp.Num() - 1; i >= 0; --i)
		{
			stack.PushBack(childrenComp[i].Get());
		}
	}

	if (collisionShapes.GetCount() == 0)
	{
		return new ndShapeInstance(new ndShapeNull());
	}

	ndShapeInstance* const compoundInstance = new ndShapeInstance(new ndShapeCompound());
	ndShapeCompound* const compound = compoundInstance->GetShape()->GetAsShapeCompound();
	compound->BeginAddRemove();
	for (ndInt32 i = collisionShapes.GetCount() - 1; i >= 0; --i)
	{
		ndShapeInstance* const subShape = collisionShapes[i]->CreateBodyInstanceShape(bodyMatrix);
		compound->AddCollision(subShape);
		delete subShape;
	}
	compound->EndAddRemove();
	return compoundInstance;
}
