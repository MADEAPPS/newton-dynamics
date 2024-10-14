// Fill out your copyright notice in the Description page of Project Settings.

#include "NewtonSceneActor.h"
#include "Selection.h"
#include "LevelEditor.h"
#include "EngineUtils.h"
#include "LandscapeProxy.h"

#include "Newton.h"
#include "NewtonCollision.h"
#include "NewtonWorldActor.h"
#include "NewtonCollisionBox.h"
#include "NewtonSceneRigidBody.h"
#include "NewtonCollisionSphere.h"
#include "NewtonCollisionCapsule.h"
#include "NewtonCollisionLandscape.h"
#include "NewtonCollisionConvexHull.h"
#include "NewtonCollisionPolygonalMesh.h"
#include "ThirdParty/newtonLibrary/Public/thirdParty/GenerateConvecApproximation.h"

// Sets default values
ANewtonSceneActor::ANewtonSceneActor()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	GenerateShapes = false;
	m_propertyChanged = false;
	
	RootBody = CreateDefaultSubobject<UNewtonSceneRigidBody>(TEXT("NewtonStaticBody"));
	RootComponent = RootBody;
	//RootBody->Mobility = EComponentMobility::Static;
}

void ANewtonSceneActor::PostLoad()
{
	Super::PostLoad();
	m_propertyChanged = true;
}

void ANewtonSceneActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);
	m_propertyChanged = true;
}

void ANewtonSceneActor::CreateCollisionFromUnrealPrimitive(TObjectPtr<UStaticMeshComponent> staticComponent)
{
	const UStaticMesh* const staticMesh = staticComponent->GetStaticMesh().Get();
	if (staticMesh)
	{
		auto GenerateSimpleCollision = [this, staticComponent]()
		{
			bool hasSimple = false;
			//AActor* const actor = staticComponent->GetOwner();
			const UStaticMesh* const staticMesh = staticComponent->GetStaticMesh().Get();
			const UBodySetup* const bodySetup = staticMesh->GetBodySetup();

			const FKAggregateGeom& aggGeom = bodySetup->AggGeom;
			auto AddComponent = [this, staticComponent](UNewtonCollision* const childComp)
			{
				//UE_LOG(LogTemp, Warning, TEXT("From NewtonSceneRigidBody, component render but does not shows in blueprint browser"));
				check(IsValid(childComp));
				FinishAddComponent(childComp, false, FTransform());
				AddInstanceComponent(childComp);
				childComp->AttachToComponent(RootBody, FAttachmentTransformRules::KeepRelativeTransform);
				childComp->MarkRenderDynamicDataDirty();
				childComp->NotifyMeshUpdated();
				childComp->SetGeometryMesh(staticComponent);
			};

			for (int i = aggGeom.SphereElems.Num() - 1; i >= 0; --i)
			{
				UNewtonCollisionSphere* const child = Cast<UNewtonCollisionSphere>(AddComponentByClass(UNewtonCollisionSphere::StaticClass(), false, FTransform(), true));
				AddComponent(child);
				hasSimple = true;
			}

			for (int i = aggGeom.SphylElems.Num() - 1; i >= 0; --i)
			{
				UNewtonCollisionCapsule* const child = Cast<UNewtonCollisionCapsule>(AddComponentByClass(UNewtonCollisionCapsule::StaticClass(), false, FTransform(), true));
				AddComponent(child);
				hasSimple = true;
			}

			for (int i = aggGeom.BoxElems.Num() - 1; i >= 0; --i)
			{
				UNewtonCollisionBox* const child = Cast<UNewtonCollisionBox>(AddComponentByClass(UNewtonCollisionBox::StaticClass(), false, FTransform(), true));
				AddComponent(child);
				hasSimple = true;
			}

			for (int i = aggGeom.ConvexElems.Num() - 1; i >= 0; --i)
			{
				UNewtonCollisionConvexHull* const child = Cast<UNewtonCollisionConvexHull>(AddComponentByClass(UNewtonCollisionConvexHull::StaticClass(), false, FTransform(), true));
				AddComponent(child);
				hasSimple = true;
			}

			for (int i = aggGeom.TaperedCapsuleElems.Num() - 1; i >= 0; --i)
			{
				check(0);
			}

			for (int i = aggGeom.LevelSetElems.Num() - 1; i >= 0; --i)
			{
				check(0);
			}

			for (int i = aggGeom.SkinnedLevelSetElems.Num() - 1; i >= 0; --i)
			{
				check(0);
			}

			return hasSimple;
		};

		auto GenerateComplexCollision = [this, staticComponent]()
		{
			//AActor* const actor = staticComponent->GetOwner();
			auto AddComponent = [this, staticComponent](UNewtonCollision* const childComp)
			{
				FinishAddComponent(childComp, false, FTransform());
				AddInstanceComponent(childComp);
				childComp->AttachToComponent(RootBody, FAttachmentTransformRules::KeepRelativeTransform);
				childComp->MarkRenderDynamicDataDirty();
				childComp->NotifyMeshUpdated();
				childComp->SetGeometryMesh(staticComponent);

				//const FTransform bodyTransform(RootBody->GetComponentToWorld());
				//const FTransform meshTransform(staticComponent->GetComponentToWorld());
				//const FTransform transform(meshTransform * bodyTransform.Inverse());
				//childComp->SetComponentToWorld(transform);
			};

			UNewtonCollisionPolygonalMesh* const child = Cast<UNewtonCollisionPolygonalMesh>(AddComponentByClass(UNewtonCollisionPolygonalMesh::StaticClass(), false, FTransform(), true));
			AddComponent(child);
		};

		const UBodySetup* const bodySetup = staticMesh->GetBodySetup();
		ECollisionTraceFlag collisionFlag = bodySetup->GetCollisionTraceFlag();
		switch (collisionFlag)
		{
			case CTF_UseDefault:
			{
				check(0);
				if (!GenerateSimpleCollision())
				{
					GenerateComplexCollision();
				}
				break;
			}
			case CTF_UseSimpleAndComplex:
			{
				if (!GenerateSimpleCollision())
				{
					GenerateComplexCollision();
				}
				break;
			}
			case CTF_UseSimpleAsComplex:
			{
				//GenerateSimpleCollision();
				break;
			}
			case CTF_UseComplexAsSimple:
			{
				check(0);
				GenerateComplexCollision();
				break;
			}
		}
	}
}

void ANewtonSceneActor::ApplyPropertyChanges()
{
	if (!m_propertyChanged)
	{
		return;
	}
	m_propertyChanged = false;

	if (!GenerateShapes)
	{
		return;
	}
	GenerateShapes = false;

	FFolder folder(GetFolder());
	if (folder.IsNone())
	{
		UE_LOG(LogTemp, Warning, TEXT("NewtonSceneActor must be child of an oulier folder filder"));
		return;
	}

	ndArray<AActor*> actorList;
	const UWorld* const world = GetWorld();
	const FString key(folder.GetPath().ToString());
	for (TActorIterator<AActor> actorItr(world); actorItr; ++actorItr)
	{
		AActor* const actor = *actorItr;
		if (actor != this)
		{
			const FString key1(actor->GetFolder().ToString());
			int index = key1.Find(key);
			if (index == 0)
			{
				actorList.PushBack(actor);
			}
		}
	}

	for (ndInt32 i = ndInt32(actorList.GetCount()) - 1; i >= 0; --i)
	{
		AActor* const sceneActor = actorList[i];
		const ALandscapeProxy* const landscapeProxy = Cast<ALandscapeProxy>(sceneActor);
		if (landscapeProxy)
		{
			GenerateLandScapeCollision(landscapeProxy);
		}
		else
		{
			GenerateStaticMeshCollision(sceneActor);
		}
	}

	FLevelEditorModule& levelEditor = FModuleManager::LoadModuleChecked<FLevelEditorModule>("LevelEditor");
	levelEditor.BroadcastComponentsEdited();
	levelEditor.BroadcastRedrawViewports(false);
}

void ANewtonSceneActor::GenerateStaticMeshCollision(const AActor* const actor)
{
	TArray<TObjectPtr<USceneComponent>> stack;
	TArray<TObjectPtr<UStaticMeshComponent>> staticMesh;

	stack.Push(TObjectPtr<USceneComponent>(actor->GetRootComponent()));
	while (stack.Num())
	{
		TObjectPtr<USceneComponent> component(stack.Pop());
		TObjectPtr<UStaticMeshComponent> mesh(Cast<UStaticMeshComponent>(component));
		if (mesh)
		{
			staticMesh.Push(mesh);
		}
		const TArray<TObjectPtr<USceneComponent>>& children = component->GetAttachChildren();
		for (ndInt32 i = children.Num() - 1; i >= 0; --i)
		{
			stack.Push(children[i].Get());
		}
	}

	const TArray<TObjectPtr<USceneComponent>>& children = RootBody->GetAttachChildren();
	auto HasCollision = [this, &children](const UStaticMeshComponent* const meshComp)
		{
			for (ndInt32 i = children.Num() - 1; i >= 0; --i)
			{
				const UNewtonCollision* const collision = Cast<UNewtonCollision>(children[i]);
				if (collision && (collision->GetGeometryMesh() == meshComp))
				{
					return true;
				}
			}
			return false;
		};

	for (ndInt32 i = staticMesh.Num() - 1; i >= 0; --i)
	{
		TObjectPtr<UStaticMeshComponent>meshComponent(staticMesh[i]);
		if (!HasCollision(meshComponent.Get()))
		{
			CreateCollisionFromUnrealPrimitive(meshComponent);
		}
	}
}

void ANewtonSceneActor::GenerateLandScapeCollision(const ALandscapeProxy* const landscapeProxy)
{
	const TArray<TObjectPtr<ULandscapeHeightfieldCollisionComponent>>& landScapeTiles = landscapeProxy->CollisionComponents;
	check(landScapeTiles.Num());
	const TArray<TObjectPtr<USceneComponent>>& children = RootBody->GetAttachChildren();

	auto HasCollision = [this, &children](const TObjectPtr<ULandscapeHeightfieldCollisionComponent>& tile)
	{
		for (ndInt32 i = children.Num() - 1; i >= 0; --i)
		{
			UNewtonCollisionLandscape* const collision = Cast<UNewtonCollisionLandscape>(children[i]);
			if (collision && (collision->GetGeometryMesh() == tile))
			{
				return true;
			}
		}
		return false;
	};

	for (ndInt32 i = landScapeTiles.Num() - 1; i >= 0; --i)
	//for (ndInt32 i = 0; i < 1; ++i)
	{
		const TObjectPtr<ULandscapeHeightfieldCollisionComponent>& tile = landScapeTiles[i];
		if (!HasCollision(tile))
		{
			UNewtonCollisionLandscape* const collisionTile = Cast<UNewtonCollisionLandscape>(AddComponentByClass(UNewtonCollisionLandscape::StaticClass(), false, FTransform(), true));
			FinishAddComponent(collisionTile, false, FTransform());
			AddInstanceComponent(collisionTile);
			collisionTile->AttachToComponent(RootBody, FAttachmentTransformRules::KeepRelativeTransform);
			collisionTile->MarkRenderDynamicDataDirty();
			collisionTile->NotifyMeshUpdated();
			collisionTile->SetGeometryMesh(tile);
		}
	}
}