// Fill out your copyright notice in the Description page of Project Settings.

#include "NewtonSceneActor.h"
#include "Selection.h"
#include "LevelEditor.h"
#include "EngineUtils.h"
#include "LandscapeProxy.h"
#include "LandscapeStreamingProxy.h"

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
#include "ThirdParty/newtonLibrary/Public/thirdParty/ndConvexApproximation.h"

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
				childComp->InitStaticMeshCompoment(staticComponent);
				childComp->MarkRenderDynamicDataDirty();
				childComp->NotifyMeshUpdated();
			};

			for (ndInt32 i = aggGeom.SphereElems.Num() - 1; i >= 0; --i)
			{
				UNewtonCollisionSphere* const child = Cast<UNewtonCollisionSphere>(AddComponentByClass(UNewtonCollisionSphere::StaticClass(), false, FTransform(), true));
				AddComponent(child);
				hasSimple = true;
			}

			for (ndInt32 i = aggGeom.SphylElems.Num() - 1; i >= 0; --i)
			{
				UNewtonCollisionCapsule* const child = Cast<UNewtonCollisionCapsule>(AddComponentByClass(UNewtonCollisionCapsule::StaticClass(), false, FTransform(), true));
				AddComponent(child);
				hasSimple = true;
			}

			for (ndInt32 i = aggGeom.BoxElems.Num() - 1; i >= 0; --i)
			{
				UNewtonCollisionBox* const child = Cast<UNewtonCollisionBox>(AddComponentByClass(UNewtonCollisionBox::StaticClass(), false, FTransform(), true));
				AddComponent(child);
				hasSimple = true;
			}

			for (ndInt32 i = aggGeom.ConvexElems.Num() - 1; i >= 0; --i)
			{
				UNewtonCollisionConvexHull* const child = Cast<UNewtonCollisionConvexHull>(AddComponentByClass(UNewtonCollisionConvexHull::StaticClass(), false, FTransform(), true));
				AddComponent(child);
				hasSimple = true;
			}

			for (ndInt32 i = aggGeom.TaperedCapsuleElems.Num() - 1; i >= 0; --i)
			{
				check(0);
			}

			for (ndInt32 i = aggGeom.LevelSetElems.Num() - 1; i >= 0; --i)
			{
				check(0);
			}

			for (ndInt32 i = aggGeom.SkinnedLevelSetElems.Num() - 1; i >= 0; --i)
			{
				check(0);
			}

			return hasSimple;
		};

		auto GenerateComplexCollision = [this, staticComponent]()
		{
			UNewtonCollisionPolygonalMesh* const childComp = Cast<UNewtonCollisionPolygonalMesh>(AddComponentByClass(UNewtonCollisionPolygonalMesh::StaticClass(), false, FTransform(), true));
			FinishAddComponent(childComp, false, FTransform());
			AddInstanceComponent(childComp);
			childComp->AttachToComponent(RootBody, FAttachmentTransformRules::KeepRelativeTransform);
			childComp->InitStaticMeshCompoment(staticComponent);
			childComp->MarkRenderDynamicDataDirty();
			childComp->NotifyMeshUpdated();
		};

		const UBodySetup* const bodySetup = staticMesh->GetBodySetup();
		ECollisionTraceFlag collisionFlag = bodySetup->GetCollisionTraceFlag();
		switch (collisionFlag)
		{
			case CTF_UseDefault:
			{
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

	UNewtonSceneRigidBody* const staticSceneBody = FindComponentByClass<UNewtonSceneRigidBody>();
	if (!staticSceneBody)
	{
		return;
	}

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
			else if (Cast<ALandscapeStreamingProxy>(actor))
			{
				const ALandscapeStreamingProxy* const streamingProxy = Cast<ALandscapeStreamingProxy>(actor);
				const AActor* const parent = streamingProxy->GetSceneOutlinerParent();
				const FString streamingKey(parent->GetFolder().ToString());
				index = streamingKey.Find(key);
				if (index == 0)
				{
					actorList.PushBack(actor);
				}
			}
		}
	}

	staticSceneBody->RemoveAllCollisions();
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
//if (sceneActor->GetName() == FString("Railing_Stairs_001"))
			GenerateStaticMeshCollision(sceneActor);
		}
	}

	FLevelEditorModule& levelEditor = FModuleManager::LoadModuleChecked<FLevelEditorModule>("LevelEditor");
	levelEditor.BroadcastComponentsEdited();
	levelEditor.BroadcastRedrawViewports(false);
}

void ANewtonSceneActor::GenerateLandScapeCollision(const ALandscapeProxy* const landscapeProxy)
{
	const TArray<TObjectPtr<ULandscapeHeightfieldCollisionComponent>>& landScapeTiles = landscapeProxy->CollisionComponents;
	for (ndInt32 i = 0; i < landScapeTiles.Num(); ++i)
	{
		const TObjectPtr<ULandscapeHeightfieldCollisionComponent>& tile = landScapeTiles[i];
		UNewtonCollisionLandscape* const collisionTile = Cast<UNewtonCollisionLandscape>(AddComponentByClass(UNewtonCollisionLandscape::StaticClass(), false, FTransform(), true));
		FinishAddComponent(collisionTile, false, FTransform());
		AddInstanceComponent(collisionTile);
		collisionTile->AttachToComponent(RootBody, FAttachmentTransformRules::KeepRelativeTransform);

		collisionTile->InitStaticMeshCompoment(tile);
		collisionTile->MarkRenderDynamicDataDirty();
		collisionTile->NotifyMeshUpdated();
	}
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

	for (ndInt32 i = staticMesh.Num() - 1; i >= 0; --i)
	{
		TObjectPtr<UStaticMeshComponent>meshComponent(staticMesh[i]);
		CreateCollisionFromUnrealPrimitive(meshComponent);
	}
}

