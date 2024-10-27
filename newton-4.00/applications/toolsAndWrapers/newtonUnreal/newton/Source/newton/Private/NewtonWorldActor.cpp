// Fill out your copyright notice in the Description page of Project Settings.


#include "NewtonWorldActor.h"
#include "Newton.h"
#include "EngineUtils.h"
#include "LevelEditor.h"
#include "Kismet/GameplayStatics.h"
#include "Interfaces/IPluginManager.h"

#include "NewtonWorld.h"
#include "NewtonJoint.h"
#include "NewtonRigidBody.h"
#include "ThirdParty/newtonLibrary/Public/dNewton/ndNewton.h"

// Sets default values
ANewtonWorldActor::ANewtonWorldActor()
	:m_world(nullptr)
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	//PrimaryActorTick.bCanEverTick = true;

	m_beginPlay = false;
	SolverPasses = 2;
	UpdateRate = 60.0f;
	ParallelThreads = 1;
	SolverIterations = 4;

	ShowDebug = false;
	ClearDebug = false;
	
	AutoSleepMode = true;
	SolverMode = SolverModeTypes::scalar;
}

ndWorld* ANewtonWorldActor::GetNewtonWorld() const
{
	return m_world ? m_world->GetNewtonWorld() : nullptr;
}

// Called when the game starts or when spawned
void ANewtonWorldActor::BeginPlay()
{
	Super::BeginPlay();
	m_beginPlay = true;
	if (!m_world)
	{
		m_world = new NewtonWorld(this);
	}
	ApplySettings();
	m_world->Sync();
}

void ANewtonWorldActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);
	if (m_world)
	{
		Cleanup();
		delete m_world;
		//switch (EndPlayReason)
		//{
		//	case EEndPlayReason::Destroyed:
		//		ndAssert(0);
		//		break;
		//	case EEndPlayReason::LevelTransition:
		//		ndAssert(0);
		//		break;
		//	case EEndPlayReason::EndPlayInEditor:
		//		ndAssert(0);
		//		break;
		//	case EEndPlayReason::RemovedFromWorld:
		//		ndAssert(0);
		//		break;
		//	case EEndPlayReason::Quit:
		//		m_world->Sync();
		//		delete m_world;
		//		break;
		//}
	}
}

void ANewtonWorldActor::StartGame()
{
	m_beginPlay = false;
	if (m_world)
	{
		m_world->StartGame();
	}
}

void ANewtonWorldActor::Cleanup()
{
	m_world->Sync();
	UWorld* const world = GetWorld();

	// remove all joints body actors.
	for (TActorIterator<AActor> actorItr(world); actorItr; ++actorItr)
	{
		AActor* const actor = *actorItr;
		if (actor->FindComponentByClass(UNewtonRigidBody::StaticClass()))
		{
			const TSet<UActorComponent*>& components = actor->GetComponents();
			for (TSet<UActorComponent*>::TConstIterator it(components.CreateConstIterator()); it; ++it)
			{
				UNewtonJoint* const joint = Cast<UNewtonJoint>(*it);
				if (joint)
				{
					joint->DestroyJoint();
				}
			}
		}
	}

	// remove all rigid body actors.
	for (TActorIterator<AActor> actorItr(world); actorItr; ++actorItr)
	{
		AActor* const actor = *actorItr;
		if (actor->FindComponentByClass(UNewtonRigidBody::StaticClass()))
		{
			const TSet<UActorComponent*>& components = actor->GetComponents();
			for (TSet<UActorComponent*>::TConstIterator it(components.CreateConstIterator()); it; ++it)
			{
				UNewtonRigidBody* const meshComp = Cast<UNewtonRigidBody>(*it);
				if (meshComp)
				{
					meshComp->DestroyRigidBody();
				}
			}
		}
	}
}

void ANewtonWorldActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	AActor::PostEditChangeProperty(PropertyChangedEvent);
	if (m_world)
	{
		ApplySettings();
	}

	bool clearOption = PropertyChangedEvent.GetPropertyName() == "ClearDebug";
	if (clearOption || (PropertyChangedEvent.GetPropertyName() == "ShowDebug"))
	{
		const UWorld* const world = GetWorld();
		for (TActorIterator<AActor> actorItr(world); actorItr; ++actorItr)
		{
			AActor* const actor = *actorItr;
			const TSet<UActorComponent*>& components = actor->GetComponents();
			for (TSet<UActorComponent*>::TConstIterator it(components.CreateConstIterator()); it; ++it)
			{
				UNewtonRigidBody* const body = Cast<UNewtonRigidBody>(*it);
				if (body)
				{
					clearOption ? body->ClearDebug() : body->ActivateDebug();
				}

				UNewtonJoint* const joint = Cast<UNewtonJoint>(*it);
				if (joint)
				{
					clearOption ? joint->ClearDebug() : joint->ActivateDebug();
				}
			}
		}

		ShowDebug = false;
		ClearDebug = false;
		FLevelEditorModule& levelEditor = FModuleManager::LoadModuleChecked<FLevelEditorModule>("LevelEditor");
		levelEditor.BroadcastComponentsEdited();
		levelEditor.BroadcastRedrawViewports(false);
	}
}

float ANewtonWorldActor::GetSimTime() const
{
	return float (m_world ? m_world->GetAverageUpdateTime(): 0.0f);
}

void ANewtonWorldActor::ApplySettings()
{
	if (m_world)
	{
		m_world->ApplySettings();
	}
}

// surrogate tick function
void ANewtonWorldActor::Update(float timestep)
{
	if (m_world)
	{
		m_world->Update(timestep);
	}
}
