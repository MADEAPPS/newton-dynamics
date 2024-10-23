// Fill out your copyright notice in the Description page of Project Settings.


#include "NewtonWorldActor.h"
#include "Newton.h"
#include "EngineUtils.h"
#include "LevelEditor.h"
#include "Kismet/GameplayStatics.h"
#include "Interfaces/IPluginManager.h"

#include "NewtonJoint.h"
#include "NewtonRigidBody.h"
#include "ThirdParty/newtonLibrary/Public/dNewton/ndNewton.h"

class ANewtonWorldActor::NewtonWorld: public ndWorld
{
	public:
	NewtonWorld(ANewtonWorldActor* const owner)
		:ndWorld()
		,m_owner(owner)
	{
	}

	virtual ~NewtonWorld()
	{
	}

	virtual void UpdateTransforms()
	{
		ndWorld::UpdateTransforms();

		const ndArray<ndBodyKinematic*>& bodyList = GetBodyList().GetView();
		for (ndInt32 i = bodyList.GetCount() - 1; i >= 0; --i)
		{
			ndBodyKinematic* const body = bodyList[i];
			ndBodyNotify* const notify = body->GetNotifyCallback();
			if (notify)
			{
				UNewtonRigidBody* const meshComp = Cast<UNewtonRigidBody>((USceneComponent*)notify->GetUserData());
				if (meshComp)
				{
					meshComp->UpdateTransform();
				}
			}
		}
	}

	ANewtonWorldActor* m_owner;
};

// Sets default values
ANewtonWorldActor::ANewtonWorldActor()
	:m_world(nullptr)
	,m_timeAccumulator(0.0f)
	,m_interpolationParam(1.0f)
	,m_beginPlay(false)
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	//PrimaryActorTick.bCanEverTick = true;

	SolverPasses = 2;
	UpdateRate = 60.0f;
	ParallelThreads = 1;
	SolverIterations = 4;

	ShowDebug = false;
	ClearDebug = false;
	
	AutoSleepMode = true;
	ConcurrentUpdate = false;
	SolverMode = SolverModeTypes::scalar;
}

ndWorld* ANewtonWorldActor::GetNewtonWorld() const
{
	return m_world;
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

void ANewtonWorldActor::StartGame()
{
	m_beginPlay = false;

	check(m_world);

	m_world->Sync();
	
	UWorld* const world = GetWorld();
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
					meshComp->CreateRigidBody(this, AutoSleepMode);
				}
			}
		}
	}
	
	for (TActorIterator<AActor> actorItr(world); actorItr; ++actorItr)
	{
		AActor* const actor = *actorItr;
		if (actor->FindComponentByClass(UNewtonRigidBody::StaticClass()))
		{
			UNewtonJoint* const component = Cast<UNewtonJoint>(actor->FindComponentByClass(UNewtonJoint::StaticClass()));
			if (component)
			{
				component->CreateJoint(this);
			}
		}
	}
	
	m_timeAccumulator = 0.0f;
}

void ANewtonWorldActor::Cleanup()
{
	m_world->Sync();

	UWorld* const world = GetWorld();

	m_world->Sync();

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

void ANewtonWorldActor::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
	Super::EndPlay(EndPlayReason);
	if (m_world)
	{
		Cleanup();
		switch (EndPlayReason)
		{
			case EEndPlayReason::Destroyed:
				ndAssert(0);
				break;
			case EEndPlayReason::LevelTransition:
				ndAssert(0);
				break;
			case EEndPlayReason::EndPlayInEditor:
				ndAssert(0);
				break;
			case EEndPlayReason::RemovedFromWorld:
				ndAssert(0);
				break;
			case EEndPlayReason::Quit:
				m_world->Sync();
				delete m_world;
				break;
		}
	}
}

void ANewtonWorldActor::Destroyed()
{
	Super::Destroyed();

	if (m_world)
	{
		m_world->Sync();
		delete m_world;
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
	return float (m_world->GetAverageUpdateTime());
}

void ANewtonWorldActor::ApplySettings()
{
	m_world->Sync();
	
	ndWorld::ndSolverModes mode = ndWorld::ndStandardSolver;
	switch(SolverMode)
	{
		case SolverModeTypes::scalar:
			mode = ndWorld::ndStandardSolver;
			break;
		case SolverModeTypes::soaSimd:
			mode = ndWorld::ndSimdSoaSolver;
			break;
	};
	
	m_world->SelectSolver(mode);
	m_world->SetSubSteps(SolverPasses);
	m_world->SetThreadCount(ParallelThreads);
	m_world->SetSolverIterations(SolverIterations);
	
	const ndBodyListView& bodyList = m_world->GetBodyList();
	for (ndBodyListView::ndNode* node = bodyList.GetFirst(); node; node = node->GetNext())
	{
		ndBodyKinematic* const body = node->GetInfo()->GetAsBodyKinematic();
		body->SetAutoSleep(AutoSleepMode);
	}
}

// Called every frame
void ANewtonWorldActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

// tick surugate
void ANewtonWorldActor::Update(float timestep)
{
	if (m_world)
	{
		const ndFloat32 descreteStep = (1.0f / UpdateRate);
		m_timeAccumulator += timestep;

		if (m_timeAccumulator > descreteStep * ndFloat32 (2.0f))
		{
			// truncate slow frame updates 
			m_timeAccumulator = m_timeAccumulator - descreteStep * ndFloor(m_timeAccumulator / descreteStep) + descreteStep;
		}

		UWorld* const world = GetWorld();
		while (m_timeAccumulator > descreteStep)
		{
			m_world->Update(descreteStep);
			for (TActorIterator<AActor> actorItr(world); actorItr; ++actorItr)
			{
				AActor* const actor = *actorItr;
				if (actor->FindComponentByClass(UNewtonRigidBody::StaticClass()))
				{
					const TSet<UActorComponent*>& components = actor->GetComponents();
					for (TSet<UActorComponent*>::TConstIterator it(components.CreateConstIterator()); it; ++it)
					{
						UNewtonRigidBody* const rigidBody = Cast<UNewtonRigidBody>(*it);
						if (rigidBody)
						{
							rigidBody->InterpolateTransform(m_interpolationParam);
						}
					}
				}
			}

			m_timeAccumulator -= descreteStep;
			if (!ConcurrentUpdate)
			{
				m_world->Sync();
			}
		}

		m_interpolationParam = ndClamp(m_timeAccumulator / descreteStep, ndFloat32(0.0f), ndFloat32(1.0f));

		for (TActorIterator<AActor> actorItr(world); actorItr; ++actorItr)
		{
			AActor* const actor = *actorItr;
			if (actor->FindComponentByClass(UNewtonRigidBody::StaticClass()))
			{
				const TSet<UActorComponent*>& components = actor->GetComponents();
				for (TSet<UActorComponent*>::TConstIterator it(components.CreateConstIterator()); it; ++it)
				{
					UNewtonRigidBody* const rigidBody = Cast<UNewtonRigidBody>(*it);
					if (rigidBody)
					{
						rigidBody->InterpolateTransform(m_interpolationParam);
					}
				}
			}
		}

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
						meshComp->CalculateLocalTransform();
					}
				}
			}
		}

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
						joint->UpdateTransform();
					}
				}
			}
		}
	}
}
