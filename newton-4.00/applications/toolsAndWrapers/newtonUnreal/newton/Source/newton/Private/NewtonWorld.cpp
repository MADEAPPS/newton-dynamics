// Fill out your copyright notice in the Description page of Project Settings.

#include "NewtonWorld.h"
#include "EngineUtils.h"

#include "NewtonJoint.h"
#include "NewtonRigidBody.h"
#include "NewtonWorldActor.h"
#include "ThirdParty/newtonLibrary/Public/dNewton/ndNewton.h"

class NewtonWorld::PhysicsEngine : public ndWorld
{
	public:
	PhysicsEngine()
		:ndWorld()
	{
	}

	void UpdateTransforms() override
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
};

#ifndef D_USING_UNREAL_TRHEADS

NewtonWorld::NewtonWorld(ANewtonWorldActor* const owner)
	:m_world(nullptr)
	,m_owner(owner)
	,m_timeAccumulator(0.0f)
{
	m_world = new PhysicsEngine;
}

NewtonWorld::~NewtonWorld()
{
	if (m_world)
	{
		delete m_world;
	}
}

ndWorld* NewtonWorld::GetNewtonWorld() const
{
	return m_world;
}

float NewtonWorld::GetAverageUpdateTime() const
{
	return m_world->GetAverageUpdateTime();
}

void NewtonWorld::Sync()
{
	m_world->Sync();
}

void NewtonWorld::ApplySettings() const
{
	m_world->Sync();
	ndWorld::ndSolverModes mode = ndWorld::ndStandardSolver;
	switch(m_owner->SolverMode)
	{
		case SolverModeTypes::scalar:
			mode = ndWorld::ndStandardSolver;
			break;
		case SolverModeTypes::soaSimd:
			mode = ndWorld::ndSimdSoaSolver;
			break;
	};
	
	m_world->SelectSolver(mode);
	m_world->SetSubSteps(m_owner->SolverPasses);
	m_world->SetThreadCount(m_owner->ParallelThreads);
	m_world->SetSolverIterations(m_owner->SolverIterations);
	
	const ndBodyListView& bodyList = m_world->GetBodyList();
	for (ndBodyListView::ndNode* node = bodyList.GetFirst(); node; node = node->GetNext())
	{
		ndBodyKinematic* const body = node->GetInfo()->GetAsBodyKinematic();
		body->SetAutoSleep(m_owner->AutoSleepMode);
	}
}

void NewtonWorld::StartGame()
{
	m_world->Sync();
	
	UWorld* const world = m_owner->GetWorld();
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
					meshComp->CreateRigidBody(m_owner, m_owner->AutoSleepMode);
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
				component->CreateJoint(m_owner);
			}
		}
	}
	
	m_timeAccumulator = 0.0f;
}

void NewtonWorld::Update(float timestep)
{
	m_timeAccumulator += timestep;
	PhysicsTick();
	VisualTick();
}

void NewtonWorld::VisualTick()
{
	const ndFloat32 descreteStep = 1.0f / m_owner->UpdateRate;
	ndFloat32 interpolationParam = ndClamp(m_timeAccumulator / descreteStep, ndFloat32(0.0f), ndFloat32(1.0f));
	
	UWorld* const world = m_owner->GetWorld();
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
					rigidBody->InterpolateTransform(interpolationParam);
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

void NewtonWorld::PhysicsTick()
{
	const ndFloat32 descreteStep = ndFloat32(1.0f) / m_owner->UpdateRate;
	
	if (m_timeAccumulator > descreteStep * ndFloat32(2.0f))
	{
		// truncate slow frame updates 
		m_timeAccumulator = m_timeAccumulator - descreteStep * ndFloor(m_timeAccumulator / descreteStep) + descreteStep;
	}
	
	UWorld* const world = m_owner->GetWorld();
	while (m_timeAccumulator > descreteStep)
	{
		m_world->Update(descreteStep);
		m_world->Sync();
		m_timeAccumulator -= descreteStep;
	
		//UE_LOG(LogTemp, Display, TEXT("loop time step %f(ms)  ticks %d"), ndFloat32(microSecondStep) * 1.0e-3f, thicks1 - thicks0);
	}
}

#else

NewtonWorld::NewtonWorld(ANewtonWorldActor* const owner)
	:FRunnable()
	,m_world(nullptr)
	,m_owner(owner)
	,m_worker(nullptr)
	,m_timeAcc(0.0f)
	,m_active(true)
	,m_terminated(false)
	,m_initialized(false)
{
	m_world = new PhysicsEngine;
	m_worker = FRunnableThread::Create(this, TEXT("newtonUnrealGlue"));
	check(m_worker);

	// block until thread is initialized.
	while (!m_initialized);
}

NewtonWorld::~NewtonWorld()
{
	if (m_world)
	{
		Stop();
		m_worker->WaitForCompletion();
		delete m_worker;
		delete m_world;
	}
}

ndWorld* NewtonWorld::GetNewtonWorld() const
{
	return m_world;
}

float NewtonWorld::GetAverageUpdateTime() const
{
	return m_world->GetAverageUpdateTime();
}

bool NewtonWorld::Init()
{
	return true;
}

bool NewtonWorld::IsTerminated() const
{
	return m_terminated.load();
}

void NewtonWorld::Stop()
{
	m_active.store(false);
	while (!IsTerminated());
}

void NewtonWorld::Exit()
{
	m_terminated.store(true);
}

uint32 NewtonWorld::Run()
{
	ndFloat32 updateRate = m_owner->UpdateRate;
	const ndFloat32 timestep = (1.0f / updateRate);

	ndInt32 thicks0 = 0;
	ndInt32 thicks1 = 0;
	m_initialized = true;
	ndUnsigned64 microSeconds0 = ndGetTimeInMicroseconds();
	ndUnsigned64 deltaMicroSeconds = ndUnsigned64(1.0e6f / updateRate);
	while (m_active.load())
	{
		ndUnsigned64 microSeconds1 = ndGetTimeInMicroseconds();
		ndUnsigned64 microSecondStep = microSeconds1 - microSeconds0;
		if (microSecondStep >= deltaMicroSeconds)
		{
			ndFloat32 timeAccumulator = m_timeAcc.load();
			ndFloat32 timeMark = ndFloat32(microSecondStep) * ndFloat32(1.0e-6f);
			if (timeAccumulator > timeMark * ndFloat32(2.0f))
			{
				// truncate slow frame updates 
				UE_LOG(LogTemp, Display, TEXT("physics fell behind, throwing out a time step"));
				timeAccumulator = timeAccumulator - timestep * ndFloor(timeAccumulator / timestep) + timestep;
			}

			m_world->Update(timestep);
			Sync();

			timeAccumulator -= timestep;
			m_timeAcc.store(timeAccumulator);

			microSeconds0 += deltaMicroSeconds;
			//UE_LOG(LogTemp, Display, TEXT("loop time step %f(ms)  ticks %d"), ndFloat32 (microSecondStep) * 1.0e-3f, thicks1 - thicks0);
			thicks0 = thicks1;
		}

		thicks1++;
		FPlatformProcess::YieldThread();
	}
	return 0;
}

void NewtonWorld::Sync()
{
	m_world->Sync();
}

void NewtonWorld::ApplySettings() const
{
	m_world->Sync();
	ndWorld::ndSolverModes mode = ndWorld::ndStandardSolver;
	switch (m_owner->SolverMode)
	{
		case SolverModeTypes::scalar:
			mode = ndWorld::ndStandardSolver;
			break;
		case SolverModeTypes::soaSimd:
			mode = ndWorld::ndSimdSoaSolver;
			break;
	};

	m_world->SelectSolver(mode);
	m_world->SetSubSteps(m_owner->SolverPasses);
	m_world->SetSolverIterations(m_owner->SolverIterations);
	if (m_world->GetThreadCount() != m_owner->ParallelThreads)
	{
		check(0);
		m_world->SetThreadCount(m_owner->ParallelThreads);
	}

	const ndBodyListView& bodyList = m_world->GetBodyList();
	for (ndBodyListView::ndNode* node = bodyList.GetFirst(); node; node = node->GetNext())
	{
		ndBodyKinematic* const body = node->GetInfo()->GetAsBodyKinematic();
		body->SetAutoSleep(m_owner->AutoSleepMode);
	}
}

void NewtonWorld::StartGame()
{
	m_world->Sync();

	UWorld* const world = m_owner->GetWorld();
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
					meshComp->CreateRigidBody(m_owner, m_owner->AutoSleepMode);
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
				component->CreateJoint(m_owner);
			}
		}
	}
	m_timeAcc.store(0.0f);
}

void NewtonWorld::Intepolate() const
{
	ndFloat32 updateRate = m_owner->UpdateRate;
	ndFloat32 timeAccumulator = m_timeAcc.load();
	const ndFloat32 timestep = (1.0f / updateRate);
	ndFloat32 interpolationParam = ndClamp(timeAccumulator / timestep, ndFloat32(0.0f), ndFloat32(1.0f));

	UWorld* const world = m_owner->GetWorld();
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
					rigidBody->InterpolateTransform(interpolationParam);
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

void NewtonWorld::Update(float timestep)
{
	m_timeAcc.store(m_timeAcc.load() + timestep);
	Intepolate();
}

#endif