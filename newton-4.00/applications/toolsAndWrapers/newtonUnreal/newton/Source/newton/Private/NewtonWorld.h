// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "HAL/Runnable.h"
#include "atomic"

class ndWorld;
class ANewtonWorldActor;

#define D_USING_UNREAL_TRHEADS

#ifndef D_USING_UNREAL_TRHEADS

class NewtonWorld
{
	class PhysicsEngine;
	public:
	NewtonWorld(ANewtonWorldActor* const owner);
	virtual ~NewtonWorld();

	void Sync();
	void ApplySettings() const;
	ndWorld* GetNewtonWorld() const;
	float GetAverageUpdateTime() const;

	void StartGame();
	void Update(float timestep);

	private:
	void VisualTick();
	void PhysicsTick();

	PhysicsEngine* m_world;
	ANewtonWorldActor* m_owner;
	float m_timeAccumulator;
};

#else

class NewtonWorld: public FRunnable
{
	class PhysicsEngine;
	public:
	class EngineMainThread;

	NewtonWorld(ANewtonWorldActor* const owner);
	~NewtonWorld();

	virtual bool Init() override;
	virtual void Stop() override;
	virtual void Exit() override;
	virtual uint32 Run() override;

	void Sync();
	void ApplySettings() const;
	ndWorld* GetNewtonWorld() const;
	float GetAverageUpdateTime() const;

	void StartGame();
	void Update(float timestep);

	private:
	void Intepolate() const;
	bool IsTerminated() const;

	PhysicsEngine* m_world;
	ANewtonWorldActor* m_owner;
	FRunnableThread* m_worker;
	std::atomic<float> m_timeAcc;
	std::atomic<bool> m_active;
	std::atomic<bool> m_terminated;
	bool m_initialized;
};

#endif