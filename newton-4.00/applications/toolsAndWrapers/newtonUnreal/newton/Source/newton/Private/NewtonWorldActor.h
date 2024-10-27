// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "NewtonWorldActor.generated.h"

class ndWorld;
class NewtonWorld;
class UDynamicMesh;

UENUM()
enum class SolverModeTypes : uint8
{
	scalar,
	soaSimd,
};

UCLASS( ClassGroup = NewtonActors, meta=(BlueprintSpawnableComponent) )
class ANewtonWorldActor : public AActor
{
	GENERATED_BODY()

	public:	
	// Sets default values for this actor's properties
	ANewtonWorldActor();

	ndWorld* GetNewtonWorld() const;

	// Called when the game starts or when spawned
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;
	
	void ApplySettings();
	float GetSimTime() const;
	void Update(float timestep);

	UPROPERTY(EditAnywhere, Category=Newton)
	float UpdateRate;

	UPROPERTY(EditAnywhere, Category=Newton, meta = (ClampMin = 2, ClampMax = 8))
	int SolverPasses;

	UPROPERTY(EditAnywhere, Category=Newton, meta = (ClampMin = 4, ClampMax = 32))
	int SolverIterations;

	UPROPERTY(EditAnywhere, Category=Newton, meta = (ClampMin = 1, ClampMax = 32))
	int ParallelThreads;

	UPROPERTY(EditAnywhere, Category = Newton, meta=(ValidEnumValues="scalar, soaSimd"))
	SolverModeTypes SolverMode;

	UPROPERTY(EditAnywhere, Category = Newton)
	bool AutoSleepMode;

	UPROPERTY(EditAnywhere, Category=Newton)
	bool ClearDebug;

	UPROPERTY(EditAnywhere, Category=Newton)
	bool ShowDebug;

	private:
	void Cleanup();
	void StartGame();

	NewtonWorld* m_world;
	bool m_beginPlay;

	friend class FnewtonModule;
	friend class UNewtonCollision;
};
