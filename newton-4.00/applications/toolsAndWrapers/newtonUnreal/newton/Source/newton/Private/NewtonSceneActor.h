// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "NewtonSceneActor.generated.h"

class ALandscapeProxy;

UCLASS(ClassGroup = NewtonScene, meta=(BlueprintSpawnableComponent), HideCategories = (Physics, Collision), MinimalAPI)
class ANewtonSceneActor : public AActor
{
	GENERATED_BODY()

	public:	
	// Sets default values for this actor's properties
	ANewtonSceneActor();

	virtual void PostLoad() override;
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;

	void ApplyPropertyChanges();

	protected:
	UPROPERTY(EditAnywhere, Category = Newton)
	bool GenerateShapes;

	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = NewtonScene, meta = (AllowPrivateAccess = "true"))
	TObjectPtr<class UNewtonSceneRigidBody> RootBody;

	private:
	void GenerateStaticMeshCollision(const AActor* const actor);
	void GenerateLandScapeCollision(const ALandscapeProxy* const landscape);
	void CreateCollisionFromUnrealPrimitive(TObjectPtr<UStaticMeshComponent> staticComponent);

	bool m_propertyChanged;
	friend class FnewtonModule;
};
