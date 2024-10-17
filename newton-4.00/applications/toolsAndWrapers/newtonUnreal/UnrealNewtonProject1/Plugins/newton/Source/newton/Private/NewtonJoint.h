// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "UObject/ObjectMacros.h"
#include "Components/SceneComponent.h"
#include "NewtonJoint.generated.h"

class ndWorld;
class ndBodyKinematic;
class UNewtonRigidBody;
class ANewtonWorldActor;
class ndJointBilateralConstraint;

//UCLASS(ClassGroup=(NewtonJoints), meta=(BlueprintSpawnableComponent), HideCategories = (RayTracing, Mobile, TextureStreaming, VirtualTexture, MaterialParameters, DynamicMeshComponent, Advanced, Activation, Collision, Lighting, BodySetup, Primitives, HLOD, ComponentTick, Rendering, Physics, Tags, Replication, ComponentReplication, Cooking, Events, LOD, Navigation, AssetUserData), MinimalAPI)
UCLASS(Abstract, meta=(BlueprintSpawnableComponent), HideCategories = (RayTracing, Mobile, TextureStreaming, VirtualTexture, MaterialParameters, DynamicMeshComponent, Advanced, Activation, Collision, Lighting, BodySetup, Primitives, HLOD, ComponentTick, Rendering, Physics, Tags, Replication, ComponentReplication, Cooking, Events, LOD, Navigation, AssetUserData), MinimalAPI)
class UNewtonJoint : public USceneComponent
{
	GENERATED_BODY()

	public:	
	// Sets default values for this component's properties
	UNewtonJoint();

	void ClearDebug();
	// Called every frame
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;

	void DestroyJoint();
	void UpdateTransform();
	virtual void DrawGizmo(float timestep) const;
	virtual void CreateJoint(ANewtonWorldActor* const worldActor);

	UPROPERTY(EditAnywhere, Category = Newton)
	bool ShowDebug;

	UPROPERTY(EditAnywhere, Category = Newton, meta = (ClampMin = 0.1f, ClampMax = 4.0f))
	float DebugScale;

	///** Pointer to second Actor to constrain. */
	//UPROPERTY(EditAnywhere, Category = Newton)
	//FName LoopNewtonStaticMesh0;
	//
	///** Pointer to second Actor to constrain. */
	//UPROPERTY(EditAnywhere, Category = Newton)
	//FName LoopNewtonStaticMesh1;

	protected:
	UNewtonRigidBody* FindChild() const;
	UNewtonRigidBody* FindParent() const;
	void GetBodyPairs(ndWorld* const world, ndBodyKinematic** body0, ndBodyKinematic** body1) const;

	void ApplyPropertyChanges();
	virtual void PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent) override;

	FTransform m_transfrom;
	ndJointBilateralConstraint* m_joint;
	FTransform m_localChildTransfrom;
	FTransform m_localParentTransfrom;
	bool m_propertyChanged;

	friend class FnewtonModule;
};
