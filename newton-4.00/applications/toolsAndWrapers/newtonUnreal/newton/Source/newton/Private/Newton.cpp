// Copyright Epic Games, Inc. All Rights Reserved.

#include "Newton.h"
#include "EngineUtils.h"
#include "Misc/MessageDialog.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "HAL/PlatformProcess.h"
#include "Containers/Ticker.h"
#include "ToolMenus.h"
#include "Selection.h"
#include "LandscapeProxy.h"
#include "BlueprintEditorModule.h"
#include "Kismet/GameplayStatics.h"
#include "Components/LineBatchComponent.h"

#include "testButtonStyle.h"
#include "testButtonCommands.h"

#include "NewtonJoint.h"
#include "NewtonCollision.h"
#include "NewtonRigidBody.h"
#include "NewtonSceneActor.h"
#include "NewtonWorldActor.h"
#include "ThirdParty/newtonLibrary/Public/dNewton/ndNewton.h"

#define LOCTEXT_NAMESPACE "FnewtonModule"

FnewtonModule* FnewtonModule::m_pluginSingleton;

int FnewtonModule::m_currentVersion = ndPluginVersion::m_firstVersion;
const FGuid FnewtonModule::m_guiID(0x2EabFDBD, 0x0105C4D10, 0x8876F38F, 0x2516A5DA);
FCustomVersionRegistration FnewtonModule::m_guidRegistration(FnewtonModule::m_guiID, FnewtonModule::m_currentVersion, TEXT("FnewtonModule"));

class FnewtonModule::ResourceCache
{
	public:
	template<class OBJECT>
	class Cache : public ndTree<TSharedPtr<OBJECT>, long long>
	{
		public:
		Cache()
			:ndTree<TSharedPtr<OBJECT>, long long>()
		{
		}

		~Cache()
		{
			CollectGarbage();
		}

		TSharedPtr<OBJECT> Find(long long hash) const
		{
			typename Cache::ndNode* const node = ndTree<TSharedPtr<OBJECT>, long long>::Find(hash);
			return node ? node->GetInfo() : TSharedPtr<OBJECT>();
		}

		void Add(const TSharedPtr<OBJECT>& object, long long hash)
		{
			check(!Find(hash));
			Cache::Insert(object, hash);
		}

		void CollectGarbage()
		{
			typename Cache::Iterator it(*this);
			for (it.Begin(); it; )
			{
				typename Cache::ndNode* const node = it.GetNode();
				it++;
				const TSharedPtr<OBJECT>& resource = node->GetInfo();
				ndInt32 refCount = resource.GetSharedReferenceCount();
				if (refCount == 1)
				{
					Cache::Remove(node);
				}
			}
		}
	};
	
	~ResourceCache()
	{
	}

	ndShape* FindShape(long long hash) const
	{
		ndTree<ndShape*, long long>::ndNode* const node = m_shapeCache.Find(hash);
		return node ? node->GetInfo() : nullptr;
	}

	TSharedPtr<UE::Geometry::FDynamicMesh3> FindDynamicMesh(long long hash) const
	{
		return m_visualMeshCache.Find(hash);
	}

	TSharedPtr<ndConvexHullSet> FindConvexHull(long long hash) const
	{
		return m_convexVhacdCache.Find(hash);
	}

	void AddDynamicMesh(const TSharedPtr<UE::Geometry::FDynamicMesh3>& mesh, long long hash)
	{
		m_visualMeshCache.Insert(mesh, hash);
	}

	void AddShape(ndShape* const shape, long long hash)
	{
		check(!m_shapeCache.Find(hash));
		shape->AddRef();
		m_shapeCache.Insert(shape, hash);
	}

	void AddConvexHull(const TSharedPtr<ndConvexHullSet>& hull, long long hash)
	{
		m_convexVhacdCache.Insert(hull, hash);
	}

	void CollectGarbage()
	{
		ndTree<ndShape*, long long>::Iterator it(m_shapeCache);
		for (it.Begin(); it; )
		{
			ndTree<ndShape*, long long>::ndNode* const node = it.GetNode();
			it++;
			ndShape* const shape = node->GetInfo();
			if (shape->GetRefCount() == 1)
			{
				shape->Release();
				m_shapeCache.Remove(node);
			}
		}
		m_visualMeshCache.CollectGarbage();
		m_convexVhacdCache.CollectGarbage();
	}

	ndTree<ndShape*, long long> m_shapeCache;
	Cache<ndConvexHullSet> m_convexVhacdCache;
	Cache<UE::Geometry::FDynamicMesh3> m_visualMeshCache;

	ndMovingAverage<16> m_renderTime;
	ndMovingAverage<16> m_physicsTime;
};

void FnewtonModule::StartupModule()
{
	// This code will execute after your module is loaded into memory; the exact timing is specified in the .uplugin file per-module
	// Get the base directory of this plugin
	FString BaseDir = IPluginManager::Get().FindPlugin("newton")->GetBaseDir();

	// Add on the relative location of the third party dll and load it
	FString LibraryPath;
#if PLATFORM_WINDOWS
	LibraryPath = FPaths::Combine(*BaseDir, TEXT("Binaries/ThirdParty/newtonLibrary/Win64/ExampleLibrary.dll"));
#elif PLATFORM_MAC
    LibraryPath = FPaths::Combine(*BaseDir, TEXT("Source/ThirdParty/newtonLibrary/Mac/Release/libExampleLibrary.dylib"));
#elif PLATFORM_LINUX
	LibraryPath = FPaths::Combine(*BaseDir, TEXT("Binaries/ThirdParty/newtonLibrary/Linux/x86_64-unknown-linux-gnu/libExampleLibrary.so"));
#endif // PLATFORM_WINDOWS

	m_newtonLibraryHandle = !LibraryPath.IsEmpty() ? FPlatformProcess::GetDllHandle(*LibraryPath) : nullptr;

	//m_newtonWorld = nullptr;
	m_runSimulation = false;
	if (m_newtonLibraryHandle)
	{
		// Call the test function in the third party library that opens a message box
		ndMemory::SetMemoryAllocators(PhysicsAlloc, PhysicsFree);

		FTSTicker& coretiker = FTSTicker::GetCoreTicker();
		m_tickDelegate = FTickerDelegate::CreateRaw(this, &FnewtonModule::Tick);
		m_tickDelegateHandle = coretiker.AddTicker(m_tickDelegate);
	}
	else
	{
		FMessageDialog::Open(EAppMsgType::Ok, LOCTEXT("ThirdPartyLibraryError", "Failed to load example third party library"));
	}

	CreateEditorRunTimeButton();

	m_pluginSingleton = this;
	m_resourceCache = new ResourceCache;
}

void FnewtonModule::ShutdownModule()
{
	delete m_resourceCache;

	UToolMenus::UnRegisterStartupCallback(this);
	UToolMenus::UnregisterOwner(this);
	FtestButtonStyle::Shutdown();
	FtestButtonCommands::Unregister();
	
	if (m_newtonLibraryHandle)
	{
		FTSTicker::GetCoreTicker().RemoveTicker(m_tickDelegateHandle);
		FPlatformProcess::FreeDllHandle(m_newtonLibraryHandle);
	}
	m_newtonLibraryHandle = nullptr;

	m_pluginSingleton = nullptr;
}

void FnewtonModule::CreateEditorRunTimeButton()
{
	FtestButtonStyle::Initialize();
	FtestButtonStyle::ReloadTextures();
	
	FtestButtonCommands::Register();
	
	PluginCommands = MakeShareable(new FUICommandList);
	
	PluginCommands->MapAction(
		FtestButtonCommands::Get().PluginAction,
		FExecuteAction::CreateRaw(this, &FnewtonModule::ToggleEditorUpdates),
		FCanExecuteAction());

	UToolMenus::RegisterStartupCallback(FSimpleMulticastDelegate::FDelegate::CreateRaw(this, &FnewtonModule::RegisterMenus));
}

// memory allocation for Newton
void* FnewtonModule::PhysicsAlloc(size_t sizeInBytes)
{
	// I do not know how to allocate raw memory from Unreal 
	// hopefully operator new will do the right thing.
	char* ptr = new char[sizeInBytes];
	return ptr;
}

void FnewtonModule::PhysicsFree(void* ptr)
{
	delete[] ptr;
}

void FnewtonModule::ToggleEditorUpdates()
{
	m_runSimulation = !m_runSimulation;
}

FnewtonModule* FnewtonModule::GetPlugin()
{
	return m_pluginSingleton;
}

TSharedPtr<UE::Geometry::FDynamicMesh3> FnewtonModule::FindDynamicMesh(long long hash) const
{
	return m_resourceCache->FindDynamicMesh(hash);
}

void FnewtonModule::AddDynamicMesh(const TSharedPtr<UE::Geometry::FDynamicMesh3>& mesh, long long hash)
{
	m_resourceCache->AddDynamicMesh(mesh, hash);
}

ndShape* FnewtonModule::FindShape(long long hash) const
{
	return m_resourceCache->FindShape(hash);
}

void FnewtonModule::AddShape(ndShape* shape, long long hash)
{
	m_resourceCache->AddShape(shape, hash);
}

TSharedPtr<ndConvexHullSet> FnewtonModule::FindConvexHull(long long hash) const
{
	return m_resourceCache->FindConvexHull(hash);
}

void FnewtonModule::AddConvexHull(const TSharedPtr<ndConvexHullSet>& hull, long long hash)
{
	m_resourceCache->AddConvexHull(hull, hash);
}

void FnewtonModule::RegisterMenus()
{
	FToolMenuOwnerScoped OwnerScoped(this);

	{
		UToolMenu* Menu = UToolMenus::Get()->ExtendMenu("LevelEditor.MainMenu.Window");
		{
			FToolMenuSection& Section = Menu->FindOrAddSection("WindowLayout");
			Section.AddMenuEntryWithCommandList(FtestButtonCommands::Get().PluginAction, PluginCommands);
		}
	}

	{
		UToolMenu* ToolbarMenu = UToolMenus::Get()->ExtendMenu("LevelEditor.LevelEditorToolBar.PlayToolBar");
		{
			FToolMenuSection& Section = ToolbarMenu->FindOrAddSection("PluginTools");
			{
				FToolMenuEntry& Entry = Section.AddEntry(FToolMenuEntry::InitToolBarButton(FtestButtonCommands::Get().PluginAction));
				Entry.SetCommandList(PluginCommands);
			}
		}
	}
}

void FnewtonModule::DrawGizmos(const UWorld* const world, float timestep) const
{
	for (TActorIterator<AActor> actorItr(world); actorItr; ++actorItr)
	{
		AActor* const actor = *actorItr;
		if (actor->FindComponentByClass(UNewtonRigidBody::StaticClass()))
		{
			const TSet<UActorComponent*>& components = actor->GetComponents();
			for (TSet<UActorComponent*>::TConstIterator it(components.CreateConstIterator()); it; ++it)
			{
				UNewtonRigidBody* const body = Cast<UNewtonRigidBody>(*it);
				if (body)
				{
					body->DrawGizmo(timestep);
				}

				const UNewtonJoint* const joint = Cast<UNewtonJoint>(*it);
				if (joint)
				{
					joint->DrawGizmo(timestep);
				}
			}
		}
	}
}

void FnewtonModule::CleanupDebugLines(const UWorld* const world, float timestep) const
{
	ULineBatchComponent* const debugLines = world->PersistentLineBatcher.Get();
	{
		ndInt32 lineCount = debugLines->BatchedLines.Num();
		for (ndInt32 i = lineCount - 1; i >= 0; --i)
		{
			FBatchedLine& line = debugLines->BatchedLines[i];
			if (line.RemainingLifeTime >= 0.0f)
			{
				line.RemainingLifeTime -= timestep;
				if (line.RemainingLifeTime < 0.0f)
				{
					lineCount--;
					debugLines->BatchedLines[i] = debugLines->BatchedLines[lineCount];
				}
			}
		}
		if (lineCount != debugLines->BatchedLines.Num())
		{
			debugLines->BatchedLines.SetNum(lineCount);
			debugLines->MarkRenderStateDirty();
		}
	}

	{
		ndInt32 pointCount = debugLines->BatchedPoints.Num();
		for (ndInt32 i = pointCount - 1; i >= 0; --i)
		{
			FBatchedPoint& point = debugLines->BatchedPoints[i];
			if (point.RemainingLifeTime >= 0.0f)
			{
				point.RemainingLifeTime -= timestep;
				if (point.RemainingLifeTime < 0.0f)
				{
					pointCount--;
					debugLines->BatchedPoints[i] = debugLines->BatchedPoints[pointCount];
				}
			}
		}
		if (pointCount != debugLines->BatchedPoints.Num())
		{
			debugLines->BatchedPoints.SetNum(pointCount);
			debugLines->MarkRenderStateDirty();
		}
	}

	{
		ndInt32 meshCount = debugLines->BatchedMeshes.Num();
		for (ndInt32 i = meshCount - 1; i >= 0; --i)
		{
			FBatchedMesh& mesh = debugLines->BatchedMeshes[i];
			if (mesh.RemainingLifeTime >= 0.0f)
			{
				mesh.RemainingLifeTime -= timestep;
				if (mesh.RemainingLifeTime < 0.0f)
				{
					meshCount--;
					debugLines->BatchedMeshes[i] = debugLines->BatchedMeshes[meshCount];
				}
			}
		}
		if (meshCount != debugLines->BatchedMeshes.Num())
		{
			debugLines->BatchedMeshes.SetNum(meshCount);
			debugLines->MarkRenderStateDirty();
		}
	}
}

void FnewtonModule::UpdatePropertyChanges(const UWorld* const world) const
{
	for (TActorIterator<AActor> actorItr(world); actorItr; ++actorItr)
	{
		AActor* const actor = *actorItr;
		if (actor->FindComponentByClass(UNewtonRigidBody::StaticClass()))
		{
			bool propertyChange = false;
			ANewtonSceneActor* const sceneActor = Cast<ANewtonSceneActor>(actor);
			if (sceneActor)
			{
				propertyChange = propertyChange || sceneActor->m_propertyChanged;
			}

			const TSet<UActorComponent*>& components = actor->GetComponents();
			for (TSet<UActorComponent*>::TConstIterator it(components.CreateConstIterator()); it; ++it)
			{
				UNewtonCollision* const shape = Cast<UNewtonCollision>(*it);
				if (shape)
				{
					propertyChange = propertyChange || shape->m_propertyChanged;
				}

				UNewtonRigidBody* const body = Cast<UNewtonRigidBody>(*it);
				if (body)
				{
					propertyChange = propertyChange || body->m_propertyChanged;
				}

				UNewtonJoint* const joint = Cast<UNewtonJoint>(*it);
				if (joint)
				{
					propertyChange = propertyChange || joint->m_propertyChanged;
				}
			}

			if (propertyChange)
			{
				if (sceneActor)
				{
					sceneActor->ApplyPropertyChanges();
				}

				for (TSet<UActorComponent*>::TConstIterator it(components.CreateConstIterator()); it; ++it)
				{
					UNewtonCollision* const shape = Cast<UNewtonCollision>(*it);
					if (shape)
					{
						shape->ApplyPropertyChanges();
					}
				}

				for (TSet<UActorComponent*>::TConstIterator it(components.CreateConstIterator()); it; ++it)
				{
					UNewtonRigidBody* const body = Cast<UNewtonRigidBody>(*it);
					if (body)
					{
						body->ApplyPropertyChanges();
					}
				}

				for (TSet<UActorComponent*>::TConstIterator it(components.CreateConstIterator()); it; ++it)
				{
					UNewtonJoint* const joint = Cast<UNewtonJoint>(*it);
					if (joint)
					{
						joint->ApplyPropertyChanges();
					}
				}
			}
		}
	}
}

void FnewtonModule::InitNewtonWorld(ANewtonWorldActor* const newtonWorld) const
{
	UWorld* const world = newtonWorld->GetWorld();
	for (TActorIterator<AActor> actorItr(world); actorItr; ++actorItr)
	{
		AActor* const actor = *actorItr;
		if (actor->FindComponentByClass(UNewtonRigidBody::StaticClass()))
		{
			const TSet<UActorComponent*>& components = actor->GetComponents();

			for (TSet<UActorComponent*>::TConstIterator it(components.CreateConstIterator()); it; ++it)
			{
				UNewtonCollision* const shape = Cast<UNewtonCollision>(*it);
				if (shape)
				{
					shape->ApplyPropertyChanges();
				}
			}

			for (TSet<UActorComponent*>::TConstIterator it(components.CreateConstIterator()); it; ++it)
			{
				UNewtonRigidBody* const body = Cast<UNewtonRigidBody>(*it);
				if (body)
				{
					body->ApplyPropertyChanges();
				}
			}

			for (TSet<UActorComponent*>::TConstIterator it(components.CreateConstIterator()); it; ++it)
			{
				UNewtonJoint* const joint = Cast<UNewtonJoint>(*it);
				if (joint)
				{
					joint->ApplyPropertyChanges();
				}
			}
		}
	}
	newtonWorld->StartGame();
}

bool FnewtonModule::Tick(float timestep)
{
	//if (! (GEngine && m_runSimulation))
	if (!GEngine)
	{
		return true;
	}

	auto FindNewtonWorldActor = [this]()
	{
		const TIndirectArray<FWorldContext>& contexts = GEngine->GetWorldContexts();
		for (ndInt32 i = contexts.Num() - 1; i >= 0; --i)
		{
			const FWorldContext& context = contexts[i];
			if (context.WorldType == EWorldType::PIE)
			{
				ANewtonWorldActor* const world = Cast<ANewtonWorldActor>(UGameplayStatics::GetActorOfClass(contexts[i].World(), ANewtonWorldActor::StaticClass()));
				return (world && world->m_world) ? world : (ANewtonWorldActor*)nullptr;
			}
		}
		return (ANewtonWorldActor*)nullptr;
	};

	ANewtonWorldActor* const newtonWorld = FindNewtonWorldActor();
	if (newtonWorld)
	{
		char tmp0[256];
		char tmp1[256];
		float simTime = newtonWorld->GetSimTime();
		m_resourceCache->m_physicsTime.Update(simTime * 1.0e3f);
		m_resourceCache->m_renderTime.Update(timestep * 1.0e3f);
		snprintf(tmp1, sizeof(tmp1), "physicsTime: %g(ms)", m_resourceCache->m_physicsTime.GetAverage());
		snprintf(tmp0, sizeof(tmp0), "renderTime:  %g(ms)", m_resourceCache->m_renderTime.GetAverage());

		FString msg0(tmp0);
		FString msg1(tmp1);
		GEngine->AddOnScreenDebugMessage(INDEX_NONE, 0.0f, FColor::Yellow, msg1);
		GEngine->AddOnScreenDebugMessage(INDEX_NONE, 0.0f, FColor::Yellow, msg0);

		CleanupDebugLines(newtonWorld->GetWorld(), timestep);
		if (newtonWorld->m_beginPlay)
		{
			InitNewtonWorld(newtonWorld);
		}
		newtonWorld->Update(timestep);
		m_resourceCache->CollectGarbage();
	}
	else
	{
		#if WITH_EDITOR
		const TIndirectArray<FWorldContext>& contexts = GEngine->GetWorldContexts();

		//float persitentTimestep = timestep * 1.5f;
		float persitentTimestep = ndClamp(timestep * 1.5f, 0.0f, 1.0f);
		for (ndInt32 i = contexts.Num() - 1; i >= 0; --i)
		{
			const FWorldContext& context = contexts[i];
			if (context.WorldType != EWorldType::PIE)
			{
				UWorld* const world = context.World();

				UpdatePropertyChanges(world);
				CleanupDebugLines(world, timestep);
				DrawGizmos(world, persitentTimestep);
			}
		}
		#endif
	}
	
	return true;
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FnewtonModule, newton)
