// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "Modules/ModuleManager.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "Interfaces/IPluginManager.h"
#include "NewtonCommons.h"

class FnewtonModule : public IModuleInterface
{
	class ResourceCache;
	public:

	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

	static FnewtonModule* GetPlugin();

	ndShape* FindShape(long long hash) const;
	TSharedPtr<ndConvexHullSet> FindConvexHull(long long hash) const;
	TSharedPtr<UE::Geometry::FDynamicMesh3> FindDynamicMesh (long long hash) const;

	void AddShape(ndShape* const shape, long long hash);
	void AddConvexHull(const TSharedPtr<ndConvexHullSet>& hull, long long hash);
	void AddDynamicMesh(const TSharedPtr<UE::Geometry::FDynamicMesh3>& mesh, long long hash);

	private:
	bool Tick(float timestep);
	void RegisterMenus();
	void ToggleEditorUpdates();

	void CreateEditorRunTimeButton();
	static void PhysicsFree(void* ptr);
	static void* PhysicsAlloc(size_t sizeInBytes);

	void UpdatePropertyChanges(const UWorld* const world) const;
	void InitNewtonWorld(ANewtonWorldActor* const newtonWorld) const;
	void DrawGizmos(const UWorld* const world, float timestep) const;
	void CleanupDebugLines(const UWorld* const world, float timestep) const;

	/** Handle to the test dll we will load */
	void* m_newtonLibraryHandle;
	FTickerDelegate m_tickDelegate;
	FTSTicker::FDelegateHandle m_tickDelegateHandle;

	// menu bar button
	TSharedPtr<class FUICommandList> PluginCommands;
	ResourceCache* m_resourceCache;
	bool m_runSimulation;
	static FnewtonModule* m_pluginSingleton;

	public:
	const static FGuid m_guiID;
	static int m_currentVersion;
	static FCustomVersionRegistration m_guidRegistration;
};
