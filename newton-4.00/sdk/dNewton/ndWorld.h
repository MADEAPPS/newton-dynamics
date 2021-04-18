/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef __D_WORLD_H__
#define __D_WORLD_H__

#include "ndNewtonStdafx.h"
#include "ndJointList.h"
#include "ndModelList.h"
#include "ndSkeletonList.h"
#include "ndBodyParticleSetList.h"

class ndWorld;
class ndModel;
class ndBodyDynamic;
class ndDynamicsUpdate;
class ndJointBilateralConstraint;

#define D_NEWTON_ENGINE_MAJOR_VERSION 4
#define D_NEWTON_ENGINE_MINOR_VERSION 00

#define D_SLEEP_ENTRIES			8


D_MSV_NEWTON_ALIGN_32
class ndWorld: public dClassAlloc
{
	public:
	enum ndSolverModes
	{	
		ndSimdSoaSolver,
		ndSimdAvx2Solver,
		ndOpenclSolver,
		ndStandardSolver,
	};

	D_NEWTON_API ndWorld();
	D_NEWTON_API virtual ~ndWorld();

	dInt32 GetEngineVersion() 
	{
		return D_NEWTON_ENGINE_MAJOR_VERSION * 100 + D_NEWTON_ENGINE_MINOR_VERSION;
	}

	void Sync() const;
	void Update(dFloat32 timestep);

	virtual void OnPostUpdate(dFloat32 timestep);

	dInt32 GetThreadCount() const;
	void SetThreadCount(dInt32 count);

	dInt32 GetSubSteps() const;
	void SetSubSteps(dInt32 subSteps);

	ndSolverModes GetSelectedSolver() const;
	D_NEWTON_API void SelectSolver(ndSolverModes solverMode);
	D_NEWTON_API const char* GetSolverString() const;

	D_NEWTON_API virtual bool AddBody(ndBody* const body);
	D_NEWTON_API virtual void RemoveBody(ndBody* const body);
	D_NEWTON_API virtual void DeleteBody(ndBody* const body);

	D_NEWTON_API virtual void AddJoint(ndJointBilateralConstraint* const joint);
	D_NEWTON_API virtual void RemoveJoint(ndJointBilateralConstraint* const joint);

	D_NEWTON_API virtual void AddModel(ndModel* const model);
	D_NEWTON_API virtual void RemoveModel(ndModel* const model);

	D_NEWTON_API void Load(const char* const path);
	D_NEWTON_API void Load(const nd::TiXmlElement* const rootNode, const char* const assetPath);
	D_NEWTON_API virtual ndBody* LoadUserDefinedBody(const nd::TiXmlNode* const parentNode, const char* const bodyClassName, dTree<const ndShape*, dUnsigned32>& shapesCache, const char* const assetPath) const;

	D_NEWTON_API void Save(const char* const path) const;
	D_NEWTON_API void Save(nd::TiXmlElement* const rootNode, const char* const assetPath) const;

	const ndBodyList& GetBodyList() const;
	const ndJointList& GetJointList() const;
	const ndModelList& GetModelList() const;
	const ndContactList& GetContactList() const;
	const ndSkeletonList& GetSkeletonList() const;
	const ndBodyParticleSetList& GetParticleList() const;

	ndBodyDynamic* GetSentinelBody() const;

	dInt32 GetSolverIterations() const;
	void SetSolverIterations(dInt32 iterations);

	ndScene* GetScene() const;

	dFloat32 GetUpdateTime() const;
	dUnsigned32 GetFrameIndex() const;
	dFloat32 GetAverageUpdateTime() const;

	ndContactNotify* GetContactNotify() const;
	void SetContactNotify(ndContactNotify* const notify);

	void DebugScene(ndSceneTreeNotiFy* const notify);

	D_NEWTON_API void ClearCache();

	private:
	void ThreadFunction();
	void PostUpdate(dFloat32 timestep);
	
	protected:
	D_NEWTON_API virtual void UpdateSkeletons();
	D_NEWTON_API virtual void UpdateTransforms();
	D_NEWTON_API virtual void ApplyExternalForces();

	private:
	class dgSolverProgressiveSleepEntry
	{
		public:
		dFloat32 m_maxAccel;
		dFloat32 m_maxAlpha;
		dFloat32 m_maxVeloc;
		dFloat32 m_maxOmega;
		dInt32 m_steps;
	};

	class ndIslandMember
	{
		public:
		ndBodyKinematic* m_root;
		ndBodyKinematic* m_body;
	};

	void ModelUpdate();
	void ModelPostUpdate();
	void ParticleUpdate();
	void CalculateAverageUpdateTime();
	void SubStepUpdate(dFloat32 timestep);
	void LoadSettings(const nd::TiXmlNode* const rootNode);
	void LoadBodies(const nd::TiXmlNode* const rootNode, dTree<const ndShape*, dUnsigned32>& shapesCache, const char* const assetPath);
	void LoadShapes(const nd::TiXmlNode* const rootNode, dTree<const ndShape*, dUnsigned32>& shapesCache, const char* const assetPath);

	bool SkeletonJointTest(ndJointBilateralConstraint* const jointA) const;
	static dInt32 CompareJointByInvMass(const ndJointBilateralConstraint* const jointA, const ndJointBilateralConstraint* const jointB, void* notUsed);

	ndScene* m_scene;
	ndBodyDynamic* m_sentinelBody;
	ndDynamicsUpdate* m_solver;
	ndJointList m_jointList;
	ndModelList m_modelList;
	ndSkeletonList m_skeletonList;
	ndBodyParticleSetList m_particleSetList;

	dFloat32 m_timestep;
	dFloat32 m_freezeAccel2;
	dFloat32 m_freezeAlpha2;
	dFloat32 m_freezeSpeed2;
	dFloat32 m_freezeOmega2;
	dFloat32 m_averageUpdateTime;
	dFloat32 m_averageTimestepAcc;
	dFloat32 m_averageFramesCount;
	dFloat32 m_lastExecutionTime;

	dgSolverProgressiveSleepEntry m_sleepTable[D_SLEEP_ENTRIES];

	dInt32 m_subSteps;
	ndSolverModes m_solverMode;
	dInt32 m_solverIterations;
	dUnsigned32 m_frameIndex;
	bool m_inUpdate;
	bool m_collisionUpdate;
	

	friend class ndScene;
	friend class ndDynamicsUpdate;
	friend class ndWorldMixedScene;
	friend class ndDynamicsUpdateSoa;
	friend class ndDynamicsUpdateAvx2;
	friend class ndDynamicsUpdateOpencl;
	friend class ndWorldSegregatedScene;
} D_GCC_NEWTON_ALIGN_32;

inline void ndWorld::Sync() const
{
	m_scene->Sync();
}

inline dInt32 ndWorld::GetThreadCount() const
{
	return m_scene->GetThreadCount();
}

inline void ndWorld::SetThreadCount(dInt32 count)
{
	m_scene->SetCount(count);
}

inline dInt32 ndWorld::GetSubSteps() const
{
	return m_subSteps;
}

inline void ndWorld::SetSubSteps(dInt32 subSteps)
{
	m_subSteps = dClamp(subSteps, 1, 16);
}

inline ndScene* ndWorld::GetScene() const
{
	return m_scene;
}

inline dInt32 ndWorld::GetSolverIterations() const
{
	return m_solverIterations;
}

inline void ndWorld::SetSolverIterations(dInt32 iterations)
{
	m_solverIterations = dUnsigned32(dMax(4, iterations));
}

inline ndContactNotify* ndWorld::GetContactNotify() const
{
	return m_scene->GetContactNotify();
}

inline void ndWorld::SetContactNotify(ndContactNotify* const notify)
{
	m_scene->SetContactNotify(notify);
}


inline ndBodyDynamic* ndWorld::GetSentinelBody() const
{
	return m_sentinelBody;
}

inline const ndBodyList& ndWorld::GetBodyList() const
{
	return m_scene->GetBodyList();
}

inline const ndJointList& ndWorld::GetJointList() const
{
	return m_jointList;
}

inline const ndContactList& ndWorld::GetContactList() const
{
	return m_scene->GetContactList();
}

inline const ndSkeletonList& ndWorld::GetSkeletonList() const
{
	return m_skeletonList;
}

inline const ndBodyParticleSetList& ndWorld::GetParticleList() const
{
	return m_particleSetList;
}

inline const ndModelList& ndWorld::GetModelList() const
{
	return m_modelList;
}

inline dFloat32 ndWorld::GetUpdateTime() const
{
	return m_lastExecutionTime;
}

inline dFloat32 ndWorld::GetAverageUpdateTime() const
{
	return m_averageUpdateTime;
}

inline dUnsigned32 ndWorld::GetFrameIndex() const
{
	return m_frameIndex;
}

inline void ndWorld::OnPostUpdate(dFloat32)
{
}

inline void ndWorld::DebugScene(ndSceneTreeNotiFy* const notify)
{
	m_scene->DebugScene(notify);
}

inline void ndWorld::Update(dFloat32 timestep)
{
	// wait until previous update complete.
	Sync();

	// save time state for use by the update callback
	m_timestep = timestep;
	m_collisionUpdate = false;

	// update the next frame asynchronous 
	m_scene->TickOne();
}

inline ndWorld::ndSolverModes ndWorld::GetSelectedSolver() const
{
	return m_solverMode;
}

#endif
