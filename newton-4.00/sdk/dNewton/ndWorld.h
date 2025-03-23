/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __ND_WORLD_H__
#define __ND_WORLD_H__

#include "ndNewtonStdafx.h"
#include "ndJointList.h"
#include "ndSkeletonList.h"
#include "dModels/ndModelList.h"

class ndWorld;
class ndModel;
class ndJointList;
class ndBodyDynamic;
class ndRayCastNotify;
class ndDynamicsUpdate;
class ndConvexCastNotify;
class ndBodiesInAabbNotify;
class ndJointBilateralConstraint;

#define D_NEWTON_ENGINE_MAJOR_VERSION 4
#define D_NEWTON_ENGINE_MINOR_VERSION 00

#define D_SLEEP_ENTRIES			8

D_MSV_NEWTON_CLASS_ALIGN_32
class ndWorld: public ndClassAlloc
{
	public:
	enum ndSolverModes
	{	
		ndStandardSolver,
		ndSimdSoaSolver,
		ndSimdAvx2Solver,
		ndCudaSolver,
	};

	D_BASE_CLASS_REFLECTION(ndWorld)
	D_NEWTON_API ndWorld();
	D_NEWTON_API virtual ~ndWorld();

	D_NEWTON_API virtual void CleanUp();
	D_NEWTON_API bool ValidateScene() const;

	D_NEWTON_API ndInt32 GetEngineVersion() const;

	D_NEWTON_API void Sync() const;
	D_NEWTON_API void Update(ndFloat32 timestep);
	D_NEWTON_API void CollisionUpdate(ndFloat32 timestep);

	D_NEWTON_API ndInt32 GetThreadCount() const;
	D_NEWTON_API void SetThreadCount(ndInt32 count);

	D_NEWTON_API ndInt32 GetSubSteps() const;
	D_NEWTON_API void SetSubSteps(ndInt32 subSteps);

	D_NEWTON_API ndSolverModes GetSelectedSolver() const;
	D_NEWTON_API void SelectSolver(ndSolverModes solverMode);

	D_NEWTON_API ndScene* GetScene() const;
	D_NEWTON_API bool IsHighPerformanceCompute() const;
	D_NEWTON_API const char* GetSolverString() const;
	D_NEWTON_API ndBodyKinematic* GetSentinelBody() const;

	D_NEWTON_API virtual bool AddBody(const ndSharedPtr<ndBody>& body);
	D_NEWTON_API virtual void AddModel(const ndSharedPtr<ndModel>& model);
	D_NEWTON_API virtual void AddJoint(const ndSharedPtr<ndJointBilateralConstraint>& joint);

	D_NEWTON_API ndSharedPtr<ndBody> GetBody(ndBody* const body) const;
	//D_NEWTON_API virtual void GetModel(ndSharedPtr<ndModel>& model);
	//D_NEWTON_API virtual void GetJoint(ndSharedPtr<ndJointBilateralConstraint>& joint);

	D_NEWTON_API virtual void RemoveBody(ndBody* const body);
	D_NEWTON_API virtual void RemoveModel(ndModel* const model);
	D_NEWTON_API virtual void RemoveJoint(ndJointBilateralConstraint* const joint);

	D_NEWTON_API const ndJointList& GetJointList() const;
	D_NEWTON_API const ndModelList& GetModelList() const;
	D_NEWTON_API const ndBodyListView& GetBodyList() const;
	D_NEWTON_API const ndBodyList& GetParticleList() const;
	D_NEWTON_API const ndContactArray& GetContactList() const;
	D_NEWTON_API const ndSkeletonList& GetSkeletonList() const;

	D_NEWTON_API ndInt32 GetSolverIterations() const;
	D_NEWTON_API void SetSolverIterations(ndInt32 iterations);
	
	D_NEWTON_API ndFloat32 GetUpdateTime() const;
	D_NEWTON_API ndUnsigned32 GetFrameNumber() const;
	D_NEWTON_API ndUnsigned32 GetSubFrameNumber() const;
	D_NEWTON_API ndFloat32 GetAverageUpdateTime() const;

	D_NEWTON_API ndContactNotify* GetContactNotify() const;
	D_NEWTON_API void SetContactNotify(ndContactNotify* const notify);

	D_NEWTON_API void DebugScene(ndSceneTreeNotiFy* const notify);
	D_NEWTON_API void SendBackgroundTask(ndBackgroundTask* const job);

	D_NEWTON_API void ClearCache();
	D_NEWTON_API void BodiesInAabb(ndBodiesInAabbNotify& callback, const ndVector& minBox, const ndVector& maxBox) const;
	D_NEWTON_API bool RayCast(ndRayCastNotify& callback, const ndVector& globalOrigin, const ndVector& globalDest) const;
	D_NEWTON_API bool ConvexCast(ndConvexCastNotify& callback, const ndShapeInstance& convexShape, const ndMatrix& globalOrigin, const ndVector& globalDest) const;

	D_NEWTON_API void CalculateJointContacts(ndContact* const contact);

	D_NEWTON_API virtual void MainUpdate();
	D_NEWTON_API virtual void WorkerUpdate(ndInt32 threadIndex);

	private:
	void ThreadFunction();
	
	protected:
	D_NEWTON_API virtual void UpdateSkeletons();
	D_NEWTON_API virtual void UpdateTransforms();
	D_NEWTON_API virtual void PostModelTransform();

	D_NEWTON_API virtual void PreUpdate(ndFloat32 timestep);
	D_NEWTON_API virtual void PostUpdate(ndFloat32 timestep);

	D_NEWTON_API virtual void OnSubStepPreUpdate(ndFloat32 timestep);
	D_NEWTON_API virtual void OnSubStepPostUpdate(ndFloat32 timestep);

	private:
	//void RemoveBody(ndSharedPtr<ndBody>& body);
	//void RemoveJoint(ndSharedPtr<ndJointBilateralConstraint>& joint);

	class dgSolverProgressiveSleepEntry
	{
		public:
		ndFloat32 m_maxAccel;
		ndFloat32 m_maxVeloc;
		ndInt32 m_steps;
	};

	class ndIslandMember
	{
		public:
		ndBodyKinematic* m_root;
		ndBodyKinematic* m_body;
	};

	void ModelUpdate();
	void ModelPostUpdate();
	void CalculateAverageUpdateTime();
	void SubStepUpdate(ndFloat32 timestep);
	void ParticleUpdate(ndFloat32 timestep);

	bool SkeletonJointTest(ndJointBilateralConstraint* const jointA) const;
	static ndInt32 CompareJointByInvMass(const ndJointBilateralConstraint* const jointA, const ndJointBilateralConstraint* const jointB, void* notUsed);

	ndScene* m_scene;
	ndDynamicsUpdate* m_solver;
	ndJointList m_jointList;
	ndModelList m_modelList;
	ndSkeletonList m_skeletonList;
	ndArray<ndSkeletonContainer*> m_activeSkeletons;
	ndSpinLock m_addRemoveBodiesLock;
	ndSpinLock m_addRemoveJointsLock;
	ndSpinLock m_addRemoveModelsLock;

	ndFloat32 m_timestep;
	ndFloat32 m_freezeAccel2;
	ndFloat32 m_freezeSpeed2;
	ndFloat32 m_averageUpdateTime;
	ndFloat32 m_averageTimestepAcc;
	ndFloat32 m_averageFramesCount;
	ndFloat32 m_lastExecutionTime;
	dgSolverProgressiveSleepEntry m_sleepTable[D_SLEEP_ENTRIES];

	ndInt32 m_subSteps;
	ndSolverModes m_solverMode;
	ndInt32 m_solverIterations;
	bool m_inUpdate;
	
	friend class ndScene;
	friend class ndIkSolver;
	friend class ndWorldScene;
	friend class ndBodyDynamic;
	friend class ndDynamicsUpdate;
	friend class ndSkeletonContainer;
	friend class ndModelArticulation;
	friend class ndDynamicsUpdateSoa;
	friend class ndDynamicsUpdateAvx2;
	friend class ndDynamicsUpdateCuda;
} D_GCC_NEWTON_CLASS_ALIGN_32;

#endif
