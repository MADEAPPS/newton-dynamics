/* 
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

#ifndef _D_NEWTON_WORLD_H_
#define _D_NEWTON_WORLD_H_

#include "stdafx.h"
#include "dAlloc.h"

class NewtonWorld;
class dNewtonBody;
class dNewtonCollision;
class dNewtonCollisionBox;
class dNewtonVehicleManager;


typedef void(*OnWorldBodyTransfromUpdateCallback)();
typedef void(*OnWorldUpdateCallback)(dFloat timestep);

class rayHitInfo
{
public:
	float intersectParam;
	int layermask;
	void* managedBodyHandle;
	const NewtonCollision* collider;
	float position[3];
	float normal[3];
	dLong collisionID;

	rayHitInfo()
	{
		clearData();
	}

	void clearData()
	{
		intersectParam = 2.0f;
		managedBodyHandle = nullptr;
		collider = nullptr;
		position[0] = 0;
		position[1] = 0;
		position[2] = 0;
		normal[0] = 0;
		normal[1] = 0;
		normal[2] = 0;
		collisionID = 0;
	}

};

class dNewtonWorld: public dAlloc
{
	public:
	class dMaterialProperties
	{
		public:
		float m_restitution;
		float m_staticFriction;
		float m_kineticFriction;
		bool m_collisionEnable;
	};

	dNewtonWorld();
	virtual ~dNewtonWorld();
	void Update(dFloat timestepInSeconds);

	void SetSolverMode(int mode);
	void SetFrameRate(dFloat frameRate);
	const dVector& GetGravity() const;
	void SetGravity(const dVector& gravity);
	void SetGravity(dFloat x, dFloat y, dFloat z);
	void SetAsyncUpdate(bool updateMode);
	void SetThreadsCount(int threads);
	void SetBroadPhase(int broadphase);
	void SetSubSteps(int subSteps);

	long long GetMaterialKey(int materialID0, int materialID1) const;
	void SetDefaultMaterial(float restitution, float staticFriction, float kineticFriction, bool collisionEnable);
	void SetMaterialInteraction(int materialID0, int materialID1, float restitution, float staticFriction, float kineticFriction, bool collisionEnable);
	void SetCallbacks(OnWorldUpdateCallback forceCallback, OnWorldBodyTransfromUpdateCallback tranformCallback);

	dNewtonBody* GetFirstBody() const;
	dNewtonBody* GetNextBody(dNewtonBody* const body) const;

	void* GetFirstContactJoint(dNewtonBody* const body) const;
	void* GetNextContactJoint(dNewtonBody* const body, void* const contact) const;
	void* GetFirstContact(void* const joint) const;
	void* GetNextContact(void* const joint, void* const contact) const;
	dNewtonBody* GetBody0(void* const contact) const;
	dNewtonBody* GetBody1(void* const contact) const;
	void* GetBody0UserData(void* const contact) const;
	void* GetBody1UserData(void* const contact) const;

	// Return a hitinfo object if something was hit. LayerMask is used to exclude colliders from the raycast.
	void* Raycast(float px, float py, float pz, float dx, float dy, float dz, int layerMask);
	static float rayFilterCallback(const NewtonBody* const body, const NewtonCollision* const shapeHit, const dFloat* const hitContact, const dFloat* const hitNormal, dLong collisionID, void* const userData, dFloat intersectParam);
	static unsigned rayPreFilterCallback(const NewtonBody* const body, const NewtonCollision* const collision, void* const userData);

	dNewtonVehicleManager* GetVehicleManager() const;
	void SaveSerializedScene(char* const sceneName);

	private:
	void UpdateWorld();

	const dMaterialProperties& FindMaterial(int id0, int id1) const;
	static void OnContactCollision(const NewtonJoint* contactJoint, dFloat timestep, int threadIndex);
	static int OnBodiesAABBOverlap(const NewtonMaterial* const material, const NewtonBody* const body0, const NewtonBody* const body1, int threadIndex);
	static int OnSubShapeAABBOverlapTest(const NewtonMaterial* const material, const NewtonBody* const body0, const void* const collisionNode0, const NewtonBody* const body1, const void* const collisionNode1, int threadIndex);

	NewtonWorld* m_world;
	dList<dNewtonCollision*> m_collisionCache;
	dTree<dMaterialProperties, long long> m_materialGraph;
	dLong m_realTimeInMicroSeconds;
	dLong m_timeStepInMicroSeconds;
	
	dFloat m_timeStep;
	dFloat m_interpotationParam;

	dVector  m_gravity;
	bool m_asyncUpdateMode;
	OnWorldUpdateCallback m_onUpdateCallback;
	OnWorldBodyTransfromUpdateCallback m_onTransformCallback;
	dMaterialProperties m_defaultMaterial;
	dNewtonVehicleManager* m_vehicleManager;

	rayHitInfo hitInfo;

	friend class dNewtonBody;
	friend class dNewtonCollision;
	friend class dNewtonDynamicBody;
	friend class dNewtonCollisionBox;
	friend class dNewtonKinematicBody;
	friend class dNewtonCollisionMesh;
	friend class dNewtonCollisionNull;
	friend class dNewtonCollisionCone;
	friend class dNewtonCollisionScene;
	friend class dNewtonCollisionSphere;
	friend class dNewtonCollisionCapsule;
	friend class dNewtonCollisionCylinder;
	friend class dNewtonCollisionCompound;
	friend class dNewtonCollisionConvexHull;
	friend class dNewtonCollisionHeightField;
	friend class dNewtonCollisionChamferedCylinder;
};


#endif
