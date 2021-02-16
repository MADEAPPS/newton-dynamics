#ifndef __D_SIMPLE_CONVEX_FRACTURE_H__
#define __D_SIMPLE_CONVEX_FRACTURE_H__

#include "ndSandboxStdafx.h"

class ndDemoMesh;
class ndDemoDebriEntity;
class ndDemoEntityManager;
class ndDemoDebriEntityRoot;

class ndSimpleConvexFracture: public ndModel
{
	class ndAtom
	{
		public:
		ndAtom();
		ndAtom(const ndAtom& atom);
		~ndAtom();

		dVector m_centerOfMass;
		dVector m_momentOfInertia;
		ndDemoDebriEntity* m_mesh;
		ndShapeInstance* m_collision;
		dFloat32 m_massFraction;
	};

	public:
	class ndDesc
	{
		public:
		ndDesc()
			:m_pointCloud()
			,m_shape(nullptr)
			,m_outTexture(nullptr)
			,m_innerTexture(nullptr)
			,m_breakImpactSpeed(10.0f)
		{
		}

		dArray<dVector> m_pointCloud;
		ndShapeInstance* m_shape;
		const char* m_outTexture;
		const char* m_innerTexture; 
		dFloat32 m_breakImpactSpeed;
	};

	class ndEffect : public dList<ndAtom>
	{
		public:
		ndEffect(ndSimpleConvexFracture* const manager, const ndDesc& desc);
		ndEffect(const ndEffect& effect);
		~ndEffect();

		private:
		ndBodyKinematic* m_body;
		ndShapeInstance* m_shape;
		ndDemoMesh* m_visualMesh;
		ndDemoDebriEntityRoot* m_debriRootEnt;
		dFloat32 m_breakImpactSpeed;

		friend ndSimpleConvexFracture;
	};

	public:
	ndSimpleConvexFracture(ndDemoEntityManager* const scene);
	~ndSimpleConvexFracture();

	void AddEffect(const ndEffect& effect, dFloat32 mass, const dMatrix& location);

	virtual void AppUpdate(ndWorld* const world);
	virtual void Update(const ndWorld* const world, dFloat32 timestep);

	void UpdateEffect(ndWorld* const world, ndEffect& effect);

	dList<ndEffect> m_effectList;
	dList<ndEffect> m_pendingEffect;
	ndDemoEntityManager* m_scene;
	dSpinLock m_lock;
};

#endif
