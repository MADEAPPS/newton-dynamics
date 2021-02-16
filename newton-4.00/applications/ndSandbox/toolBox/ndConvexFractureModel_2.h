#ifndef __D_CONVEX_FRACTURE_2_H__
#define __D_CONVEX_FRACTURE_2_H__

#include "ndSandboxStdafx.h"

class ndDemoMesh;
class ndDemoDebriEntity;
class ndDemoEntityManager;
class ndDemoDebriEntityRoot;

class ndConvexFractureModel_2: public ndModel
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
			,m_outerShape(nullptr)
			,m_innerShape(nullptr)
			,m_outTexture(nullptr)
			,m_innerTexture(nullptr)
			,m_breakImpactSpeed(10.0f)
		{
		}

		dArray<dVector> m_pointCloud;
		ndShapeInstance* m_outerShape;
		ndShapeInstance* m_innerShape;
		const char* m_outTexture;
		const char* m_innerTexture;
		dFloat32 m_breakImpactSpeed;
	};

	class ndEffect : public dList<ndAtom>
	{
		public:
		ndEffect(ndConvexFractureModel_2* const manager, const ndDesc& desc);
		ndEffect(const ndEffect& effect);
		~ndEffect();

		private:
		ndBodyKinematic* m_body;
		ndShapeInstance* m_shape;
		ndDemoMesh* m_visualMesh;
		ndDemoDebriEntityRoot* m_debriRootEnt;
		dFloat32 m_breakImpactSpeed;
		friend ndConvexFractureModel_2;
	};

	public:
	ndConvexFractureModel_2(ndDemoEntityManager* const scene);
	~ndConvexFractureModel_2();

	void AddEffect(const ndEffect& effect, dFloat32 mass, const dMatrix& location);

	virtual void AppUpdate(ndWorld* const world);
	virtual void Update(const ndWorld* const world, dFloat32 timestep);

	void UpdateEffect(ndWorld* const world, ndEffect& effect);

	void ExplodeLocation(ndBodyDynamic* const body, const dMatrix& matrix, dFloat32 factor) const;

	dList<ndEffect> m_effectList;
	dList<ndEffect> m_pendingEffect;
	ndDemoEntityManager* m_scene;
	dSpinLock m_lock;
};

#endif
