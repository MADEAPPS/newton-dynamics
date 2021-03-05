#ifndef __D_CONVEX_FRACTURE_4_H__
#define __D_CONVEX_FRACTURE_4_H__

#include "ndSandboxStdafx.h"

class ndDemoMesh;
class ndDemoDebrisEntity;
class ndDemoEntityManager;
class ndDemoDebrisRootEntity;

class ndConvexFractureModel_4: public ndModel
{
	class ndAtom
	{
		public:
		ndAtom();
		ndAtom(const ndAtom& atom);
		~ndAtom();

		dVector m_centerOfMass;
		dVector m_momentOfInertia;
		ndDemoDebrisEntity* m_mesh;
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
		ndEffect(ndConvexFractureModel_4* const manager, const ndDesc& desc);
		ndEffect(const ndEffect& effect);
		~ndEffect();

		private:
		ndBodyKinematic* m_body;
		ndShapeInstance* m_shape;
		ndDemoMesh* m_visualMesh;
		ndDemoDebrisRootEntity* m_debrisRootEnt;
		dFloat32 m_breakImpactSpeed;
		friend ndConvexFractureModel_4;
	};

	public:
	ndConvexFractureModel_4(ndDemoEntityManager* const scene);
	~ndConvexFractureModel_4();

	void AddEffect(const ndEffect& effect, dFloat32 mass, const dMatrix& location);

	virtual void Update(ndWorld* const world, dFloat32 timestep);
	virtual void PostUpdate(ndWorld* const world, dFloat32 timestep);

	void UpdateEffect(ndWorld* const world, ndEffect& effect);

	void ExplodeLocation(ndBodyDynamic* const body, const dMatrix& matrix, dFloat32 factor) const;

	dList<ndEffect> m_effectList;
	dList<ndEffect> m_pendingEffect;
	ndDemoEntityManager* m_scene;
	dSpinLock m_lock;
};

#endif
