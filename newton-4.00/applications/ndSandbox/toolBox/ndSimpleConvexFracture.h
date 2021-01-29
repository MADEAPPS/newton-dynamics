#ifndef __D_WOOD_FRACTURE_H__
#define __D_WOOD_FRACTURE_H__

#include "ndSandboxStdafx.h"

class ndDemoMesh;
class ndDemoEntityManager;

class ndSimpleConvexFracture: public ndModel
{
	class ndFractureAtom
	{
		public:
		ndFractureAtom()
			:m_centerOfMass(0.0f)
			,m_momentOfInertia(0.0f)
			,m_mesh(nullptr)
			,m_collision(nullptr)
			,m_massFraction(0.0f)
		{
		}

		dVector m_centerOfMass;
		dVector m_momentOfInertia;
		ndDemoMesh* m_mesh;
		ndShapeInstance* m_collision;
		dFloat32 m_massFraction;
	};

	class ndVoronoidFractureEffect : public dList<ndFractureAtom>
	{
		public:
		ndVoronoidFractureEffect(ndDemoEntityManager* const scene, ndMeshEffect* const mesh, dInt32 interiorMaterial);
		ndVoronoidFractureEffect(const ndVoronoidFractureEffect& list);
		~ndVoronoidFractureEffect();
		ndBodyKinematic* m_body;
		bool m_isDead;
	};

	public:
	ndSimpleConvexFracture(ndDemoEntityManager* const scene);
	~ndSimpleConvexFracture();

	void AddFracturedWoodPrimitive(ndDemoEntityManager* const scene, 
		const ndShapeInstance& shape, 
		dFloat32 density, const dVector& origin, 
		dInt32 xCount, dInt32 zCount, dFloat32 spacing, 
		dInt32 type, dInt32 materialID);

	virtual void Update(const ndWorld* const world, dFloat32 timestep);

	void UpdateEffect(ndVoronoidFractureEffect& effect, dFloat32 timestep);
	dList<ndVoronoidFractureEffect> m_effectList;
};



#endif
