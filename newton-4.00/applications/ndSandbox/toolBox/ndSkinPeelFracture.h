#ifndef __D_SKIN_PEEL_FRACTURE_H__
#define __D_SKIN_PEEL_FRACTURE_H__

#include "ndSandboxStdafx.h"

/*
class ndDemoMesh;
class ndDemoEntityManager;

class ndSkinPeelFracture: public ndModel
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
		dFloat32 m_breakImpactSpeed;
	};

	public:
	ndSkinPeelFracture(ndDemoEntityManager* const scene);
	~ndSkinPeelFracture();

	void AddFracturedWoodPrimitive(ndDemoEntityManager* const scene, 
		const ndShapeInstance& shape, 
		const char* const outTexture, const char* const innerTexture,
		dFloat32 breakImpactSpeed, dFloat32 density, const dVector& origin, 
		dInt32 xCount, dInt32 zCount, dFloat32 spacing, 
		dInt32 type, dInt32 materialID);

	virtual void AppUpdate(ndWorld* const world);
	virtual void Update(const ndWorld* const world, dFloat32 timestep);

	void UpdateEffect(ndWorld* const world, ndVoronoidFractureEffect& effect);

	dList<ndVoronoidFractureEffect> m_effectList;
	dList<ndVoronoidFractureEffect> m_pendingEffect;
	dSpinLock m_lock;
};
*/

class ndDemoMesh;
class ndDemoEntityManager;

class ndSkinPeelFracture : public ndModel
{
	class ndAtom
	{
		public:
		ndAtom();
		ndAtom(const ndAtom& atom);
		~ndAtom();

		dVector m_centerOfMass;
		dVector m_momentOfInertia;
		ndDemoMesh* m_mesh;
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
		ndEffect(ndSkinPeelFracture* const manager, const ndDesc& desc);
		ndEffect(const ndEffect& effect);
		~ndEffect();

		private:
		ndBodyKinematic* m_body;
		ndShapeInstance* m_shape;
		ndDemoMesh* m_visualMesh;
		dFloat32 m_breakImpactSpeed;

		friend ndSkinPeelFracture;
	};

	public:
	ndSkinPeelFracture(ndDemoEntityManager* const scene);
	~ndSkinPeelFracture();

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
