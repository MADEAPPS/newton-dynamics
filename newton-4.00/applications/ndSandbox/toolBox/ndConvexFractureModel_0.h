#ifndef __D_CONVEX_FRACTURE_0_H__
#define __D_CONVEX_FRACTURE_0_H__

#include "ndSandboxStdafx.h"


class ndConvexFracture
{
	class ndDebrisNotify : public ndDemoEntityNotify
	{
		public:
		ndDebrisNotify(ndDemoEntityManager* const manager, ndDemoEntity* const entity);
		void OnObjectPick() const;
	};

	public:
	ndConvexFracture();
	~ndConvexFracture();

	void GenerateEffect(ndDemoEntityManager* const scene);

	void AddEffect(ndDemoEntityManager* const scene, const dMatrix& location);

	void ExplodeLocation(ndBodyDynamic* const body, const dMatrix& location, dFloat32 factor) const;

	dMatrix m_textureMatrix;
	dArray<dVector> m_pointCloud;
	ndMeshEffect* m_singleManifoldMesh;
	const char* m_innerTexture;
	dFloat32 m_tileFactor;
	dFloat32 m_mass;
	dFloat32 m_breakImpactSpeed;
	dInt32 m_interiorMaterialIndex;

	private:
	ndDemoDebrisRootEntity* m_debriRootEnt;
};

#endif
