/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "ndSandboxStdafx.h"
#include "ndSkyBox.h"
#include "ndDemoMesh.h"
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"
#include "ndDemoInstanceEntity.h"

#if 0
class ndR2D2Material : public ndApplicationMaterial
{
	public:
	ndR2D2Material()
		:ndApplicationMaterial()
	{
	}

	ndR2D2Material(const ndR2D2Material& src)
		:ndApplicationMaterial(src)
	{
	}

	ndApplicationMaterial* Clone() const
	{
		return new ndR2D2Material(*this);
	}

	bool OnAabbOverlap(const ndContact* const, ndFloat32, const ndShapeInstance& instanceShape0, const ndShapeInstance& instanceShape1) const
	{
		// filter self collision when the contact is with in the same model
		const ndShapeMaterial& material0 = instanceShape0.GetMaterial();
		const ndShapeMaterial& material1 = instanceShape1.GetMaterial();

		ndUnsigned64 pointer0 = material0.m_userParam[ndDemoContactCallback::m_modelPointer].m_intData;
		ndUnsigned64 pointer1 = material1.m_userParam[ndDemoContactCallback::m_modelPointer].m_intData;
		if (pointer0 == pointer1)
		{
			// here we know the part are from the same model.
			// we can apply some more filtering by for now we just disable all self model collisions. 
			return false;
		}
		return true;
	}
};

class R2D2ModelNotify : public ndModelNotify
{
	public:
	R2D2ModelNotify()
		:ndModelNotify()
	{
	}

	R2D2ModelNotify(const R2D2ModelNotify& src)
		:ndModelNotify(src)
	{
	}

	~R2D2ModelNotify()
	{
	}

	ndModelNotify* Clone() const
	{
		return new R2D2ModelNotify(*this);
	}

	void Update(ndFloat32)
	{
	}

	void PostUpdate(ndFloat32)
	{
	}

	void PostTransformUpdate(ndFloat32)
	{
	}
};
#endif

void ndBasicRigidBody(ndDemoEntityManager* const scene)
{
	// build a floor
	//BuildFloorBox(scene, ndGetIdentityMatrix());
	ndSharedPtr<ndBody> body(BuildFlatPlane(scene, true, true));

	ndMatrix matrix(ndGetIdentityMatrix());
#if 0
	ndWorld* const world = scene->GetWorld();

	ndR2D2Material material;

	ndContactCallback* const callback = (ndContactCallback*)scene->GetWorld()->GetContactNotify();
	callback->RegisterMaterial(material, ndDemoContactCallback::m_modelPart, ndDemoContactCallback::m_modelPart);

	auto SetMaterial = [](ndModelArticulation* const root)
	{
		for (ndModelArticulation::ndNode* node = root->GetRoot()->GetFirstIterator(); node; node = node->GetNextIterator())
		{
			ndSharedPtr<ndBody> body(node->m_body);
			ndShapeInstance& instanceShape = body->GetAsBodyDynamic()->GetCollisionShape();
			instanceShape.m_shapeMaterial.m_userId = ndDemoContactCallback::m_modelPart;
			instanceShape.m_shapeMaterial.m_userParam[ndDemoContactCallback::m_modelPointer].m_ptrData = root;
		}
	};

	//char fileName[256];
	//ndGetWorkingFileName("r2d2.urdf", fileName);
	//ndUrdfFile urdf;
	//ndModelArticulation* const r2d2 = urdf.Import(fileName);
	//
	//ndMatrix modelMatrix(r2d2->GetRoot()->m_body->GetMatrix());
	//modelMatrix.m_posit.m_y = 0.5f;
	//r2d2->SetTransform(modelMatrix);
	//r2d2->AddToWorld(world);
	//r2d2->SetNotifyCallback(new R2D2ModelNotify);
	//SetModelVisualMesh(scene, r2d2);
	//SetMaterial(r2d2);

#if 0
	{
		ndUrdfFile urdf1;
		char fileName1[256];

		// export and import file 
		ndGetWorkingFileName("r2d2Exported.urdf", fileName1);
		urdf1.Export(fileName1, r2d2);
		ndModelArticulation* const r2d3 = urdf1.Import(fileName1);

		ndMatrix modelMatrix1(ndGetIdentityMatrix());
		modelMatrix1.m_posit.m_y = 0.5f;
		modelMatrix1.m_posit.m_z = 1.0f;
		r2d3->SetTransform(modelMatrix1);
		r2d3->AddToWorld(world);
		r2d3->SetNotifyCallback(new R2D2ModelNotify);
		SetModelVisualMesh(scene, r2d3);
		SetMaterial(r2d3);
	}
#endif
#endif

	ndMatrix origin1(ndGetIdentityMatrix());
	origin1.m_posit.m_x = 20.0f;

	//AddPlanks(scene, origin1, 1.0f, 4);
	AddSphere(scene, origin1, 1.0f, 0.5f);
	//AddCapsulesStacks(scene, origin1, 10.0f, 0.5f, 0.5f, 1.0f, 1, 2, 7);
	//AddCapsulesStacks(scene, origin1, 10.0f, 0.5f, 0.5f, 1.0f, 10, 10, 7);
	//AddCapsulesStacks(scene, origin1, 10.0f, 0.5f, 0.5f, 1.0f, 4, 4, 4);
	//AddCapsulesStacks(scene, origin1, 10.0f, 0.5f, 0.5f, 1.0f, 2, 2, 7);

	matrix.m_posit.m_x -= 5.0f;
	matrix.m_posit.m_y += 2.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 0.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}




