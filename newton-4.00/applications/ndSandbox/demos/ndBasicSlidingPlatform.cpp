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
#include "ndDemoCamera.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndMakeStaticMap.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoEntityManager.h"


// this classs is is onle to show how to take control of the rigiody body in the scene.
class SimpleConveyorFloor : public ndModel
{
	class ndNode : public ndNodeHierarchy<ndNode>
	{
		public:
		ndNode(const ndNode& src)
			:ndNodeHierarchy<ndNode>(src)
			,m_body(src.m_body)
			,m_joint(src.m_joint)
			,m_name(src.m_name)
		{
		}

		ndNode(const ndSharedPtr<ndBody>& body, const ndSharedPtr<ndJointBilateralConstraint>& joint, ndNode* const parent)
			:ndNodeHierarchy<ndNode>()
			,m_body(body)
			,m_joint(joint)
			,m_name("")
		{
			if (parent)
			{
				Attach(parent);
			}
		}
		virtual ~ndNode()
		{

		}

		ndSharedPtr<ndBody> m_body;
		ndSharedPtr<ndJointBilateralConstraint> m_joint;
		ndString m_name;
	};

	public:
	SimpleConveyorFloor()
		:ndModel()
		,m_rootNode(nullptr)
	{

	}
	SimpleConveyorFloor(const SimpleConveyorFloor& src)
		:ndModel(src)
		,m_rootNode(nullptr)
	{
	}

	virtual ~SimpleConveyorFloor()
	{
		if (m_rootNode)
			delete m_rootNode;
	}

	void AddRootBody(const ndSharedPtr<ndBody>& rootBody)
	{
		ndSharedPtr <ndJointBilateralConstraint> dummyJoint;
		m_rootNode = new ndNode(rootBody, dummyJoint, nullptr);
	}

	void IncrementRootPosition(const ndVector& vPosIncrement, ndFloat32 timestep)
	{
		if (m_rootNode && m_rootNode->m_body)
		{
			ndMatrix mBodyMatrix (m_rootNode->m_body->GetMatrix());

#if 0
			timestep = 0;
			mBodyMatrix.m_posit += vPosIncrement;
			m_rootNode->m_body->GetAsBodyKinematic()->SetMatrixUpdateScene(mBodyMatrix);
#else
			ndVector veloc(m_rootNode->m_body->GetVelocity());
			ndVector newVeloc(vPosIncrement.Scale(1.0f / timestep));
			ndVector deltaVeloc(newVeloc - veloc);
			ndVector impulse(deltaVeloc.Scale(1.0f / m_rootNode->m_body->GetInvMass()));
			m_rootNode->m_body->GetAsBodyKinematic()->ApplyImpulsePair(impulse, ndVector::m_zero, timestep);      // 0.8 = momentum canceling factor
#endif
		}
	}

	private:
	ndNode* m_rootNode;
};

class CBasicBodyModelNotify : public ndModelNotify
{
	public:
	CBasicBodyModelNotify(SimpleConveyorFloor* const basicBodyModel)
		:ndModelNotify()
	{
		SetModel(basicBodyModel);
	}
	virtual void PostUpdate(ndFloat32 timestep) override
	{
		ndModelNotify::PostUpdate(timestep);

		auto myModel = (SimpleConveyorFloor*)GetModel();
		ndPhysicsWorld* const world = (ndPhysicsWorld*)myModel->GetWorld();
		ndDemoEntityManager* const scene = world->GetManager();

		if (scene->GetKeyState(ImGuiKey_LeftArrow))
			myModel->IncrementRootPosition(ndVector(-0.01f, 0.f, 0.f, 0.f), timestep);
		else if (scene->GetKeyState(ImGuiKey_RightArrow))
			myModel->IncrementRootPosition(ndVector(0.01f, 0.f, 0.f, 0.f), timestep);
	}
};

void ndBasicSlidingPlatform(ndDemoEntityManager* const scene)
{
	constexpr ndFloat32 groundHeight = 0.f;

	ndVector origin(0.0f, 0.0f, 0.0f, 1.0f);
	ndMatrix xform = ndGetIdentityMatrix();

	ndMatrix groundXF;
	// flat floor and walls
	{
		ndFloat32 angle = 0.f / 180.f * float(ndPi);
		ndQuaternion q(ndVector(0.f, 0.f, 1.f, 0.f), angle);
		groundXF = ndCalculateMatrix(q, origin + ndVector(15.f, groundHeight + 4.f, 0.f, 0.f));

		groundXF.m_posit = origin + ndVector(0.f, -.5f + groundHeight, 0.f, 0.f);

		//ndSharedPtr<ndBody> bodyFloor (BuildFloorBox(scene, groundXF, true));
		// add a kinematic body and set some velocity
		ndSharedPtr<ndBody> bodyFloor (BuildFloorBox(scene, groundXF, "marbleCheckBoard.png", 0.1f, true));

		bodyFloor->SetVelocity(ndVector(0.f, 0.f, -1.0f, 0.f)); 

		// build a fence around the limits
		ndFloat32 size = 100;
		ndMatrix xf = groundXF;
		xf.m_posit = origin + xf.RotateVector(ndVector(size, 0.f, 0.f, 0.f)); 
		AddBox(scene, xf, 0.0f, 1.0f, 5.0f, size * 2.f);
		
		xf.m_posit = origin + xf.RotateVector(ndVector(-size, 0.f, 0.f, 0.f)); 
		AddBox(scene, xf, 0.0f, 1.0f, 5.0f, size * 2.f);
		
		xf.m_posit = origin + xf.RotateVector(ndVector(0.f, 0.f, size, 0.f)); 
		AddBox(scene, xf, 0.0f, size * 1.98f, 5.0f, 1.0f);
		
		xf.m_posit = origin + xf.RotateVector(ndVector(0.f, 0.f, -size, 0.f)); 
		AddBox(scene, xf, 0.0f, size * 1.98f, 5.0f, 1.0f);
	}

	// add dynamic box, 
	{
		xform.m_posit = origin + ndVector(7.0f, 10.0f, 0.0f, 0.0f);
		ndSharedPtr<ndBody> box (AddBox(scene, xform, 10.0f, 5.0f, 0.5f, 1.0f));
		box->SetMatrix(xform);
	}

	if (1) // another dynamic box
	{
		xform.m_posit = origin + ndVector(0.0f, 7.0f, 0.0f, 0.0f);
		ndSharedPtr<ndBody> box (AddBox(scene, xform, 10.0f, 5.0f, 0.5f, 1.0f));
		box->SetMatrix(xform);

		SimpleConveyorFloor* pModel(new SimpleConveyorFloor());
		pModel->AddRootBody(box);
		pModel->SetNotifyCallback(ndSharedPtr<ndModelNotify>(new CBasicBodyModelNotify(pModel)));
		scene->GetWorld()->AddModel(pModel);
	}

	ndMatrix matrix(ndGetIdentityMatrix());
	matrix.m_posit.m_x -= 15.0f;
	matrix.m_posit.m_y += 2.0f;
	ndQuaternion rotation(ndVector(0.0f, 1.0f, 0.0f, 0.0f), 0.0f * ndDegreeToRad);
	scene->SetCameraMatrix(rotation, matrix.m_posit);
}
