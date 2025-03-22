
/* Copyright (c) <2003-2019> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include <cstdio>
#include "ndNewton.h"
#include <gtest/gtest.h>

using ClientNodePtr = std::shared_ptr<class ClientNode>;

constexpr float BOX_DIM = 1.0f;
constexpr float Z_OFFSET = -4.0f;
constexpr float HALF_BOX_DIM = BOX_DIM * 0.5f;

class ClientNode : public ndClassAlloc, public std::enable_shared_from_this<ClientNode>
{
	public:
	static ClientNodePtr create() 
	{ 
		//return std::make_shared<ClientNode>(); 
		return std::shared_ptr<class ClientNode>(new ClientNode);
	}

	ClientNodePtr getPtr() 
	{ 
		return shared_from_this(); 
	}

	ClientNode() 
		:ndClassAlloc()
		,worldMatrix(ndGetIdentityMatrix())
	{
	}

	~ClientNode() = default;

	void* getUserdata() { return userdata; }
	void setUserData(void* const data) { userdata = data; }

	void setPickableState(bool state) { pickable = state; }
	bool getPickableState() const { return pickable; }

	void setTransform(const ndMatrix& matrix) { worldMatrix = matrix; }
	const ndMatrix& getTransform() const {return worldMatrix;}

	private:
	void* userdata = nullptr;
	ndMatrix worldMatrix;
	bool pickable = true;
};

class NotifyClientNode : public ndBodyNotify
{
	public:
	NotifyClientNode(ClientNodePtr& clientNode) :
		ndBodyNotify(ndVector::m_zero), // 0 gravity
		node(clientNode)
	{
	}

	~NotifyClientNode() = default;

	void OnTransform(ndInt32, const ndMatrix&) override
	{
	}

	void OnApplyExternalForce(ndInt32, ndFloat32) override
	{
	}

	// user data is a pointer to our client node
	void* GetUserData() const
	{
		return node.get();
	}

	private:
	ClientNodePtr node;
};

static ndBodyDynamic* buildNewtonBodyFromClientNode(ClientNodePtr& node)
{
	// Create the rigid body
	ndBodyDynamic* const body = new ndBodyDynamic();

	// use the client's transform
	body->SetMatrix(node->getTransform());

	// allow the client node to receive newton body notifications
	body->SetNotifyCallback(new NotifyClientNode(node));

	// bind this ndBodyDynamic* to the client node with userData
	node->setUserData(body);

	// Attach a collision shape and use a convenience function to automatically
	// compute the inertia matrix for the body.
	ndShapeInstance box(new ndShapeBox(BOX_DIM, BOX_DIM, BOX_DIM));
	body->SetCollisionShape(box);
	body->SetMassMatrix(2.0f, box);

	return body;
};

class RayCastFilter : public ndRayCastClosestHitCallback
{
public:
	RayCastFilter() :
		ndRayCastClosestHitCallback()
	{
	}

	// here's where we can filter out a node that has picking disabled 
	ndUnsigned32 OnRayPrecastAction(const ndBody* const body, const ndShapeInstance* const) override
	{
		ndBody* const nbody = const_cast<ndBody*> (body);
		ndBodyKinematic* const kBody = nbody->GetAsBodyKinematic();

		NotifyClientNode* const notify = (NotifyClientNode*)kBody->GetNotifyCallback();
		ClientNodePtr node = static_cast<ClientNode*> (notify->GetUserData())->getPtr();
		if (!node) return 0;

		// pass through the body if it is not in the pickable state
		return node->getPickableState() ? 1 : 0;
	}
};

struct PickInfo
{
	ndVector rayStart;
	ndVector rayEnd;
	ndFloat32 parameter;
	ndVector position;
	ndVector normal;
	ndBodyKinematic* pickedBody = nullptr;
	ClientNodePtr pickedNode = nullptr;
};

static void rayCast(ndWorld& world, PickInfo& info)
{
	RayCastFilter rayCaster;
	if (world.RayCast(rayCaster, info.rayStart, info.rayEnd))
	{
		info.parameter = rayCaster.m_param;
		info.position = rayCaster.m_contact.m_point;
		info.normal = rayCaster.m_contact.m_normal;
		info.pickedBody = (ndBodyKinematic*)rayCaster.m_contact.m_body0;

		NotifyClientNode* const notify = (NotifyClientNode*)info.pickedBody->GetNotifyCallback();
		info.pickedNode = static_cast<ClientNode*> (notify->GetUserData())->getPtr();
	}
}

TEST(RayCast, rayCastFilter)
{
	ndWorld world;
	world.SetSubSteps(2);
	world.SetThreadCount(std::thread::hardware_concurrency() - 1);

	// create 2 client nodes
	// first is at the origin
	ClientNodePtr node0 = ClientNode::create();
	ndMatrix matrix(ndGetIdentityMatrix());
	node0->setTransform(matrix);
	ndSharedPtr<ndBody> body0 (buildNewtonBodyFromClientNode(node0));
	world.AddBody(body0);

	// second is offset down the Z axis
	ClientNodePtr node1 = ClientNode::create();
	matrix.m_posit.m_z = Z_OFFSET;
	node1->setTransform(matrix);
	ndSharedPtr<ndBody> body1 (buildNewtonBodyFromClientNode(node1));
	world.AddBody(body1);

	PickInfo info;
	info.rayStart = ndVector(0.0f, 0.0f, 10.0f, 1.0f);
	info.rayEnd = ndVector(0.0f, 0.0f, -10.0f, 1.0f);

	// fire the pick ray
	rayCast(world, info);

	EXPECT_TRUE(info.pickedBody != nullptr);
	EXPECT_TRUE(info.pickedNode != nullptr);

	// this first body we hit should be at the origin
	EXPECT_TRUE(info.pickedBody->GetMatrix().m_posit.m_z == 0.0f);

	// the first body hit point z coordinate position should be at body z cord + HALF_BOX_DIM
	EXPECT_TRUE(info.position.m_z == info.pickedBody->GetMatrix().m_posit.m_z + HALF_BOX_DIM);

	// now make this node we just hit unpickable
	info.pickedNode->setPickableState(false);

	// fire the same ray again
	rayCast(world, info);

	// this time, because the first body is now unseen by the ray, 
	// we hit the second body whose z coordinate is Z_OFFSET
	EXPECT_TRUE(info.pickedBody->GetMatrix().m_posit.m_z == Z_OFFSET);

	// the second body hitpoint z coordinate should be Z_OFFSET + HALF_BOX_DIM;
	EXPECT_TRUE(info.position.m_z == Z_OFFSET + HALF_BOX_DIM); 
}
