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

#include "ndNewton.h"
#include <gtest/gtest.h>

/* Baseline test: create and destroy an empty Newton world. */
TEST(RemoveJoints, CreateWorld) 
{
	ndWorld* world = new ndWorld();
	ndShapeInstance shape(new ndShapeSphere(0.25));
	ndMatrix matrix(ndGetIdentityMatrix());
	ndBodyDynamic* bodies[3];
	bodies[0] = new ndBodyDynamic();
	bodies[0]->SetCollisionShape(shape);
	bodies[0]->SetMatrix(matrix);
	bodies[0]->SetMassMatrix(1, shape);
	ndSharedPtr<ndBody> bodyPtr(bodies[0]);
	world->AddBody(bodyPtr);

	matrix.m_posit.m_y += 2;

	bodies[1] = new ndBodyDynamic();
	bodies[1]->SetCollisionShape(shape);
	bodies[1]->SetMatrix(matrix);
	bodies[1]->SetMassMatrix(1, shape);
	ndSharedPtr<ndBody> bp(bodies[1]);
	world->AddBody(bp);

	matrix.m_posit.m_y += 2;

	bodies[2] = new ndBodyDynamic();
	bodies[2]->SetCollisionShape(shape);
	bodies[2]->SetMatrix(matrix);
	bodies[2]->SetMassMatrix(1, shape);
	ndSharedPtr<ndBody> bp2(bodies[2]);
	world->AddBody(bp2);

	ndJointFixDistance* joint = new ndJointFixDistance(bodies[0]->GetMatrix().m_posit, bodies[1]->GetMatrix().m_posit, bodies[0], bodies[1]);
	ndSharedPtr<ndJointBilateralConstraint> jp(joint);
	world->AddJoint(jp);
	ndJointFixDistance* joint1 = new ndJointFixDistance(bodies[1]->GetMatrix().m_posit, bodies[2]->GetMatrix().m_posit, bodies[1], bodies[2]);
	ndSharedPtr<ndJointBilateralConstraint> jp1(joint1);
	world->AddJoint(jp1);

	world->Update(0.02);

	std::cout << "started";

	ndBodyKinematic::ndJointList jl = bodies[1]->GetJointList();
	ndBodyKinematic::ndJointList::ndNode* jln = jl.GetFirst();

	std::cout << "Joint to remove: " + std::to_string((long long)jln) + "\n";
	world->RemoveJoint(jln->GetInfo());
	jln = jln->GetNext();

	std::cout << "Next Joint from removed: " + std::to_string((long long)jln) + "\n";
	std::cout << "\n";
	jl = bodies[1]->GetJointList();
	jln = jl.GetFirst();

	std::cout << "First Joint after remove: " + std::to_string((long long)jln) + "\n";
	std::cout << "\n";
	world->CleanUp();
}
