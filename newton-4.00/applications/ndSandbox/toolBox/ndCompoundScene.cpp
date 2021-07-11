/* Copyright (c) <2003-2021> <Newton Game Dynamics>
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
#include "ndDemoEntity.h"
#include "ndPhysicsWorld.h"
#include "ndCompoundScene.h"
#include "ndDemoEntityManager.h"
#include "ndHeightFieldPrimitive.h"


ndBodyKinematic* BuildCompoundScene(ndDemoEntityManager* const scene, const dMatrix& location)
{
	ndShapeInstance sceneInstance (new ndShapeCompound());

	ndShapeCompound* const compound = sceneInstance.GetShape()->GetAsShapeCompound();
	compound->BeginAddRemove();

	ndShapeInstance* const subShape = AddHeightfieldSubShape(scene, sceneInstance);

	compound->EndAddRemove();

	ndPhysicsWorld* const world = scene->GetWorld();
	ndBodyDynamic* const body = new ndBodyDynamic();
	//body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
	body->SetNotifyCallback(new ndDemoEntityNotify(scene, nullptr));
	body->SetMatrix(location);
	body->SetCollisionShape(sceneInstance);

	world->AddBody(body);
	return body;
}