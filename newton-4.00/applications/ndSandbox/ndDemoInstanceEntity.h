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

#ifndef __DEMO_INSTANCE_ENTITY_H__
#define __DEMO_INSTANCE_ENTITY_H__

#include "ndDemoEntity.h"


class ndDemoInstanceEntity: public ndDemoEntity
{
	public:

	ndDemoInstanceEntity(const ndDemoInstanceEntity& copyFrom);
	ndDemoInstanceEntity(const dMatrix& matrix, ndDemoEntity* const parent);
	//ndDemoInstanceEntity(DemoInstanceEntityManager& world, const dScene* const scene, dScene::dTreeNode* const rootSceneNode, dTree<DemoMeshInterface*, dScene::dTreeNode*>& meshCache, DemoInstanceEntityManager::EntityDictionary& entityDictionary, ndDemoInstanceEntity* const parent = nullptr);
	virtual ~ndDemoInstanceEntity(void);

	virtual void Render(dFloat32 timeStep, ndDemoEntityManager* const scene, const dMatrix& matrix) const;

};

#endif