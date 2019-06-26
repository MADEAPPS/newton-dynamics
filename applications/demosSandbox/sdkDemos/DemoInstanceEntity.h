/* Copyright (c) <2003-2016> <Newton Game Dynamics>
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

#include "DemoEntity.h"


class DemoInstanceEntity: public DemoEntity
{
	public:

	DemoInstanceEntity(const DemoInstanceEntity& copyFrom);
	DemoInstanceEntity(const dMatrix& matrix, DemoEntity* const parent);
	//DemoInstanceEntity(DemoInstanceEntityManager& world, const dScene* const scene, dScene::dTreeNode* const rootSceneNode, dTree<DemoMeshInterface*, dScene::dTreeNode*>& meshCache, DemoInstanceEntityManager::EntityDictionary& entityDictionary, DemoInstanceEntity* const parent = NULL);
	virtual ~DemoInstanceEntity(void);

	virtual void Render(dFloat timeStep, DemoEntityManager* const scene) const;

};

#endif