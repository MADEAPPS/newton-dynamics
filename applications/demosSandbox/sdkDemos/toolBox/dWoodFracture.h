#ifndef AFX_WOOD_FRACTURE_H___H
#define AFX_WOOD_FRACTURE_H___H

#include "toolbox_stdafx.h"
#include <dVector.h>
#include <dMatrix.h>
#include "TargaToOpenGl.h"
#include "DemoMesh.h"
#include "DemoEntityManager.h"
#include "DemoCamera.h"
#include "PhysicsUtils.h"

#define INITIAL_DELAY							1000
#define NUMBER_OF_INTERNAL_PARTS				3

#define BREAK_IMPACT_IN_METERS_PER_SECONDS		8.0f

class WoodFractureAtom
{
public:
	WoodFractureAtom();
	//
	dVector m_centerOfMass;
	dVector m_momentOfInirtia;
	DemoMesh* m_mesh;
	NewtonCollision* m_collision;
	dFloat m_massFraction;
};

//////////////////////////////////////

class WoodFractureEffect : public dList<WoodFractureAtom>
{
public:
	WoodFractureEffect(NewtonWorld* const world);
	WoodFractureEffect(const WoodFractureEffect& list);
	virtual ~WoodFractureEffect();
	//
	NewtonWorld* m_world;
};

/////////////////////////////////////

class SimpleWoodFracturedEffectEntity : public DemoEntity
{
public:
	SimpleWoodFracturedEffectEntity(DemoMesh* const mesh, const WoodFractureEffect& columnDebris);
	~SimpleWoodFracturedEffectEntity();
	//
	void SimulationPostListener(DemoEntityManager* const scene, DemoEntityManager::dListNode* const mynode, dFloat timeStep);
	static void AddFracturedWoodEntity(DemoEntityManager* const scene, DemoMesh* const visualMesh, NewtonCollision* const collision, const WoodFractureEffect& fractureEffect, const dVector& location);
	//
	int m_delay;
	WoodFractureEffect m_effect;
	NewtonBody* m_myBody;
	dFloat m_myMassInverse;
	//static unsigned m_lock;
};
//unsigned SimpleFracturedEffectEntity::m_lock;

/////////////////////////////////////////

class WoodDelaunayEffect : public WoodFractureEffect
{
public:
	WoodDelaunayEffect(NewtonWorld* const world, NewtonMesh* const mesh, int interiorMaterial);
	//
	static void AddFracturedWoodPrimitive(DemoEntityManager* const scene, dFloat mass, const dVector& origin, const dVector& size, int xCount, int zCount, dFloat spacing, PrimitiveType type, int materialID, const dMatrix& shapeOffsetMatrix);
};

//////////////////////////////////////

class WoodVoronoidEffect : public WoodFractureEffect
{
public:
	WoodVoronoidEffect(NewtonWorld* const world, NewtonMesh* const mesh, int interiorMaterial);
	//
	static void AddFracturedWoodPrimitive(DemoEntityManager* const scene, dFloat mass, const dVector& origin, const dVector& size, int xCount, int zCount, dFloat spacing, int stype, int materialID, const dMatrix& shapeOffsetMatrix);
};


#endif
