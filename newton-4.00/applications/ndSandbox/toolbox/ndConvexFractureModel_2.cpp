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
#include "ndDemoMesh.h"
#include "ndDemoEntity.h"
#include "ndPhysicsWorld.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndDemoEntityNotify.h"
#include "ndDemoDebrisEntity.h"
#include "ndDemoEntityManager.h"
#include "ndConvexFractureModel_2.h"

#define DEBRI_EXPLODE_LOCATION

#if 0
ndConvexFractureModel_2::ndAtom::ndAtom()
	:m_centerOfMass(0.0f)
	,m_momentOfInertia(0.0f)
	,m_mesh(nullptr)
	,m_collision(nullptr)
	,m_massFraction(0.0f)
{
	ndAssert(0);
}

ndConvexFractureModel_2::ndAtom::ndAtom(const ndAtom& atom)
	:m_centerOfMass(atom.m_centerOfMass)
	,m_momentOfInertia(atom.m_momentOfInertia)
	,m_mesh(nullptr)
	,m_collision(new ndShapeInstance(*atom.m_collision))
	,m_massFraction(atom.m_massFraction)
{
	ndAssert(0);
}

ndConvexFractureModel_2::ndAtom::~ndAtom()
{
	ndAssert(0);
	if (m_collision)
	{
		delete m_collision;
	}
}

ndConvexFractureModel_2::ndEffect::ndEffect(ndConvexFractureModel_2* const manager, const ndDesc& desc)
	:ndList<ndAtom>()
	,m_body(nullptr)
	,m_shape(new ndShapeInstance(*desc.m_shape))
	,m_visualMesh(nullptr)
	,m_debrisRootEnt(nullptr)
	,m_breakImpactSpeed(desc.m_breakImpactSpeed)
{
	ndVector pMin;
	ndVector pMax;
	desc.m_shape->CalculateAabb(ndGetIdentityMatrix(), pMin, pMax);
	ndVector size(pMax - pMin);

	// Get the volume of the original mesh
	ndMeshEffect mesh(*desc.m_shape);
	mesh.GetMaterials().PushBack(ndMeshEffect::ndMaterial());
	ndMeshEffect::ndMaterial& material0 = mesh.GetMaterials()[0];
	ndMeshEffect::ndMaterial& material1 = mesh.GetMaterials()[1];
	strcpy(material0.m_textureName, desc.m_outTexture);
	strcpy(material1.m_textureName, desc.m_innerTexture);

	// create a texture matrix, for applying the material's UV to all internal faces
	ndMatrix textureMatrix(ndGetIdentityMatrix());
	textureMatrix[0][0] = 1.0f / size.m_x;
	textureMatrix[1][1] = 1.0f / size.m_y;
	textureMatrix.m_posit.m_x = -0.5f;
	textureMatrix.m_posit.m_y = -0.5f;
	mesh.UniformBoxMapping(0, textureMatrix);

	m_visualMesh = new ndDemoMesh("fracture", &mesh, manager->m_scene->GetShaderCache());

	// now we call create we decompose the mesh into several convex pieces 
	ndMeshEffect* const debrisMeshPieces = mesh.CreateVoronoiConvexDecomposition(desc.m_pointCloud, 1, &textureMatrix[0][0]);
	ndAssert(debrisMeshPieces);

	// now we iterate over each pieces and for each one we create a visual entity and a rigid body
	ndMeshEffect* nextDebris;
	ndMatrix translateMatrix(ndGetIdentityMatrix());

	ndFloat32 volume = ndFloat32(mesh.CalculateVolume());
	ndDemoEntityManager* const scene = manager->m_scene;

	ndArray<glDebrisPoint> vertexArray;
	m_debrisRootEnt = new ndDemoDebrisRootEntity;
	for (ndMeshEffect* debri = debrisMeshPieces->GetFirstLayer(); debri; debri = nextDebris)
	{
		// get next segment piece
		nextDebris = debrisMeshPieces->GetNextLayer(debri);

		//clip the voronoi cell convexes against the mesh 
		ndMeshEffect* const fracturePiece = mesh.ConvexMeshIntersection(debri);
		if (fracturePiece)
		{
			// make a convex hull collision shape
			ndShapeInstance* const collision = fracturePiece->CreateConvexCollision(ndFloat32(0.0f));
			if (collision)
			{
				// we have a piece which has a convex collision  representation, add that to the list
				ndAtom& atom = Append()->GetInfo();
				atom.m_mesh = new ndDemoDebrisEntity(fracturePiece, vertexArray, m_debrisRootEnt, scene->GetShaderCache());

				// get center of mass
				ndMatrix inertia(collision->CalculateInertia());
				atom.m_centerOfMass = inertia.m_posit;

				// get the mass fraction;
				ndFloat32 debriVolume = collision->GetVolume();
				atom.m_massFraction = debriVolume / volume;

				// set the collision shape
				atom.m_collision = collision;

				// transform the mesh the center mass in order to get the 
				//local inertia of this debri piece.
				translateMatrix.m_posit = atom.m_centerOfMass.Scale(-1.0f);
				translateMatrix.m_posit.m_w = 1.0f;
				fracturePiece->ApplyTransform(translateMatrix);
				ndShapeInstance* const inertiaShape = fracturePiece->CreateConvexCollision(ndFloat32(0.0f));
				ndMatrix momentOfInertia(inertiaShape->CalculateInertia());
				atom.m_momentOfInertia = ndVector(momentOfInertia[0][0], momentOfInertia[1][1], momentOfInertia[2][2], ndFloat32(0.0f));
				delete inertiaShape;
			}
			delete fracturePiece;
		}

		delete debri;
	}
	m_debrisRootEnt->FinalizeConstruction(vertexArray);

	delete debrisMeshPieces;
}

ndConvexFractureModel_2::ndEffect::ndEffect(const ndEffect& effect)
	:m_body(new ndBodyDynamic())
	,m_shape(nullptr)
	,m_visualMesh(nullptr)
	,m_debrisRootEnt(new ndDemoDebrisRootEntity(*effect.m_debrisRootEnt))
	,m_breakImpactSpeed(effect.m_breakImpactSpeed)
{
	ndAssert(0);
	m_body->SetCollisionShape(*effect.m_shape);

	ndDemoDebrisEntity* mesh = (ndDemoDebrisEntity*) m_debrisRootEnt->GetFirstChild();
	for (ndNode* node = effect.GetFirst(); node; node = node->GetNext())
	{
		const ndAtom& srcAtom = node->GetInfo();
		ndAtom& newAtom = Append(srcAtom)->GetInfo();
		newAtom.m_mesh = mesh;
		ndAssert(newAtom.m_mesh == srcAtom.m_mesh);

		mesh = (ndDemoDebrisEntity*)mesh->GetNext();
	}
}

ndConvexFractureModel_2::ndEffect::~ndEffect()
{
	ndAssert(0);
	if (m_shape)
	{
		delete m_shape;
	}

	if (m_debrisRootEnt)
	{
		delete m_debrisRootEnt;
	}
}

ndConvexFractureModel_2::ndConvexFractureModel_2(ndDemoEntityManager* const scene)
	:ndModel()
	,m_effectList()
	,m_pendingEffect()
	,m_scene(scene)
	,m_lock()
{
}

ndConvexFractureModel_2::~ndConvexFractureModel_2()
{
}

//void ndConvexFractureModel_2::Update(ndFloat32 timestep)
void ndConvexFractureModel_2::Update(ndFloat32)
{
	ndAssert(0);
	ndList<ndEffect>::ndNode* nextNody;
	for (ndList<ndEffect>::ndNode* node = m_effectList.GetFirst(); node; node = nextNody)
	{
		nextNody = node->GetNext();
		ndEffect& effect = node->GetInfo();

		ndFloat32 maxImpactImpulse = 0.0f;
		const ndBodyKinematic::ndContactMap& contactMap = effect.m_body->GetContactMap();
		ndBodyKinematic::ndContactMap::Iterator it(contactMap);
		for (it.Begin(); it; it++)
		{
			ndContact* const contact = *it;
			if (contact->IsActive())
			{
				const ndContactPointList& contactPoints = contact->GetContactPoints();
				for (ndContactPointList::ndNode* contactNode = contactPoints.GetFirst(); contactNode; contactNode = contactNode->GetNext())
				{
					const ndContactMaterial& contactPoint = contactNode->GetInfo();
					const ndFloat32 impulseImpact = contactPoint.m_normal_Force.m_impact;
					if (impulseImpact > maxImpactImpulse)
					{
						maxImpactImpulse = impulseImpact;
					}
				}
			}
		}

		ndFloat32 impactSpeed = maxImpactImpulse * effect.m_body->GetInvMass();
		if (impactSpeed >= effect.m_breakImpactSpeed)
		{
			ndScopeSpinLock lock(m_lock);
			m_effectList.Unlink(node);
			m_pendingEffect.Append(node);
		}
	}
}

//void ndConvexFractureModel_2::PostUpdate(ndFloat32 timestep)
void ndConvexFractureModel_2::PostUpdate(ndFloat32)
{
	ndAssert(0);
	if (m_pendingEffect.GetCount())
	{
		D_TRACKTIME();
		ndList<ndEffect>::ndNode* next;
		for (ndList<ndEffect>::ndNode* node = m_pendingEffect.GetFirst(); node; node = next)
		{
			next = node->GetNext();
			ndEffect& effect = node->GetInfo();
			UpdateEffect(world, effect);
			//world->DeleteBody(effect.m_body);
			m_pendingEffect.Remove(node);
		}
	}
}

//void ndConvexFractureModel_2::AddEffect(const ndEffect& effect, ndFloat32 mass, const ndMatrix& location)
void ndConvexFractureModel_2::AddEffect(const ndEffect&, ndFloat32, const ndMatrix&)
{
	ndAssert(0);
	//ndEffect& newEffect = m_effectList.Append(effect)->GetInfo();
	//
	//ndDemoEntity* const entity = new ndDemoEntity(location, nullptr);
	//entity->SetMesh(effect.m_visualMesh, ndGetIdentityMatrix());
	//m_scene->AddEntity(entity);
	//
	//ndBodyDynamic* const body = newEffect.m_body->GetAsBodyDynamic();
	//m_scene->GetWorld()->AddBody(body);
	//
	//body->SetNotifyCallback(new ndDemoEntityNotify(m_scene, entity));
	//body->SetMatrix(location);
	//body->SetMassMatrix(mass, *effect.m_shape);
}

void ndConvexFractureModel_2::ExplodeLocation(ndBodyDynamic* const body, const ndMatrix& location, ndFloat32 factor) const
{
	ndAssert(0);
	ndVector center(location.TransformVector(body->GetCentreOfMass()));
	ndVector radios((center - location.m_posit) & ndVector::m_triplexMask);
	ndVector dir(radios.Normalize());
	ndFloat32 lenght = ndSqrt(radios.DotProduct(radios).GetScalar());
	dir = dir.Scale(lenght * factor);
	ndMatrix matrix(location);
	matrix.m_posit += dir;
	body->SetMatrix(matrix);
}

void ndConvexFractureModel_2::UpdateEffect(ndWorld* const world, ndEffect& effect)
{
	ndAssert(0);
	D_TRACKTIME();

	ndVector omega(effect.m_body->GetOmega());
	ndVector veloc(effect.m_body->GetVelocity());
	ndVector massMatrix(effect.m_body->GetMassMatrix());
	ndMatrix bodyMatrix(effect.m_body->GetMatrix());
	ndVector com(bodyMatrix.TransformVector(effect.m_body->GetCentreOfMass()));

	ndPhysicsWorld* const physicsWorld = (ndPhysicsWorld*)world;
	ndDemoEntityManager* const scene = physicsWorld->GetManager();
	ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)effect.m_body->GetNotifyCallback();
	ndDemoEntity* const visualEntity = (ndDemoEntity*)notify->GetUserData();

	ndMatrix matrix(visualEntity->GetCurrentMatrix());
	ndQuaternion rotation(matrix);

	ndDemoEntity* const debriRootEnt = effect.m_debrisRootEnt;
	effect.m_debrisRootEnt = nullptr;
	scene->AddEntity(debriRootEnt);

	for (ndEffect::ndNode* node = effect.GetFirst(); node; node = node->GetNext())
	{
		ndAtom& atom = node->GetInfo();
		ndDemoDebrisEntity* const entity = atom.m_mesh;
		entity->SetMatrix(rotation, matrix.m_posit);

		ndFloat32 debriMass = massMatrix.m_w * atom.m_massFraction;

		// calculate debris initial velocity
		ndVector center(matrix.TransformVector(atom.m_centerOfMass));
		ndVector debriVeloc(veloc + omega.CrossProduct(center - com));

		ndBodyKinematic* const body = new ndBodyDynamic();
		ndSharedPtr<ndBody> bodyPtr(body);
		world->AddBody(bodyPtr);

		body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
		body->SetMatrix(matrix);

		body->SetCollisionShape(*atom.m_collision);
		ndVector debriMassMatrix(atom.m_momentOfInertia.Scale(debriMass));
		debriMassMatrix.m_w = debriMass;
		body->SetMassMatrix(debriMassMatrix);
		body->SetCentreOfMass(atom.m_centerOfMass);
		body->SetAngularDamping(ndVector(ndFloat32(0.1f)));

		body->SetOmega(omega);
		body->SetVelocity(debriVeloc);
#ifdef DEBRI_EXPLODE_LOCATION
		ExplodeLocation(body->GetAsBodyDynamic(), matrix, 0.3f);
#endif
	}
}

#endif