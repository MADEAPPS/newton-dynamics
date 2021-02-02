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

// dGeometry.cpp: implementation of the dGeometry class.
//
//////////////////////////////////////////////////////////////////////
#include "ndSandboxStdafx.h"
#include "ndDemoMesh.h"
#include "ndDemoEntity.h"
#include "ndPhysicsWorld.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndTargaToOpenGl.h"
#include "ndDemoEntityManager.h"
#include "ndSkinPeelFracture.h"

ndSkinPeelFracture::ndSkinPeelFracture(ndDemoEntityManager* const scene)
	:ndModel()
	,m_effectList()
	,m_pendingEffect()
	,m_lock()
{
}

ndSkinPeelFracture::~ndSkinPeelFracture()
{
}

void ndSkinPeelFracture::AddFracturedWoodPrimitive(
	ndDemoEntityManager* const scene, const ndShapeInstance& shape,
	const char* const outTexture, const char* const innerTexture,
	dFloat32 breakImpactSpeed, dFloat32 density, const dVector& origin,
	dInt32 xCount, dInt32 zCount, dFloat32 spacing,
	dInt32 type, dInt32 materialID)
{
//breakImpactSpeed = 0;

	// create a newton mesh from the collision primitive
	ndMeshEffect mesh(shape);

	// add the interial material;
	mesh.GetMaterials().PushBack(ndMeshEffect::dMaterial());
	ndMeshEffect::dMaterial& material0 = mesh.GetMaterials()[0];
	ndMeshEffect::dMaterial& material1 = mesh.GetMaterials()[1];
	strcpy(material0.m_textureName, outTexture);
	strcpy(material1.m_textureName, innerTexture);

	dMatrix aligmentUV(dGetIdentityMatrix());
	aligmentUV.m_posit.m_x = -0.5f;
	aligmentUV.m_posit.m_y = -0.5f;
	mesh.UniformBoxMapping(0, aligmentUV);

	// make a visual mesh for display
	ndDemoMesh* const visualMesh = new ndDemoMesh("fracture", &mesh, scene->GetShaderCache());
	
	// create a  mesh fracture from the newton mesh primitive
	ndVoronoidFractureEffect fracture(scene, &mesh, 1);
	
	dFloat32 startElevation = 100.0f;
	dMatrix matrix(dGetIdentityMatrix());
	
	ndWorld* const world = scene->GetWorld();
	for (dInt32 i = 0; i < xCount; i++) 
	{
		dFloat32 x = origin.m_x + (i - xCount / 2) * spacing;
		for (dInt32 j = 0; j < zCount; j++) 
		{
			dFloat32 z = origin.m_z + (j - zCount / 2) * spacing;
	
			matrix.m_posit.m_x = x;
			matrix.m_posit.m_z = z;
			dVector floor(FindFloor(*world, dVector(matrix.m_posit.m_x, startElevation, matrix.m_posit.m_z, dFloat32 (0.0f)), 2.0f * startElevation));
			matrix.m_posit.m_y = floor.m_y + 0.5f;

			dFloat32 mass = density * shape.GetVolume();
			ndBodyDynamic* const body = new ndBodyDynamic();
			ndDemoEntity* const entity = new ndDemoEntity(matrix, nullptr);
			entity->SetMesh(visualMesh, dGetIdentityMatrix());

			body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
			body->SetMatrix(matrix);
			body->SetCollisionShape(shape);
			body->SetMassMatrix(mass, shape);
			world->AddBody(body);
			scene->AddEntity(entity);

			ndVoronoidFractureEffect& newEffect = m_effectList.Append(fracture)->GetInfo();
			newEffect.m_body = body;
			newEffect.m_breakImpactSpeed = breakImpactSpeed;
		}
	}
	
	// do not forget to release the assets	
	visualMesh->Release();
}

ndSkinPeelFracture::ndVoronoidFractureEffect::ndVoronoidFractureEffect(ndDemoEntityManager* const scene, ndMeshEffect* const mesh, dInt32 interiorMaterial)
	:m_body(nullptr)
	,m_breakImpactSpeed(0.0f)
{
	// first we populate the bounding Box area with few random point to get some interior subdivisions.
	// the subdivision are local to the point placement, by placing these pointCloud visual ally with a 3d tool
	// and have precise control of how the debris are created.
	// the number of pieces is equal to the number of point inside the Mesh plus the number of point on the mesh 
	dBigVector bigSize;
	dMatrix matrix(mesh->CalculateOOBB(bigSize));
	dVector size(bigSize);

	dVector pointCloud[32];
	pointCloud[0] = dVector(-size.m_x * 0.5f, -size.m_y * 0.5f, -size.m_z * 0.5f, dFloat32 (0.0f));
	pointCloud[1] = dVector(-size.m_x * 0.5f, -size.m_y * 0.5f,  size.m_z * 0.5f, dFloat32 (0.0f));
	pointCloud[2] = dVector(-size.m_x * 0.5f,  size.m_y * 0.5f, -size.m_z * 0.5f, dFloat32 (0.0f));
	pointCloud[3] = dVector(-size.m_x * 0.5f,  size.m_y * 0.5f,  size.m_z * 0.5f, dFloat32 (0.0f));
	
	pointCloud[4] = dVector(size.m_x * 0.5f, -size.m_y * 0.5f, -size.m_z * 0.5f, dFloat32 (0.0f));
	pointCloud[5] = dVector(size.m_x * 0.5f, -size.m_y * 0.5f,  size.m_z * 0.5f, dFloat32 (0.0f));
	pointCloud[6] = dVector(size.m_x * 0.5f,  size.m_y * 0.5f, -size.m_z * 0.5f, dFloat32 (0.0f));
	pointCloud[7] = dVector(size.m_x * 0.5f,  size.m_y * 0.5f,  size.m_z * 0.5f, dFloat32 (0.0f));
	
	dInt32 count = 8;
	for (dInt32 i = 0; i < count; i++)
	{
		dFloat32 x = dGaussianRandom(size.m_x * 0.1f);
		dFloat32 y = dGaussianRandom(size.m_y * 0.1f);
		dFloat32 z = dGaussianRandom(size.m_y * 0.1f);
		pointCloud[i] += dVector(x, y, z, dFloat32(0.0f));
	}
	
	//pointCloud[0] = dVector::m_zero;
	//count = 0;
	
	// create a texture matrix, for applying the material's UV to all internal faces
	dMatrix textureMatrix(dGetIdentityMatrix());
	textureMatrix[0][0] = 1.0f / size.m_x;
	textureMatrix[1][1] = 1.0f / size.m_y;
	
	//// Get the volume of the original mesh
	dFloat32 volume = dFloat32 (mesh->CalculateVolume());

	// now we call create we decompose the mesh into several convex pieces 
	ndMeshEffect* const debriMeshPieces = mesh->CreateVoronoiConvexDecomposition(count, pointCloud, interiorMaterial, &textureMatrix[0][0]);
	dAssert(debriMeshPieces);
	
	// now we iterate over each pieces and for each one we create a visual entity and a rigid body
	ndMeshEffect* nextDebri;

	dMatrix translateMatrix(dGetIdentityMatrix());
	for (ndMeshEffect* debri = debriMeshPieces->GetFirstLayer(); debri; debri = nextDebri)
	{
		// get next segment piece
		nextDebri = debriMeshPieces->GetNextLayer(debri);
	
		//clip the voronoi cell convexes against the mesh 
		ndMeshEffect* const fracturePiece = mesh->ConvexMeshIntersection(debri);
		if (fracturePiece) 
		{
			// make a convex hull collision shape
			ndShapeInstance* const collision = fracturePiece->CreateConvexCollision(dFloat32(0.0f));
			if (collision)
			{
				// we have a piece which has a convex collision  representation, add that to the list
				ndFractureAtom& atom = Append()->GetInfo();
				atom.m_mesh = new ndDemoMesh("fracture", fracturePiece, scene->GetShaderCache());

				// get center of mass
				dMatrix inertia(collision->CalculateInertia());
				atom.m_centerOfMass = inertia.m_posit;

				// get the mass fraction;
				dFloat32 debriVolume = collision->GetVolume();
				atom.m_massFraction = debriVolume / volume;

				// set the collision shape
				atom.m_collision = collision;

				// transform the mesh the center mass in order to get the 
				//local inertia of this debri piece.
				translateMatrix.m_posit = atom.m_centerOfMass.Scale(-1.0f);
				translateMatrix.m_posit.m_w = 1.0f;
				fracturePiece->ApplyTransform(translateMatrix);
				ndShapeInstance* const inertiaShape = fracturePiece->CreateConvexCollision(dFloat32(0.0f));
				dMatrix momentOfInertia (inertiaShape->CalculateInertia());
				atom.m_momentOfInertia = dVector(momentOfInertia[0][0], momentOfInertia[1][1], momentOfInertia[2][2], dFloat32(0.0f));
				delete inertiaShape;
			}
			delete fracturePiece;
		}
	
		delete debri;
	}
	
	delete debriMeshPieces;
}

ndSkinPeelFracture::ndVoronoidFractureEffect::ndVoronoidFractureEffect(const ndVoronoidFractureEffect& list)
	:m_body(nullptr)
	,m_breakImpactSpeed(list.m_breakImpactSpeed)
{
	for (dListNode* node = list.GetFirst(); node; node = node->GetNext()) 
	{
		ndFractureAtom& atom = Append(node->GetInfo())->GetInfo();
		atom.m_collision = new ndShapeInstance(*atom.m_collision);
		atom.m_mesh->AddRef();
	}
}

ndSkinPeelFracture::ndVoronoidFractureEffect::~ndVoronoidFractureEffect()
{
	for (dListNode* node = GetFirst(); node; node = node->GetNext()) 
	{
		ndFractureAtom& atom = node->GetInfo();
		atom.m_mesh->Release();
		delete atom.m_collision;
	}
}

void ndSkinPeelFracture::Update(const ndWorld* const world, dFloat32 timestep)
{
	dList<ndVoronoidFractureEffect>::dListNode* nextNody;
	for (dList<ndVoronoidFractureEffect>::dListNode* node = m_effectList.GetFirst(); node; node = nextNody)
	{
		nextNody = node->GetNext();
		ndVoronoidFractureEffect& effect = node->GetInfo();

		dFloat32 maxImpactImpulse = 0.0f;
		const ndBodyKinematic::ndContactMap& contactMap = effect.m_body->GetContactMap();
		ndBodyKinematic::ndContactMap::Iterator it(contactMap);
		for (it.Begin(); it; it++)
		{
			ndContact* const contact = *it;
			if (contact->IsActive())
			{
				const ndContactPointList& contactPoints = contact->GetContactPoints();
				for (ndContactPointList::dListNode* contactNode = contactPoints.GetFirst(); contactNode; contactNode = contactNode->GetNext())
				{
					const ndContactMaterial& contactPoint = contactNode->GetInfo();
					const dFloat32 impulseImpact = contactPoint.m_normal_Force.m_impact;
					if (impulseImpact > maxImpactImpulse)
					{
						maxImpactImpulse = impulseImpact;
					}
				}
			}
		}

		dFloat32 impactSpeed = maxImpactImpulse * effect.m_body->GetInvMass();
		if (impactSpeed >= effect.m_breakImpactSpeed)
		{
			dScopeSpinLock lock (m_lock);
			m_effectList.Unlink(node);
			m_pendingEffect.Append(node);
		}
	}
}

void ndSkinPeelFracture::AppUpdate(ndWorld* const world)
{
	if (m_pendingEffect.GetCount())
	{
		D_TRACKTIME();
		world->Sync();
		dList<ndVoronoidFractureEffect>::dListNode* next;
		for (dList<ndVoronoidFractureEffect>::dListNode* node = m_pendingEffect.GetFirst(); node; node = next)
		{
			next = node->GetNext();
			ndVoronoidFractureEffect& effect = node->GetInfo();
			UpdateEffect(world, effect);
			world->DeleteBody(effect.m_body);
			m_pendingEffect.Remove(node);
		}
	}
}

void ndSkinPeelFracture::UpdateEffect(ndWorld* const world, ndVoronoidFractureEffect& effect)
{
	D_TRACKTIME();
	dVector omega(effect.m_body->GetOmega());
	dVector veloc(effect.m_body->GetVelocity());
	dVector massMatrix(effect.m_body->GetMassMatrix());
	dMatrix bodyMatrix(effect.m_body->GetMatrix());
	dVector com(bodyMatrix.TransformVector(effect.m_body->GetCentreOfMass()));

	ndPhysicsWorld* const physicsWorld = (ndPhysicsWorld*)world;
	ndDemoEntityManager* const scene = physicsWorld->GetManager();
	ndDemoEntityNotify* const notify = (ndDemoEntityNotify*)effect.m_body->GetNotifyCallback();
	ndDemoEntity* const visualEntiry = (ndDemoEntity*)notify->GetUserData();

	dMatrix matrix(visualEntiry->GetCurrentMatrix());
	dQuaternion rotation(matrix);

	for (ndVoronoidFractureEffect::dListNode* node = effect.GetFirst(); node; node = node->GetNext())
	{
		ndFractureAtom& atom = node->GetInfo();
		ndDemoEntity* const entity = new ndDemoEntity(dMatrix(rotation, matrix.m_posit), nullptr);
		entity->SetName("debris");
		entity->SetMesh(atom.m_mesh, dGetIdentityMatrix());
		scene->AddEntity(entity);
		
		dFloat32 debriMass = massMatrix.m_w * atom.m_massFraction;
		
		// calculate debris initial velocity
		dVector center(matrix.TransformVector(atom.m_centerOfMass));
		dVector debriVeloc(veloc + omega.CrossProduct(center - com));

		ndBodyDynamic* const body = new ndBodyDynamic();
		body->SetNotifyCallback(new ndDemoEntityNotify(scene, entity));
		body->SetMatrix(matrix);
		body->SetCollisionShape(*atom.m_collision);
		dVector debriMassMatrix(atom.m_momentOfInertia.Scale(debriMass));
		debriMassMatrix.m_w = debriMass;
		body->SetMassMatrix(debriMassMatrix);
		body->SetCentreOfMass(atom.m_centerOfMass);
		body->SetAngularDamping(dVector(dFloat32(0.1f)));

		body->SetOmega(omega);
		body->SetVelocity(debriVeloc);
		world->AddBody(body);
	}
}