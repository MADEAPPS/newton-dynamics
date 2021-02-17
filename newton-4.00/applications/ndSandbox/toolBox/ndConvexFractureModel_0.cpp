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

//////////////////////////////////////////////////////////////////////
#include "ndSandboxStdafx.h"
#include "ndDemoMesh.h"
#include "ndDemoEntity.h"
#include "ndPhysicsWorld.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndTargaToOpenGl.h"
#include "ndDemoDebriEntity.h"
#include "ndDemoEntityManager.h"
#include "ndConvexFractureModel_0.h"


class ndConvexFractureRootEntity : public ndDemoDebriRootEntity
{
	public:
	ndConvexFractureRootEntity(ndMeshEffect* const singleManifoldMesh, dFloat32 mass)
		:ndDemoDebriRootEntity()
		,m_mass(mass)
		,m_meshVolume(dFloat32(singleManifoldMesh->CalculateVolume()))
	{
	}

	ndConvexFractureRootEntity(const ndConvexFractureRootEntity& copyFrom)
		:ndDemoDebriRootEntity(copyFrom)
		,m_mass(copyFrom.m_mass)
		,m_meshVolume(copyFrom.m_meshVolume)
	{
	}

	dFloat32 m_mass;
	dFloat32 m_meshVolume;
};

class ndConvexFractureEntity: public ndDemoDebriEntity
{
	public:
	ndConvexFractureEntity(ndMeshEffect* const meshNode, dArray<DebriPoint>& vertexArray, ndDemoDebriRootEntity* const parent, const ndShaderPrograms& shaderCache, ndShapeInstance* const collision)
		:ndDemoDebriEntity(meshNode, vertexArray, parent, shaderCache)
		,m_collision(collision)
		,m_drebriBody(nullptr)
	{
		// get center of mass
		dMatrix inertia(collision->CalculateInertia());
		m_centerOfMass = inertia.m_posit;
		
		// get the mass fraction;
		const dFloat32 debriVolume = collision->GetVolume();
		const dFloat32 volume = ((ndConvexFractureRootEntity*)GetParent())->m_meshVolume;
		m_massFraction = debriVolume / volume;
		
		// transform the mesh the center mass in order to get the 
		// local inertia of this debri piece.
		dMatrix translateMatrix(dGetIdentityMatrix());
		translateMatrix.m_posit = m_centerOfMass.Scale(-1.0f);
		translateMatrix.m_posit.m_w = 1.0f;
		meshNode->ApplyTransform(translateMatrix);
		ndShapeInstance* const inertiaShape = meshNode->CreateConvexCollision(dFloat32(0.0f));
		dMatrix momentOfInertia(inertiaShape->CalculateInertia());
		m_momentOfInertia = dVector(momentOfInertia[0][0], momentOfInertia[1][1], momentOfInertia[2][2], dFloat32(0.0f));
		delete inertiaShape;
	}

	ndConvexFractureEntity(const ndConvexFractureEntity& clone)
		:ndDemoDebriEntity(clone)
		,m_centerOfMass(clone.m_centerOfMass)
		,m_momentOfInertia(clone.m_momentOfInertia)
		,m_collision(new ndShapeInstance(*clone.m_collision))
		,m_drebriBody(new ndBodyDynamic())
		,m_massFraction(clone.m_massFraction)
	{
		m_drebriBody->SetCollisionShape(*m_collision);

		ndConvexFractureRootEntity* const cloneParent = ((ndConvexFractureRootEntity*)clone.GetParent());
		dFloat32 debriMass = m_massFraction * cloneParent->m_mass;
		dVector debriMassMatrix(m_momentOfInertia.Scale(debriMass));
		debriMassMatrix.m_w = debriMass;
		m_drebriBody->SetMassMatrix(debriMassMatrix);
		m_drebriBody->SetCentreOfMass(m_centerOfMass);
		m_drebriBody->SetAngularDamping(dVector(dFloat32(0.1f)));
		
		//body->SetOmega(omega);
		//body->SetVelocity(debriVeloc);
	}
	
	~ndConvexFractureEntity()
	{
		delete m_collision;
	}

	dNodeBaseHierarchy* CreateClone() const
	{
		return new ndConvexFractureEntity(*this);
	}

	dVector m_centerOfMass;
	dVector m_momentOfInertia;
	ndShapeInstance* m_collision;
	ndBodyDynamic* m_drebriBody;
	dFloat32 m_massFraction;
};

ndConvexFracture::ndConvexFracture()
	:m_singleManifoldMesh(nullptr)
	,m_textureMatrix(dGetIdentityMatrix())
	,m_pointCloud()
	,m_innerTexture(nullptr)
	,m_tileFactor(1.0f)
	,m_mass(1.0f)
	,m_breakImpactSpeed(10.0f)
	,m_interiorMaterialIndex(0)
	,m_debriRootEnt(nullptr)
{
}

ndConvexFracture::~ndConvexFracture()
{
	if (m_debriRootEnt)
	{
		delete m_debriRootEnt;
	}
}

void ndConvexFracture::GenerateEffect(ndDemoEntityManager* const scene)
{
	ndMeshEffect* const debriMeshPieces = m_singleManifoldMesh->CreateVoronoiConvexDecomposition(m_pointCloud, m_interiorMaterialIndex, &m_textureMatrix[0][0]);
	
	dArray<DebriPoint> vertexArray;
	m_debriRootEnt = new ndConvexFractureRootEntity(m_singleManifoldMesh, m_mass);
	
	ndMeshEffect* nextDebri;
	for (ndMeshEffect* debri = debriMeshPieces->GetFirstLayer(); debri; debri = nextDebri)
	{
		// get next segment piece
		nextDebri = debriMeshPieces->GetNextLayer(debri);
	
		//clip the voronoi cell convexes against the mesh 
		ndMeshEffect* const fracturePiece = m_singleManifoldMesh->ConvexMeshIntersection(debri);
		if (fracturePiece)
		{
			// make a convex hull collision shape
			ndShapeInstance* const collision = fracturePiece->CreateConvexCollision(dFloat32(0.0f));
			if (collision)
			{
				new ndConvexFractureEntity(fracturePiece, vertexArray, m_debriRootEnt, scene->GetShaderCache(), collision);
			}
			delete fracturePiece;
		}
	
		delete debri;
	}
	m_debriRootEnt->FinalizeConstruction(vertexArray);
	
	delete debriMeshPieces;
}

void ndConvexFracture::AddEffect(ndDemoEntityManager* const scene, const dMatrix& location)
{
	ndConvexFractureRootEntity* const entity = new ndConvexFractureRootEntity(*((ndConvexFractureRootEntity*)m_debriRootEnt));
	scene->AddEntity(entity);

	ndWorld* const world = scene->GetWorld();
	for (ndConvexFractureEntity* debriEnt = (ndConvexFractureEntity*)entity->GetChild(); debriEnt; debriEnt = (ndConvexFractureEntity*)debriEnt->GetSibling())
	{
		debriEnt->SetMatrixUsafe(dQuaternion(location), location.m_posit);
		ndBodyDynamic* const body = debriEnt->m_drebriBody;
		world->AddBody(body);
		body->SetNotifyCallback(new ndDemoEntityNotify(scene, debriEnt));
		body->SetMatrix(location);
	}
}