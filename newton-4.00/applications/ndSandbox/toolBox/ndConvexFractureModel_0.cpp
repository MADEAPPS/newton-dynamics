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

	struct JointPair
	{
		dInt32 m_m0;
		dInt32 m_m1;
	};

	dArray<JointPair> m_jointConnection;
	dFloat32 m_mass;
	dFloat32 m_meshVolume;
};

class ndConvexFractureEntity: public ndDemoDebriEntity
{
	public:
	ndConvexFractureEntity(ndMeshEffect* const meshNode, dArray<DebriPoint>& vertexArray, ndDemoDebriRootEntity* const parent, const ndShaderPrograms& shaderCache, ndShapeInstance* const collision, dInt32 enumerator)
		:ndDemoDebriEntity(meshNode, vertexArray, parent, shaderCache)
		,m_collision(collision)
		,m_drebriBody(nullptr)
		,m_enumerator(enumerator)
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
		,m_enumerator(clone.m_enumerator)
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
	dInt32 m_enumerator;
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
	
	dInt32 enumerator = 0;
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
				new ndConvexFractureEntity(fracturePiece, vertexArray, m_debriRootEnt, scene->GetShaderCache(), collision, enumerator);
				enumerator++;
				
			}
			delete fracturePiece;
		}
	
		delete debri;
	}
	m_debriRootEnt->FinalizeConstruction(vertexArray);
	
	delete debriMeshPieces;
	ndConvexFractureRootEntity* const rootEntity = (ndConvexFractureRootEntity*)m_debriRootEnt;

	// calculate joint graph pairs, brute force for now

	ndShapeInstance::ndDistanceCalculator distanceCalculator;
	distanceCalculator.m_matrix0 = dGetIdentityMatrix();
	distanceCalculator.m_matrix1 = dGetIdentityMatrix();
	for (ndConvexFractureEntity* ent0 = (ndConvexFractureEntity*)m_debriRootEnt->GetChild(); ent0; ent0 = (ndConvexFractureEntity*)ent0->GetSibling())
	{
		distanceCalculator.m_shape0 = ent0->m_collision;
		for (ndConvexFractureEntity* ent1 = (ndConvexFractureEntity*)ent0->GetSibling(); ent1; ent1 = (ndConvexFractureEntity*)ent1->GetSibling())
		{
			if ((ent1->m_enumerator != 7))	continue;

			distanceCalculator.m_shape1 = ent1->m_collision;
			if (distanceCalculator.ClosestPoint())
			{
				dFloat32 dist = distanceCalculator.m_normal.DotProduct(distanceCalculator.m_point1 - distanceCalculator.m_point0).GetScalar();
				if (dist <= dFloat32(1.0e-2f))
				{
					dVector point;
					dVector midPoint((distanceCalculator.m_point1 + distanceCalculator.m_point0).Scale(0.5f));

					dVector p0(midPoint + distanceCalculator.m_normal.Scale(0.1f));
					dVector p1(midPoint - distanceCalculator.m_normal.Scale(0.1f));
					bool isFaceContact = (distanceCalculator.m_shape0->ClosestPoint(distanceCalculator.m_matrix0, p0, point) == 3);
					isFaceContact = isFaceContact || (distanceCalculator.m_shape1->ClosestPoint(distanceCalculator.m_matrix1, p1, point) == 3);
					if (isFaceContact)
					{
						dTrace(("pair %d %d\n", ent0->m_enumerator, ent1->m_enumerator));
						ndConvexFractureRootEntity::JointPair pair;
						pair.m_m0 = ent0->m_enumerator;
						pair.m_m1 = ent1->m_enumerator;
						rootEntity->m_jointConnection.PushBack(pair);
					}
				}
			}
		}
		break;
	}
}

void ndConvexFracture::ExplodeLocation(ndBodyDynamic* const body, const dMatrix& location, dFloat32 factor) const
{
	dVector center(location.TransformVector(body->GetCentreOfMass()));
	dVector radios((center - location.m_posit) & dVector::m_triplexMask);
	dVector dir(radios.Normalize());
	dFloat32 lenght = dSqrt(radios.DotProduct(radios).GetScalar());
	dir = dir.Scale(lenght * factor);
	dMatrix matrix(location);
	matrix.m_posit += dir;
	body->SetMatrix(matrix);
}

void ndConvexFracture::AddEffect(ndDemoEntityManager* const scene, const dMatrix& location)
{
	const ndConvexFractureRootEntity* const rootEntity = (ndConvexFractureRootEntity*)m_debriRootEnt;
	ndConvexFractureRootEntity* const entity = new ndConvexFractureRootEntity(*rootEntity);
	scene->AddEntity(entity);

	ndWorld* const world = scene->GetWorld();

	dInt32 bodyCount = 0;
	for (ndConvexFractureEntity* debriEnt = (ndConvexFractureEntity*)entity->GetChild(); debriEnt; debriEnt = (ndConvexFractureEntity*)debriEnt->GetSibling())
	{
		bodyCount++;
		dAssert(debriEnt->m_drebriBody);
		dAssert(debriEnt->m_enumerator < bodyCount);
	}

	ndBodyDynamic** const bodyArray = dAlloca(ndBodyDynamic*, bodyCount);
	memset(bodyArray, 0, bodyCount * sizeof(ndBodyDynamic*));
	for (ndConvexFractureEntity* debriEnt = (ndConvexFractureEntity*)entity->GetChild(); debriEnt; debriEnt = (ndConvexFractureEntity*)debriEnt->GetSibling())
	{
		debriEnt->SetMatrixUsafe(dQuaternion(location), location.m_posit);
		ndBodyDynamic* const body = debriEnt->m_drebriBody;
		world->AddBody(body);
		body->SetNotifyCallback(new ndDemoEntityNotify(scene, debriEnt));
		body->SetMatrix(location);
		bodyArray[debriEnt->m_enumerator] = body;
#if 1
		ExplodeLocation(body, location, 0.3f);
#endif
	}

	// create all the joints
	const dArray<ndConvexFractureRootEntity::JointPair>& jointConnection = rootEntity->m_jointConnection;
	for (dInt32 i = 0; i < jointConnection.GetCount(); i++)
	{
		ndBodyDynamic* const body0 = bodyArray[jointConnection[i].m_m0];
		ndBodyDynamic* const body1 = bodyArray[jointConnection[i].m_m1];

		dMatrix matrix0(body0->GetMatrix());
		dMatrix matrix1(body1->GetMatrix());
		dVector pivot0(matrix0.TransformVector(body0->GetCentreOfMass()));
		dVector pivot1(matrix1.TransformVector(body1->GetCentreOfMass()));
		ndJointFixDistance* const joint = new ndJointFixDistance(pivot0, pivot1, body0, body1);
		joint->SetSolverModel(m_secundaryCloseLoop);
		world->AddJoint(joint);
	}
}