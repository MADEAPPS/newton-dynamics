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
#include "ndContactCallback.h"
#include "ndDemoDebrisEntity.h"
#include "ndDemoEntityManager.h"
#include "ndConvexFractureModel_0.h"

class ndFaceArrayDatabase : public ndShapeDebugCallback
{
	public:
	struct ndFaceInfo
	{
		dPlane m_plane;
		dFixSizeBuffer<dVector, 16> m_polygon;
		dInt32 m_count;

		bool CheckCoplanal(const ndFaceInfo& plane, const dMatrix& matrix, const ndFaceInfo& plane2d) const
		{
			dVector pointCloud2d[128];
			dVector dir(m_plane & dVector::m_triplexMask);
			dFloat32 project = dir.DotProduct(plane.m_plane).GetScalar();
			if (project > dFloat32(0.9999f)) 
			{
				dFloat32 dist = m_plane.m_w - plane.m_plane.m_w;
				if (dAbs(dist) < dFloat32(1.0e-4f)) 
				{
					dInt32 pointCount = 0;
					for (dInt32 i = 0; i < m_count; i++)
					{
						for (dInt32 j = 0; j < plane.m_count; j++)
						{
							pointCloud2d[pointCount] = plane2d.m_polygon[i] - matrix.TransformVector(plane.m_polygon[j]);
							pointCount++;
						}
					}
					pointCount = dConvexHull2d(pointCloud2d, pointCount);

					dInt32 k0 = pointCount - 1;
					for (dInt32 k = 0; k < pointCount; k++)
					{
						const dVector e0(dVector::m_zero - pointCloud2d[k0]);
						const dVector e1(pointCloud2d[k] - pointCloud2d[k0]);
						const dVector cross(e1.CrossProduct(e0));
						if (cross.m_z < dFloat32 (1.0e-6f))
						{
							return false;
						}
						k0 = k;
					}

					return true;
				}
			}
			return false;
		}
	};

	ndFaceArrayDatabase(dFloat32 sign = 1.0f)
		:ndShapeDebugCallback()
		,m_sign(sign)
		,m_count(0)
	{
	}

	void DrawPolygon(dInt32 vertexCount, const dVector* const faceArray)
	{
		ndFaceInfo& face = m_polygons[m_count];
		face.m_count = vertexCount;
		dAssert(vertexCount <= face.m_polygon.GetSize());
		for (dInt32 i = 0; i < vertexCount; i++)
		{
			face.m_polygon[i] = faceArray[i];
		}

		dVector normal(dVector::m_zero);
		dVector edge0(faceArray[1] - faceArray[0]);
		for (dInt32 i = 2; i < vertexCount; i++)
		{
			dVector edge1(faceArray[i] - faceArray[0]);
			normal += edge0.CrossProduct(edge1);
			edge0 = edge1;
		}
		normal = normal & dVector::m_triplexMask;
		normal = normal.Normalize().Scale (m_sign);
		face.m_plane = dPlane(normal, -normal.DotProduct(faceArray[0]).GetScalar());
		//dTrace(("%f %f %f %f\n", face.m_plane.m_x, face.m_plane.m_y, face.m_plane.m_z, face.m_plane.m_w));
		m_count++;
		dAssert(m_count < m_polygons.GetSize());
	}

	bool ndFaceArrayDatabase::IsFaceContact(ndShapeInstance* const shape)
	{
		ndFaceArrayDatabase siblingDataBase(-1.0f);
		shape->DebugShape(dGetIdentityMatrix(), siblingDataBase);

		for (dInt32 i = 0; i < m_count; i++)
		{
			const ndFaceInfo& face0 = m_polygons[i];

			dMatrix matrix;
			matrix.m_posit = face0.m_polygon[0];
			matrix.m_front = (face0.m_polygon[1] - matrix.m_posit) & dVector::m_triplexMask;
			matrix.m_front = matrix.m_front.Normalize();
			matrix.m_right = face0.m_plane & dVector::m_triplexMask;
			matrix.m_up = matrix.m_right.CrossProduct(matrix.m_front) & dVector::m_triplexMask;
			matrix.m_posit.m_w = 1.0f;
			matrix = matrix.Inverse();

			ndFaceInfo transformedFace;
			for (dInt32 j = 0; j < face0.m_count; j++)
			{
				transformedFace.m_polygon[j] = matrix.TransformVector(face0.m_polygon[j]);
			}

			for (dInt32 j = 0; j < siblingDataBase.m_count; j++)
			{
				const ndFaceInfo& face1 = siblingDataBase.m_polygons[j];
				if (face0.CheckCoplanal(face1, matrix, transformedFace))
				{
					return true;
				}
			}
		}
		return false;
	}

	dFixSizeBuffer<ndFaceInfo, 128> m_polygons;
	dFloat32 m_sign;
	dInt32 m_count;
};


class ndConvexFractureRootEntity : public ndDemoDebrisRootEntity
{
	public:
	ndConvexFractureRootEntity(ndMeshEffect* const singleManifoldMesh, dFloat32 mass)
		:ndDemoDebrisRootEntity()
		,m_mass(mass)
		,m_meshVolume(dFloat32(singleManifoldMesh->CalculateVolume()))
	{
	}

	ndConvexFractureRootEntity(const ndConvexFractureRootEntity& copyFrom)
		:ndDemoDebrisRootEntity(copyFrom)
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

class ndConvexFractureEntity: public ndDemoDebrisEntity
{
	public:
	ndConvexFractureEntity(ndMeshEffect* const meshNode, dArray<DebrisPoint>& vertexArray, ndDemoDebrisRootEntity* const parent, const ndShaderPrograms& shaderCache, ndShapeInstance* const collision, dInt32 enumerator)
		:ndDemoDebrisEntity(meshNode, vertexArray, parent, shaderCache)
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
		:ndDemoDebrisEntity(clone)
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

ndConvexFracture::ndDebrisNotify::ndDebrisNotify(ndDemoEntityManager* const manager, ndDemoEntity* const entity)
	:ndDemoEntityNotify(manager, entity)
{
}

void ndConvexFracture::ndDebrisNotify::OnObjectPick() const
{
	ndConvexFractureEntity* const debris = (ndConvexFractureEntity*) m_entity;
	dTrace(("debris entity id: %d    ", debris->m_enumerator));
	ndDemoEntityNotify::OnObjectPick();
}

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

void ndConvexFracture::GenerateEffect(ndDemoEntityManager* const scene)
{
	ndMeshEffect* const debrisMeshPieces = m_singleManifoldMesh->CreateVoronoiConvexDecomposition(m_pointCloud, m_interiorMaterialIndex, &m_textureMatrix[0][0]);

	dArray<DebrisPoint> vertexArray;
	m_debriRootEnt = new ndConvexFractureRootEntity(m_singleManifoldMesh, m_mass);

	dInt32 enumerator = 0;
	ndMeshEffect* nextDebris;
	for (ndMeshEffect* debri = debrisMeshPieces->GetFirstLayer(); debri; debri = nextDebris)
	{
		// get next segment piece
		nextDebris = debrisMeshPieces->GetNextLayer(debri);

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

	delete debrisMeshPieces;
	ndConvexFractureRootEntity* const rootEntity = (ndConvexFractureRootEntity*)m_debriRootEnt;

	// calculate joint graph pairs, brute force for now
	ndShapeInstance::ndDistanceCalculator distanceCalculator;
	distanceCalculator.m_matrix0 = dGetIdentityMatrix();
	distanceCalculator.m_matrix1 = dGetIdentityMatrix();
	for (ndConvexFractureEntity* ent0 = (ndConvexFractureEntity*)m_debriRootEnt->GetChild(); ent0; ent0 = (ndConvexFractureEntity*)ent0->GetSibling())
	{
		ndFaceArrayDatabase checkConectivitity;
		distanceCalculator.m_shape0 = ent0->m_collision;
		distanceCalculator.m_shape0->DebugShape(distanceCalculator.m_matrix0, checkConectivitity);
		for (ndConvexFractureEntity* ent1 = (ndConvexFractureEntity*)ent0->GetSibling(); ent1; ent1 = (ndConvexFractureEntity*)ent1->GetSibling())
		{
			distanceCalculator.m_shape1 = ent1->m_collision;
			if (distanceCalculator.ClosestPoint())
			{
				dFloat32 dist = distanceCalculator.m_normal.DotProduct(distanceCalculator.m_point1 - distanceCalculator.m_point0).GetScalar();
				if (dist <= dFloat32(1.0e-2f))
				{
					if (checkConectivitity.IsFaceContact(ent1->m_collision))
					{
						//dTrace(("pair %d %d\n", ent0->m_enumerator, ent1->m_enumerator));
						ndConvexFractureRootEntity::JointPair pair;
						pair.m_m0 = ent0->m_enumerator;
						pair.m_m1 = ent1->m_enumerator;
						rootEntity->m_jointConnection.PushBack(pair);
					}
				}
			}
		}
	}
}

void ndConvexFracture::AddEffect(ndDemoEntityManager* const scene, const dMatrix& location)
{
	const ndConvexFractureRootEntity* const rootEntity = (ndConvexFractureRootEntity*)m_debriRootEnt;
	ndConvexFractureRootEntity* const entity = new ndConvexFractureRootEntity(*rootEntity);
	scene->AddEntity(entity);

	ndWorld* const world = scene->GetWorld();

	dInt32 bodyCount = 0;
	for (ndConvexFractureEntity* debrisEnt = (ndConvexFractureEntity*)entity->GetChild(); debrisEnt; debrisEnt = (ndConvexFractureEntity*)debrisEnt->GetSibling())
	{
		bodyCount = dMax(bodyCount, debrisEnt->m_enumerator + 1);
		dAssert(debrisEnt->m_drebriBody);
		//dAssert(debrisEnt->m_enumerator < bodyCount);
	}

	ndContactCallback* const callback = (ndContactCallback*)world->GetContactNotify();
	ndBodyDynamic** const bodyArray = dAlloca(ndBodyDynamic*, bodyCount);
	memset(bodyArray, 0, bodyCount * sizeof(ndBodyDynamic*));

	dInt32 debrisID = ndContactCallback::m_dedris;
	ndMaterial& material0 = callback->RegisterMaterial(ndContactCallback::m_default, ndContactCallback::m_dedris);
	//ndMaterial& material1 = callback->RegisterMaterial(ndContactCallback::m_dedris, ndContactCallback::m_dedris);
	//instanceShape.m_shapeMaterial.m_userParam[0].m_floatData = 10.0f;

	// register a contact joint physics material pair and 
	// set the physics parameters and application custom options 
	//dFloat32 frictionValue = dFloat32(i) / 15.0f;
	//material.m_staticFriction0 = frictionValue;
	//material.m_staticFriction1 = frictionValue;
	//material.m_dynamicFriction0 = frictionValue;
	//material.m_dynamicFriction1 = frictionValue;


	for (ndConvexFractureEntity* debrisEnt = (ndConvexFractureEntity*)entity->GetChild(); debrisEnt; debrisEnt = (ndConvexFractureEntity*)debrisEnt->GetSibling())
	{

bool test = debrisEnt->m_enumerator == 0;
test = test || debrisEnt->m_enumerator == 1;
test = test || debrisEnt->m_enumerator == 3;
test = test || debrisEnt->m_enumerator == 5;
test = true;
if (!test)
debrisEnt->SetMatrixUsafe(dQuaternion(location), location.m_posit + dVector(0.0f, -10.0f, 0.0f, 0.0f));
else
{
		debrisEnt->SetMatrixUsafe(dQuaternion(location), location.m_posit);
		ndBodyDynamic* const body = debrisEnt->m_drebriBody;
		world->AddBody(body);
		body->SetNotifyCallback(new ndDebrisNotify(scene, debrisEnt));
		body->SetMatrix(location);
		bodyArray[debrisEnt->m_enumerator] = body;

		// set material id properties
		ndShapeInstance& instanceShape = body->GetCollisionShape();
		instanceShape.m_shapeMaterial.m_userId = debrisID;
	#if 0
		ExplodeLocation(body, location, 0.3f);
	#endif
}
	}

	// create all the joints
	const dArray<ndConvexFractureRootEntity::JointPair>& jointConnection = rootEntity->m_jointConnection;
	for (dInt32 i = 0; i < jointConnection.GetCount(); i++)
	{
		bool test = false;
		test = test || jointConnection[i].m_m0 == 0 && jointConnection[i].m_m1 == 1;
		test = test || jointConnection[i].m_m0 == 0 && jointConnection[i].m_m1 == 3;
		test = test || jointConnection[i].m_m0 == 0 && jointConnection[i].m_m1 == 5;
		test = true;
		if (test)
		{
			ndBodyDynamic* const body0 = bodyArray[jointConnection[i].m_m0];
			ndBodyDynamic* const body1 = bodyArray[jointConnection[i].m_m1];
			if (body0 && body1)
			{
				ndJointFixDebrisLink* const joint = new ndJointFixDebrisLink(body0, body1);
				world->AddJoint(joint);
			}
		}
	}
}