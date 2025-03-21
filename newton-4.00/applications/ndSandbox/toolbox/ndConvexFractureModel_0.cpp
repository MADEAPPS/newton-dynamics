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

//////////////////////////////////////////////////////////////////////
#include "ndSandboxStdafx.h"
#include "ndDemoMesh.h"
#include "ndDemoEntity.h"
#include "ndPhysicsWorld.h"
#include "ndPhysicsUtils.h"
#include "ndPhysicsWorld.h"
#include "ndContactCallback.h"
#include "ndDemoDebrisEntity.h"
#include "ndDemoEntityManager.h"
#include "ndConvexFractureModel_0.h"

#if 0
class ndFaceArrayDatabase : public ndShapeDebugNotify
{
	public:
	struct ndFaceInfo
	{
		bool CheckCoplanal(const ndFaceInfo& plane, const ndMatrix& matrix, const ndFaceInfo& plane2d) const
		{
			ndVector pointCloud2d[16*16];
			ndVector dir(m_plane & ndVector::m_triplexMask);
			ndFloat32 project = dir.DotProduct(plane.m_plane).GetScalar();
			if (project > ndFloat32(0.9999f)) 
			{
				ndFloat32 dist = m_plane.m_w - plane.m_plane.m_w;
				if (ndAbs(dist) < ndFloat32(1.0e-4f)) 
				{
					ndInt32 pointCount = 0;
					for (ndInt32 i = 0; i < m_count; ++i)
					{
						for (ndInt32 j = 0; j < plane.m_count; ++j)
						{
							pointCloud2d[pointCount] = plane2d.m_polygon[i] - matrix.TransformVector(plane.m_polygon[j]);
							pointCount++;
							ndAssert(pointCount < ndInt32(sizeof(pointCloud2d) / sizeof(pointCloud2d[0])));
						}
					}
					pointCount = ndConvexHull2d(pointCloud2d, pointCount);

					ndInt32 k0 = pointCount - 1;
					for (ndInt32 k = 0; k < pointCount; ++k)
					{
						const ndVector e0(ndVector::m_zero - pointCloud2d[k0]);
						const ndVector e1(pointCloud2d[k] - pointCloud2d[k0]);
						const ndVector cross(e1.CrossProduct(e0));
						if (cross.m_z < ndFloat32 (1.0e-6f))
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

		ndPlane m_plane;
		ndFixSizeArray<ndVector, 16> m_polygon;
		ndInt32 m_count;
	};

	ndFaceArrayDatabase(ndFloat32 sign = 1.0f)
		:ndShapeDebugNotify()
		,m_sign(sign)
		,m_count(0)
	{
	}

	void DrawPolygon(ndInt32 vertexCount, const ndVector* const faceArray, const ndEdgeType* const)
	{
		ndFaceInfo& face = m_polygons[m_count];
		face.m_count = vertexCount;
		ndAssert(vertexCount <= face.m_polygon.GetCapacity());
		for (ndInt32 i = 0; i < vertexCount; ++i)
		{
			face.m_polygon[i] = faceArray[i];
		}

		ndVector normal(ndVector::m_zero);
		ndVector edge0(faceArray[1] - faceArray[0]);
		for (ndInt32 i = 2; i < vertexCount; ++i)
		{
			ndVector edge1(faceArray[i] - faceArray[0]);
			normal += edge0.CrossProduct(edge1);
			edge0 = edge1;
		}
		normal = normal & ndVector::m_triplexMask;
		normal = normal.Normalize().Scale (m_sign);
		face.m_plane = ndPlane(normal, -normal.DotProduct(faceArray[0]).GetScalar());
		//ndTrace(("%f %f %f %f\n", face.m_plane.m_x, face.m_plane.m_y, face.m_plane.m_z, face.m_plane.m_w));
		m_count++;
		ndAssert(m_count < m_polygons.GetCapacity());
	}

	bool IsFaceContact(ndShapeInstance* const shape)
	{
		ndFaceArrayDatabase siblingDataBase(-1.0f);
		shape->DebugShape(ndGetIdentityMatrix(), siblingDataBase);

		for (ndInt32 i = 0; i < m_count; ++i)
		{
			const ndFaceInfo& face0 = m_polygons[i];

			ndMatrix matrix;
			matrix.m_posit = face0.m_polygon[0];
			matrix.m_front = (face0.m_polygon[1] - matrix.m_posit) & ndVector::m_triplexMask;
			matrix.m_front = matrix.m_front.Normalize();
			matrix.m_right = face0.m_plane & ndVector::m_triplexMask;
			matrix.m_up = matrix.m_right.CrossProduct(matrix.m_front) & ndVector::m_triplexMask;
			matrix.m_posit.m_w = 1.0f;
			matrix = matrix.OrthoInverse();

			ndFaceInfo transformedFace;
			for (ndInt32 j = 0; j < face0.m_count; ++j)
			{
				transformedFace.m_polygon[j] = matrix.TransformVector(face0.m_polygon[j]);
			}

			for (ndInt32 j = 0; j < siblingDataBase.m_count; ++j)
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

	ndFixSizeArray<ndFaceInfo, 128> m_polygons;
	ndFloat32 m_sign;
	ndInt32 m_count;
};


class ndConvexFractureRootEntity : public ndDemoDebrisRootEntity
{
	public:
	ndConvexFractureRootEntity(ndMeshEffect* const singleManifoldMesh, ndFloat32 mass)
		:ndDemoDebrisRootEntity()
		,m_mass(mass)
		,m_meshVolume(ndFloat32(singleManifoldMesh->CalculateVolume()))
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
		ndInt32 m_m0;
		ndInt32 m_m1;
	};

	ndArray<JointPair> m_jointConnection;
	ndFloat32 m_mass;
	ndFloat32 m_meshVolume;
};

class ndConvexFractureEntity: public ndDemoDebrisEntity
{
	public:
	ndConvexFractureEntity(ndMeshEffect* const meshNode, ndArray<glDebrisPoint>& vertexArray, ndDemoDebrisRootEntity* const parent, const ndShaderCache& shaderCache, ndShapeInstance* const collision, ndInt32 enumerator)
		:ndDemoDebrisEntity(meshNode, vertexArray, parent, shaderCache)
		,m_collision(collision)
		,m_drebriBody____(nullptr)
		,m_enumerator(enumerator)
	{
		// get center of mass
		ndMatrix inertia(collision->CalculateInertia());
		m_centerOfMass = inertia.m_posit;
		
		// get the mass fraction;
		const ndFloat32 debriVolume = collision->GetVolume();
		const ndFloat32 volume = ((ndConvexFractureRootEntity*)GetParent())->m_meshVolume;
		m_massFraction = debriVolume / volume;
		
		// transform the mesh the center mass in order to get the 
		// local inertia of this debri piece.
		ndMatrix translateMatrix(ndGetIdentityMatrix());
		translateMatrix.m_posit = m_centerOfMass.Scale(-1.0f);
		translateMatrix.m_posit.m_w = 1.0f;
		meshNode->ApplyTransform(translateMatrix);
		ndShapeInstance* const inertiaShape = meshNode->CreateConvexCollision(ndFloat32(0.0f));
		ndMatrix momentOfInertia(inertiaShape->CalculateInertia());
		m_momentOfInertia = ndVector(momentOfInertia[0][0], momentOfInertia[1][1], momentOfInertia[2][2], ndFloat32(0.0f));
		delete inertiaShape;
	}

	ndConvexFractureEntity(const ndConvexFractureEntity& clone)
		:ndDemoDebrisEntity(clone)
		,m_centerOfMass(clone.m_centerOfMass)
		,m_momentOfInertia(clone.m_momentOfInertia)
		,m_collision(new ndShapeInstance(*clone.m_collision))
		,m_drebriBody____(new ndBodyDynamic())
		,m_massFraction(clone.m_massFraction)
		,m_enumerator(clone.m_enumerator)
	{
		m_drebriBody____->SetCollisionShape(*m_collision);

		ndConvexFractureRootEntity* const cloneParent = ((ndConvexFractureRootEntity*)clone.GetParent());
		ndFloat32 debriMass = m_massFraction * cloneParent->m_mass;
		ndVector debriMassMatrix(m_momentOfInertia.Scale(debriMass));
		debriMassMatrix.m_w = debriMass;
		m_drebriBody____->SetMassMatrix(debriMassMatrix);
		m_drebriBody____->SetCentreOfMass(m_centerOfMass);
		m_drebriBody____->SetAngularDamping(ndVector(ndFloat32(0.1f)));
		
		//body->SetOmega(omega);
		//body->SetVelocity(debriVeloc);
	}
	
	~ndConvexFractureEntity()
	{
		delete m_collision;
	}

	ndDemoEntity* CreateClone() const
	{
		return new ndConvexFractureEntity(*this);
	}

	ndVector m_centerOfMass;
	ndVector m_momentOfInertia;
	ndShapeInstance* m_collision;
	//ndBodyDynamic* m_drebriBody;
	ndSharedPtr<ndBodyKinematic> m_drebriBody____;
	ndFloat32 m_massFraction;
	ndInt32 m_enumerator;
};

ndConvexFracture::ndDebrisNotify::ndDebrisNotify(ndDemoEntityManager* const manager, ndDemoEntity* const entity)
	:ndDemoEntityNotify(manager, entity)
{
}

void ndConvexFracture::ndDebrisNotify::OnObjectPick() const
{
	ndAssert(0);
	ndTrace(("debris entity id: %d    ", ((ndConvexFractureEntity*)m_entity)->m_enumerator));
	//ndDemoEntityNotify::OnObjectPick();
}

ndConvexFracture::ndConvexFracture()
	:m_textureMatrix(ndGetIdentityMatrix())
	,m_pointCloud()
	,m_singleManifoldMesh(nullptr)
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

void ndConvexFracture::ExplodeLocation(ndBodyDynamic* const body, const ndMatrix& location, ndFloat32 factor) const
{
	ndVector center(location.TransformVector(body->GetCentreOfMass()));
	ndVector radios((center - location.m_posit) & ndVector::m_triplexMask);
	ndVector dir(radios.Normalize());
	ndFloat32 lenght = ndSqrt(radios.DotProduct(radios).GetScalar());
	dir = dir.Scale(lenght * factor);
	ndMatrix matrix(location);
	matrix.m_posit += dir;
	body->SetMatrix(matrix);
}

void ndConvexFracture::GenerateEffect(ndDemoEntityManager* const scene)
{
	ndMeshEffect* const debrisMeshPieces = m_singleManifoldMesh->CreateVoronoiConvexDecomposition(m_pointCloud, m_interiorMaterialIndex, &m_textureMatrix[0][0]);

	ndArray<glDebrisPoint> vertexArray;
	m_debriRootEnt = new ndConvexFractureRootEntity(m_singleManifoldMesh, m_mass);

	ndInt32 enumerator = 0;
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
			ndShapeInstance* const collision = fracturePiece->CreateConvexCollision(ndFloat32(0.0f));
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
	ndShapeInstance::ndDistanceCalculator distanceCalculator(scene->GetWorld()->GetScene());
	distanceCalculator.m_matrix0 = ndGetIdentityMatrix();
	distanceCalculator.m_matrix1 = ndGetIdentityMatrix();
	for (ndConvexFractureEntity* ent0 = (ndConvexFractureEntity*)m_debriRootEnt->GetFirstChild(); ent0; ent0 = (ndConvexFractureEntity*)ent0->GetNext())
	{
		ndFaceArrayDatabase checkConectivitity;
		distanceCalculator.m_shape0 = ent0->m_collision;
		distanceCalculator.m_shape0->DebugShape(distanceCalculator.m_matrix0, checkConectivitity);
		for (ndConvexFractureEntity* ent1 = (ndConvexFractureEntity*)ent0->GetNext(); ent1; ent1 = (ndConvexFractureEntity*)ent1->GetNext())
		{
			distanceCalculator.m_shape1 = ent1->m_collision;
			if (distanceCalculator.ClosestPoint())
			{
				ndFloat32 dist = distanceCalculator.m_normal.DotProduct(distanceCalculator.m_point1 - distanceCalculator.m_point0).GetScalar();
				if (dist <= ndFloat32(1.0e-2f))
				{
					if (checkConectivitity.IsFaceContact(ent1->m_collision))
					{
						//ndTrace(("pair %d %d\n", ent0->m_enumerator, ent1->m_enumerator));
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

void ndConvexFracture::AddEffect(ndDemoEntityManager* const scene, const ndMatrix& location)
{
	const ndConvexFractureRootEntity* const rootEntity = (ndConvexFractureRootEntity*)m_debriRootEnt;
	ndConvexFractureRootEntity* const entity = new ndConvexFractureRootEntity(*rootEntity);
	scene->AddEntity(entity);

	//ndWorld* const world = scene->GetWorld();

	ndInt32 bodyCount = 0;
	for (ndConvexFractureEntity* debrisEnt = (ndConvexFractureEntity*)entity->GetFirstChild(); debrisEnt; debrisEnt = (ndConvexFractureEntity*)debrisEnt->GetNext())
	{
		bodyCount = ndMax(bodyCount, debrisEnt->m_enumerator + 1);
		ndAssert(*debrisEnt->m_drebriBody____);
		//ndAssert(debrisEnt->m_enumerator < bodyCount);
	}

	//ndContactCallback* const callback = (ndContactCallback*)world->GetContactNotify();
	ndBodyDynamic** const bodyArray = ndAlloca(ndBodyDynamic*, bodyCount);
	memset(bodyArray, 0, bodyCount * sizeof(ndBodyDynamic*));
	
	ndAssert(0);
	//ndInt32 debrisID = ndApplicationMaterial::m_dedris;
	//ndMaterial& material0 = callback->RegisterMaterial(ndContactCallback::m_default, ndContactCallback::m_dedris);
	//ndAssert(0);
	//material0;
	//ndMaterial& material1 = callback->RegisterMaterial(ndContactCallback::m_dedris, ndContactCallback::m_dedris);
	//instanceShape.m_shapeMaterial.m_userParam[0].m_floatData = 10.0f;

	// register a contact joint physics material pair and 
	// set the physics parameters and application custom options 
	//ndFloat32 frictionValue = ndFloat32(i) / 15.0f;
	//material.m_staticFriction0 = frictionValue;
	//material.m_staticFriction1 = frictionValue;
	//material.m_dynamicFriction0 = frictionValue;
	//material.m_dynamicFriction1 = frictionValue;


	for (ndConvexFractureEntity* debrisEnt = (ndConvexFractureEntity*)entity->GetFirstChild(); debrisEnt; debrisEnt = (ndConvexFractureEntity*)debrisEnt->GetNext())
	{

bool test = debrisEnt->m_enumerator == 0;
test = test || debrisEnt->m_enumerator == 1;
test = test || debrisEnt->m_enumerator == 3;
test = test || debrisEnt->m_enumerator == 5;
test = true;
if (!test)
debrisEnt->SetMatrix(ndQuaternion(location), location.m_posit + ndVector(0.0f, -10.0f, 0.0f, 0.0f));
else
{
		ndAssert(0);
		//debrisEnt->SetMatrix(ndQuaternion(location), location.m_posit);
		////ndBodyDynamic* const body = debrisEnt->m_drebriBody;
		//world->AddBody(debrisEnt->m_drebriBody____);
		//debrisEnt->m_drebriBody____->SetNotifyCallback(new ndDebrisNotify(scene, debrisEnt));
		//debrisEnt->m_drebriBody____->SetMatrix(location);
		//bodyArray[debrisEnt->m_enumerator] = debrisEnt->m_drebriBody____->GetAsBodyDynamic();
		//
		//// set material id properties
		//ndShapeInstance& instanceShape = debrisEnt->m_drebriBody____->GetCollisionShape();
		//instanceShape.m_shapeMaterial.m_userId = debrisID;
	#if 0
		ExplodeLocation(body, location, 0.3f);
	#endif
}
	}

	// create all the joints
	const ndArray<ndConvexFractureRootEntity::JointPair>& jointConnection = rootEntity->m_jointConnection;
	for (ndInt32 i = 0; i < jointConnection.GetCount(); ++i)
	{
		bool test = false;
		test = test || ((jointConnection[i].m_m0 == 0) && (jointConnection[i].m_m1 == 1));
		test = test || ((jointConnection[i].m_m0 == 0) && (jointConnection[i].m_m1 == 3));
		test = test || ((jointConnection[i].m_m0 == 0) && (jointConnection[i].m_m1 == 5));
		test = true;
		if (test)
		{
			ndAssert(0);
			//ndBodyDynamic* const body0 = bodyArray[jointConnection[i].m_m0];
			//ndBodyDynamic* const body1 = bodyArray[jointConnection[i].m_m1];
			//if (body0 && body1)
			//{
			//	ndJointFix6dof* const joint = new ndJointFix6dof(body0->GetMatrix(), body0, body1);
			//	ndAssert(0);
			//	//joint->SetSolverModel(m_secundaryCloseLoop);
			//	world->AddJoint(joint);
			//}
		}
	}
}

#endif