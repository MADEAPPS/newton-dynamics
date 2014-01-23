/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "dgPhysicsStdafx.h"


#include "dgBody.h"
#include "dgWorld.h"
#include "dgContact.h"
#include "dgMeshEffect.h"
#include "dgCollisionBVH.h"
#include "dgDeformableBody.h"
#include "dgDeformableContact.h"
#include "dgCollisionConvexPolygon.h"
#include "dgCollisionDeformableSolidMesh.h"


#define DG_DEFORMABLE_DEFAULT_STIFFNESS		 (dgFloat32 (0.8f))


//#define DG_DEFORMABLE_STACK_DEPTH	256
//#define DG_DEFORMABLE_PADDING				 (dgFloat32 (4.0f))
//#define DG_DEFORMABLE_INV_PADDING			 (dgFloat32 (1.0f) / DG_DEFORMABLE_PADDING)
//#define DG_DEFORMABLE_DEFAULT_PLASTICITY	 (dgFloat32 (0.3f))
//#define DG_DEFORMABLE_PLANE_DISTANCE_TOL	 (dgFloat32 (1.0e-4f))
//#define DG_DEFORMABLE_DEFAULT_SKIN_THICKNESS (dgFloat32 (5.0e-2f))



class dgCollisionDeformableSolidMesh::dgCluster
{
	public:
	dgCluster()
		:m_count(0)
		,m_points(NULL)
	{
	}

	~dgCluster()
	{
		if (m_points) {
			dgFree (m_points); 			
		}
	}

	dgInt32 m_count;
	dgInt32* m_points;

};

class dgCollisionDeformableSolidMesh::dgClusterBuilder: public dgList<dgCluster>
{
	public:
	dgClusterBuilder (dgMemoryAllocator* const allocator)
		:dgList<dgCluster>(allocator)
	{
	}
};

						   
dgCollisionDeformableSolidMesh::dgCollisionDeformableSolidMesh (dgWorld* const world, dgDeserialize deserialization, void* const userData)
	:dgCollisionDeformableMesh (world, deserialization, userData)
{
	dgAssert (0);
}


dgCollisionDeformableSolidMesh::dgCollisionDeformableSolidMesh(dgWorld* const world, dgMeshEffect* const mesh)
	:dgCollisionDeformableMesh (world, mesh, m_deformableSolidMesh)
	,m_posit(NULL)
	,m_shapePosit(NULL)
	,m_clusterWeight(NULL)
	,m_clusterAqqInv(NULL) 
	,m_clusterRotationInitialGuess(NULL) 
	,m_clusterCom0(NULL)
	,m_clusterMass(NULL)
	,m_clusterPosit(NULL)
	,m_clusterPositStart(NULL)
	,m_clustersCount(0)
	,m_stiffness(DG_DEFORMABLE_DEFAULT_STIFFNESS)
{
	m_rtti |= dgCollisionDeformableSolidMesh_RTTI;

	m_clustersCount = 1;
	m_shapePosit = (dgVector*) dgMallocStack (sizeof (dgVector) * m_particles.m_count);
	m_posit = (dgVector*) dgMallocStack (sizeof (dgVector) * m_particles.m_count);
	m_clusterPositStart = (dgInt32*) dgMallocStack (sizeof (dgInt32) * (m_particles.m_count + 1));

	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		m_shapePosit[i] = m_particles.m_posit[i];
	}
	CreateClusters(1, dgFloat32 (1.0e10f));
}

dgCollisionDeformableSolidMesh::dgCollisionDeformableSolidMesh (const dgCollisionDeformableSolidMesh& source)
	:dgCollisionDeformableMesh (source)
	,m_clustersCount(source.m_clustersCount)
	,m_stiffness(source.m_stiffness)
{
	m_rtti |= dgCollisionCompoundBreakable_RTTI;
	m_shapePosit = (dgVector*) dgMallocStack (sizeof (dgVector) * source.m_particles.m_count);
	m_posit = (dgVector*) dgMallocStack (sizeof (dgVector) * source.m_particles.m_count);
	m_clusterWeight = (dgFloat32*) dgMallocStack (sizeof (dgFloat32) * (source.m_clusterPositStart[source.m_particles.m_count]));
	m_clusterPosit = (dgInt32*) dgMallocStack (sizeof (dgInt32) * (source.m_clusterPositStart[source.m_particles.m_count]));
	m_clusterPositStart = (dgInt32*) dgMallocStack (sizeof (dgInt32) * (source.m_particles.m_count + 1));
	
	m_clusterAqqInv = (dgMatrix*) dgMallocStack (sizeof (dgMatrix) * m_clustersCount);
	m_clusterRotationInitialGuess = (dgMatrix*) dgMallocStack (sizeof (dgMatrix) * m_clustersCount);
	m_clusterCom0 = (dgVector*) dgMallocStack (sizeof (dgVector) * m_clustersCount);
	m_clusterMass = (dgFloat32*) dgMallocStack (sizeof (dgVector) * m_clustersCount);

	memcpy (m_posit, source.m_posit, sizeof (dgVector) * source.m_particles.m_count);
	memcpy (m_shapePosit, source.m_shapePosit, sizeof (dgVector) * source.m_particles.m_count);

	memcpy (m_clusterAqqInv, source.m_clusterAqqInv, sizeof (dgMatrix) * m_clustersCount);
	memcpy (m_clusterRotationInitialGuess, source.m_clusterRotationInitialGuess, sizeof (dgMatrix) * m_clustersCount);
	memcpy (m_clusterCom0, source.m_clusterCom0, sizeof (dgVector) * m_clustersCount);
	memcpy (m_clusterMass, source.m_clusterMass, sizeof (dgFloat32) * m_clustersCount);

	memcpy (m_clusterWeight, source.m_clusterWeight, sizeof (dgFloat32) * (source.m_clusterPositStart[source.m_particles.m_count]));
	memcpy (m_clusterPosit, source.m_clusterPosit, sizeof (dgInt32) * (source.m_clusterPositStart[source.m_particles.m_count]));
	memcpy (m_clusterPositStart, source.m_clusterPositStart, sizeof (dgInt32) * (source.m_particles.m_count + 1));
}


dgCollisionDeformableSolidMesh::~dgCollisionDeformableSolidMesh(void)
{
	if (m_shapePosit) {
		dgFree (m_shapePosit);
		dgFree (m_posit);
		dgFree (m_clusterWeight);
		dgFree (m_clusterPosit);
		dgFree (m_clusterPositStart);
		dgFree (m_clusterAqqInv); 
		dgFree (m_clusterRotationInitialGuess); 
		dgFree (m_clusterCom0);
		dgFree (m_clusterMass);
	}
}


void dgCollisionDeformableSolidMesh::CreateClusters (dgInt32 count, dgFloat32 overlapingWidth)
{
	if (m_clusterPosit) {
		dgFree (m_clusterAqqInv); 
		dgFree (m_clusterRotationInitialGuess); 
		dgFree (m_clusterCom0);
		dgFree (m_clusterMass);
		dgFree (m_clusterWeight);
		dgFree (m_clusterPosit);
	}

	if (count <= 1) {
		// special case of only one region
		m_clustersCount = 1;
		m_clusterAqqInv = (dgMatrix*) dgMallocStack (sizeof (dgMatrix) * m_clustersCount);
		m_clusterRotationInitialGuess = (dgMatrix*) dgMallocStack (sizeof (dgMatrix) * m_clustersCount);
		m_clusterCom0 = (dgVector*) dgMallocStack (sizeof (dgVector) * m_clustersCount);
		m_clusterMass = (dgFloat32*) dgMallocStack (sizeof (dgFloat32) * m_clustersCount);
		m_clusterWeight = (dgFloat32*) dgMallocStack (sizeof (dgFloat32) * (m_particles.m_count));
		m_clusterPosit = (dgInt32*) dgMallocStack (sizeof (dgInt32) * (m_particles.m_count));
		for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
			m_clusterPositStart[i] = i;
			m_clusterPosit[i] = 0;
			m_clusterWeight[i] = dgFloat32 (1.0f);
		}
		m_clusterPositStart[m_particles.m_count] = m_particles.m_count;
	} else {
		dgClusterBuilder clusterList (GetAllocator());

		dgCluster& cluster = clusterList.Append()->GetInfo();

		cluster.m_count = m_particles.m_count;
		cluster.m_points = ((dgInt32*) dgMallocStack (sizeof (dgUnsigned32) * cluster.m_count));
		for (dgInt32 i = 0; i < cluster.m_count; i ++) {
			cluster.m_points[i] = i;
		}

		dgClusterBuilder::dgListNode* nextNode = NULL;
		for (dgClusterBuilder::dgListNode* node = clusterList.GetFirst(); node && (clusterList.GetCount() < count); node = nextNode) {
			dgCluster& cluster = node->GetInfo();
			dgVector median (dgFloat32 (0.0f));
			dgVector varian (dgFloat32 (0.0f));

			for (dgInt32 i = 0; i < cluster.m_count; i ++) {
				const dgVector& p = m_shapePosit[cluster.m_points[i]];
				median += p;
				varian += p.CompProduct4(p);
			}

			varian = varian.Scale4 (dgFloat32 (cluster.m_count)) - median.CompProduct4(median);

			dgInt32 index = 0;
			dgFloat32 maxVarian = dgFloat32 (-1.0e10f);
			for (dgInt32 i = 0; i < 3; i ++) {
				if (varian[i] > maxVarian) {
					index = i;
					maxVarian = varian[i];
				}
			}

			dgVector center = median.Scale4 (dgFloat32 (1.0f) / dgFloat32 (cluster.m_count));
			dgFloat32 test = center[index];

			dgInt32 i0 = 0;
			dgInt32 i1 = cluster.m_count - 1;
			do {    

				for (; i0 <= i1; i0 ++) {
					const dgVector& p = m_shapePosit[cluster.m_points[i0]];
					if (p[index] > test) {
						break;
					}
				}

				for (; i1 >= i0; i1 --) {
					const dgVector& p = m_shapePosit[cluster.m_points[i1]];
					if (p[index] < test) {
						break;
					}
				}

				if (i0 < i1)	{
					dgSwap(cluster.m_points[i0], cluster.m_points[i1]);
					i0++; 
					i1--;
				}

			} while (i0 <= i1);

			dgInt32 middle = i0 + 1;

			dgInt32 leftSideOvelap = 0;
			dgFloat32 leftBarrier = test + overlapingWidth;
			for (dgInt32 i = middle; i < cluster.m_count; i ++) {
				const dgVector& p = m_shapePosit[cluster.m_points[i]];
				leftSideOvelap += (p[index] < leftBarrier) ? 1 : 0;
			}

			dgInt32 rightSideOvelap = 0;
			dgFloat32 rightBarrier = test - overlapingWidth;
			for (dgInt32 i = 0; i < middle; i ++) {
				const dgVector& p = m_shapePosit[cluster.m_points[i]];
				rightSideOvelap += (p[index] > rightBarrier) ? 1 : 0;
			}

			if (rightSideOvelap || leftSideOvelap) {
				dgCluster& leftCluster = clusterList.Append()->GetInfo();
				leftCluster.m_count = middle + leftSideOvelap;
				leftCluster.m_points = ((dgInt32*) dgMallocStack (sizeof (dgUnsigned32) * leftCluster.m_count));

				dgInt32 j = 0;
				for (dgInt32 i = 0; i < middle; i ++) {
					leftCluster.m_points[j] = cluster.m_points[i];
					j ++;
				}
				
				for (dgInt32 i = middle; i < cluster.m_count; i ++) {
					const dgVector& p = m_shapePosit[cluster.m_points[i]];
					if (p[index] < leftBarrier) {
						leftCluster.m_points[j] = cluster.m_points[i];
						j ++;
						dgAssert (j <= leftCluster.m_count);
					}
				}

				j = 0;
				dgCluster& rightCluster = clusterList.Append()->GetInfo();
				rightCluster.m_count = cluster.m_count - middle + rightSideOvelap;
				rightCluster.m_points = ((dgInt32*) dgMallocStack (sizeof (dgUnsigned32) * rightCluster.m_count));
				for (dgInt32 i = middle; i < cluster.m_count; i ++) {
					rightCluster.m_points[j] = cluster.m_points[i];
					j ++;
				}
				for (dgInt32 i = 0; i < middle; i ++) {
					const dgVector& p = m_shapePosit[cluster.m_points[i]];
					if (p[index] > rightBarrier) {
						rightCluster.m_points[j] = cluster.m_points[i];
						j ++;
						dgAssert (j <= rightCluster.m_count);
					}
				}
				nextNode = node->GetNext();
				clusterList.Remove(node);
			} else {
				dgAssert(0);
			}
		}

		m_clustersCount = clusterList.GetCount();
		m_clusterAqqInv = (dgMatrix*) dgMallocStack (sizeof (dgMatrix) * m_clustersCount);
		m_clusterRotationInitialGuess = (dgMatrix*) dgMallocStack (sizeof (dgMatrix) * m_clustersCount);
		m_clusterCom0 = (dgVector*) dgMallocStack (sizeof (dgVector) * m_clustersCount);
		m_clusterMass = (dgFloat32*) dgMallocStack (sizeof (dgFloat32) * m_clustersCount);

		dgInt32 poolSize = 0;
		dgStack<dgInt32> particleClusterCountPool(m_particles.m_count);
		dgInt32* const particleClusterCount = &particleClusterCountPool[0];
		memset (particleClusterCount, 0, particleClusterCountPool.GetSizeInBytes());
		for (dgClusterBuilder::dgListNode* node = clusterList.GetFirst(); node; node = node->GetNext()) {
			dgCluster& cluster = node->GetInfo();
			poolSize += cluster.m_count;
			for (dgInt32 i = 0; i < cluster.m_count; i ++) {
				dgInt32 j = cluster.m_points[i];
				particleClusterCount[j] ++;
			}
		}

		m_clusterWeight = (dgFloat32*) dgMallocStack (sizeof (dgFloat32) * poolSize);
		m_clusterPosit = (dgInt32*) dgMallocStack (sizeof (dgInt32) * poolSize);
		
		dgInt32 acc = 0;
		for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
			dgInt32 count = particleClusterCount[i];
			m_clusterPositStart[i] = acc;
			particleClusterCount[i] = acc;
			acc += count;
		}
		m_clusterPositStart[m_particles.m_count] = acc;

		dgInt32 clusterIndex = 0;
		for (dgClusterBuilder::dgListNode* node = clusterList.GetFirst(); node; node = node->GetNext()) {
			dgCluster& cluster = node->GetInfo();
			for (dgInt32 i = 0; i < cluster.m_count; i ++) {
				dgInt32 j = cluster.m_points[i];
				dgInt32 base = particleClusterCount[j];
				m_clusterPosit[base] = clusterIndex;
				m_clusterWeight[base] += dgFloat32 (1.0f);
				particleClusterCount[j] ++;
			}
			clusterIndex ++;
		}

		for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
			const dgInt32 start = m_clusterPositStart[i];
			const dgInt32 count = m_clusterPositStart[i + 1] - start;
			dgAssert (count);
			dgFloat32 weight = dgFloat32 (1.0f) / count;
			for (dgInt32 j = 0; j < count; j ++) {
				m_clusterWeight[start + j] = weight;
			}
		}
	}
}

void dgCollisionDeformableSolidMesh::InitClusters()
{
	dgStack<dgMatrix> covarianceMatrixPool(m_particles.m_count);
	dgMatrix* const covarianceMatrix = &covarianceMatrixPool[0];

	const dgFloat32* const masses = m_particles.m_unitMass;
	for (dgInt32 i = 0; i < m_clustersCount; i ++) {
		m_clusterCom0[i] = dgVector (dgFloat32 (0.0f));
		m_clusterMass[i] = dgFloat32 (0.0f);
		m_clusterRotationInitialGuess[i] = dgGetIdentityMatrix();
	}

	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		const dgVector& r = m_shapePosit[i];
		dgVector mr (r.Scale4(masses[i]));
		covarianceMatrix[i] = dgMatrix (mr, r);
		dgAssert (covarianceMatrix[i].TestSymetric3x3());

		const dgInt32 start = m_clusterPositStart[i];
		const dgInt32 count = m_clusterPositStart[i + 1] - start;
		for (dgInt32 j = 0; j < count; j ++) {
			dgInt32 index = m_clusterPosit[start + j];
			m_clusterCom0[index] += mr;
			m_clusterMass[index] += masses[i];
		}
	}

	for (dgInt32 i = 0; i < m_clustersCount; i ++) {
		dgVector mcr0 (m_clusterCom0[i]);
		m_clusterCom0[i] = mcr0.Scale4 (dgFloat32 (1.0f) / m_clusterMass[i]);
		m_clusterAqqInv[i] = dgMatrix (mcr0.CompProduct4(dgVector::m_negOne), m_clusterCom0[i]);
		dgAssert (m_clusterAqqInv[i].TestSymetric3x3());
	}


	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		const dgInt32 start = m_clusterPositStart[i];
		const dgInt32 count = m_clusterPositStart[i + 1] - start;

		for (dgInt32 j = 0; j < count; j ++) {
			dgInt32 index = m_clusterPosit[start + j];
			dgMatrix& covariance = m_clusterAqqInv[index];
			covariance.m_front += covarianceMatrix[i].m_front;
			covariance.m_up += covarianceMatrix[i].m_up;
			covariance.m_right += covarianceMatrix[i].m_right;
			dgAssert (covariance.TestSymetric3x3());
		}
	}

	for (dgInt32 i = 0; i < m_clustersCount; i ++) {
		dgMatrix& AqqInv = m_clusterAqqInv[i];
		dgAssert (AqqInv.TestSymetric3x3());
		AqqInv = AqqInv.Symetric3by3Inverse();
	}
}


void dgCollisionDeformableSolidMesh::Serialize(dgSerialize callback, void* const userData) const
{
	dgAssert (0);
}

dgInt32 dgCollisionDeformableSolidMesh::CalculateSignature () const
{
	dgAssert (0);
	return 0;
}

void dgCollisionDeformableSolidMesh::EndConfiguration ()
{
	dgAssert(0);
}

void dgCollisionDeformableSolidMesh::ConstraintParticle (dgInt32 particleIndex, const dgVector& posit, const dgBody* const body)
{
	dgAssert(0);
}


void dgCollisionDeformableSolidMesh::SetMass (dgFloat32 mass)
{
	if ((mass < dgFloat32 (1.0e5f)) && (mass > dgFloat32 (0.0f))) {
		mass = mass / m_particles.m_count;
		dgFloat32* const unitMass = m_particles.m_unitMass;
		for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
			unitMass[i] = mass;
		}

		InitClusters();
	}
}

void dgCollisionDeformableSolidMesh::SetMatrix(const dgMatrix& matrix)
{
   	dgAssert (m_myBody);
   	dgBody* const body = GetBody();
    dgMatrix globalMatrix (body->GetCollision()->GetLocalMatrix() * matrix);
    for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
        m_particles.m_posit[i] = m_shapePosit[i];
        m_posit[i] = matrix.TransformVector(m_shapePosit[i]) & dgVector::m_triplexMask;
    }
}

void dgCollisionDeformableSolidMesh::DebugCollision (const dgMatrix& matrix, dgCollision::OnDebugCollisionMeshCallback callback, void* const userData) const
{
	dgCollisionDeformableMesh::DebugCollision (matrix, callback, userData);
	if (m_onDebugDisplay) {
		dgStack<dgBigVector> points (m_particles.m_count);
		for (dgInt32 k = 0; k < m_clustersCount; k ++) {

			dgInt32 vertexCount = 0;
			for (dgInt32 i = 0; (i < m_particles.m_count); i ++) {
				const dgInt32 start = m_clusterPositStart[i];
				const dgInt32 count = m_clusterPositStart[i + 1] - start;
				for (dgInt32 j = 0; j < count; j ++) {
					dgInt32 index = m_clusterPosit[start + j];
					if (index == k) {
						points[vertexCount] = m_posit[i] + m_basePosit;
						vertexCount ++;
						break;
					}
				}
			}

			dgConvexHull3d convexHull (GetAllocator(), &points[0].m_x, sizeof (dgBigVector), vertexCount, dgFloat32 (0.0f));
			for (dgConvexHull3d::dgListNode* faceNode = convexHull.GetFirst(); faceNode; faceNode = faceNode->GetNext()) {
				const dgConvexHull3DFace& face = faceNode->GetInfo();

				dgTriplex points[3];
				for (dgInt32 l = 0; l < 3; l ++) {
					dgVector p (convexHull.GetVertex(face.m_index[l]));
					points[l].m_x = p.m_x;
					points[l].m_y = p.m_y;
					points[l].m_z = p.m_z;
				}
				m_onDebugDisplay (userData, 3, &points[0].m_x, k);
			}
		}
	}
}

void dgCollisionDeformableSolidMesh::ApplyExternalForces (dgFloat32 timestep)
{
	dgAssert (m_myBody);

	dgBody* const body = GetBody();


	dgAssert (body->GetMass().m_w > dgFloat32 (0.0f));
	dgAssert (body->GetMass().m_w < dgFloat32 (1.0e5f));
	const dgMatrix& matrix = body->GetCollision()->GetGlobalMatrix();

	dgFloat32 invMass = body->GetInvMass().m_w;
	dgVector velocyStep (body->GetForce().Scale4(invMass * timestep));
//velocyStep = dgVector(0.0f);

	dgVector* const veloc = m_particles.m_veloc;
	dgFloat32* const unitMass = m_particles.m_unitMass;

/*
invMass = 0;
dgVector w (0.0f, 0.0f, 1.0f, 0.0f);
for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
unitMass[i] = 1.0f;
veloc[i] = w * m_posit[i];
invMass += unitMass[i];
}
invMass = 1.0f / invMass;
*/

	dgVector com (dgFloat32 (0.0f));
	dgVector comVeloc (dgFloat32 (0.0f));
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		dgVector mass (unitMass[i]);
        const dgVector& p = m_posit[i]; 
		veloc[i] += velocyStep;
		com += p.CompProduct4(mass);
		comVeloc += veloc[i].CompProduct4(mass);
	}
	com = com.Scale4(invMass);
	comVeloc = comVeloc.Scale4(invMass);


	const dgMatrix& indentity = dgGetIdentityMatrix();
	dgMatrix inertiaMatrix (dgGetZeroMatrix());
	inertiaMatrix.m_posit = dgVector::m_wOne;
	dgVector comAngularMomentum (dgFloat32 (0.0f));
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		dgVector mass (unitMass[i]);
		dgVector r (m_posit[i] - com);
		dgVector mr (r.CompProduct4(mass));
		dgVector relVeloc (veloc[i] - comVeloc);
		comAngularMomentum += mr * relVeloc;

		dgMatrix inertia (mr, r);
		dgVector diagInertia (mr.DotProduct4(r));

		inertiaMatrix.m_front += (indentity.m_front.CompProduct4(diagInertia) - inertia.m_front); 
		inertiaMatrix.m_up += (indentity.m_up.CompProduct4(diagInertia) - inertia.m_up); 
		inertiaMatrix.m_right += (indentity.m_right.CompProduct4(diagInertia) - inertia.m_right);
		dgAssert (inertiaMatrix.TestSymetric3x3());
	}

dgVector damp (0.3f); 
	dgMatrix invInertia (inertiaMatrix.Symetric3by3Inverse());
	dgVector omega (invInertia.RotateVector(comAngularMomentum));
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		dgVector r (m_posit[i] - com);
		dgVector deltaVeloc (comVeloc + omega * r - veloc[i]);
		veloc[i] += deltaVeloc.CompProduct4(damp);
	}

//Sleep (50);

	dgMatrix tmp;
	dgMatrix transform;
	dgVector scale;
	inertiaMatrix.PolarDecomposition (transform, scale, tmp, matrix);
	body->GetCollision()->SetGlobalMatrix(transform);
	body->SetMatrixOriginAndRotation(body->GetCollision()->GetLocalMatrix().Inverse() * transform);
    body->SetVelocity(comVeloc);
    body->SetOmega(omega);
}

void dgCollisionDeformableSolidMesh::ResolvePositionsConstraints (dgFloat32 timestep)
{
	dgAssert (m_myBody);

	dgInt32 strideInBytes = sizeof (dgVector) * m_clustersCount + sizeof (dgMatrix) * m_clustersCount + sizeof (dgMatrix) * m_particles.m_count;
	m_world->m_solverMatrixMemory.ExpandCapacityIfNeessesary (1, strideInBytes);
	dgVector* const regionCom = (dgVector*)&m_world->m_solverMatrixMemory[0];
	dgMatrix* const sumQiPi = (dgMatrix*) &regionCom[m_clustersCount];
	dgMatrix* const covarianceMatrix = (dgMatrix*) &sumQiPi[m_clustersCount];

	const dgFloat32* const masses = m_particles.m_unitMass;
	dgVector zero (dgFloat32 (0.0f));
	for (dgInt32 i = 0; i < m_clustersCount; i ++) {
		regionCom[i] = zero;
	}

	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		dgVector mass (masses[i]);
		const dgVector& r = m_posit[i];
		const dgVector& r0 = m_shapePosit[i];
		dgVector mr (r.Scale4(masses[i]));
		covarianceMatrix[i] = dgMatrix (r0, mr);
	
		const dgInt32 start = m_clusterPositStart[i];
		const dgInt32 count = m_clusterPositStart[i + 1] - start;
		for (dgInt32 j = 0; j < count; j ++) {
			dgInt32 index = m_clusterPosit[start + j];
			regionCom[index] += mr;
		}
	}

	for (dgInt32 i = 0; i < m_clustersCount; i ++) {
		dgVector mcr (regionCom[i]);
		regionCom[i] = mcr.Scale4 (dgFloat32 (1.0f) / m_clusterMass[i]);
		const dgVector& cr0 = m_clusterCom0[i];
		sumQiPi[i] = dgMatrix (cr0, mcr.CompProduct4(dgVector::m_negOne));
	}

	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		const dgInt32 start = m_clusterPositStart[i];
		const dgInt32 count = m_clusterPositStart[i + 1] - start;
		const dgMatrix& covariance = covarianceMatrix[i];
		for (dgInt32 j = 0; j < count; j ++) {
			dgInt32 index = m_clusterPosit[start + j];
			dgMatrix& QiPi = sumQiPi[index];
			QiPi.m_front += covariance.m_front;
			QiPi.m_up += covariance.m_up;
			QiPi.m_right += covariance.m_right;
		}
	}

dgVector beta0 (dgFloat32 (0.93f));
//dgVector beta0 (dgFloat32 (0.0f));
dgVector beta1 (dgVector::m_one - beta0);
	dgFloat32 stiffness = dgFloat32 (0.3f);

	for (dgInt32 i = 0; i < m_clustersCount; i ++) {
		dgMatrix& QiPi = sumQiPi[i];

		dgMatrix S (QiPi * QiPi.Transpose4X4());
		dgVector eigenValues;

		S.EigenVectors (eigenValues, m_clusterRotationInitialGuess[i]);
		m_clusterRotationInitialGuess[i] = S;

#ifdef _DEBUG_____ 
		dgMatrix P0 (QiPi * QiPi.Transpose4X4());
		dgMatrix D (dgGetIdentityMatrix());
		D[0][0] = eigenValues[0];
		D[1][1] = eigenValues[1];
		D[2][2] = eigenValues[2];
		dgMatrix P1 (S.Transpose4X4() * D * S);
		dgAssert (P0.TestSymetric3x3());
		dgAssert (P1.TestSymetric3x3());

		dgMatrix xx (P1 * P0.Symetric3by3Inverse());
		dgAssert (dgAbsf (xx[0][0] - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));
		dgAssert (dgAbsf (xx[1][1] - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));
		dgAssert (dgAbsf (xx[2][2] - dgFloat32 (1.0f)) < dgFloat32 (1.0e-3f));
		dgAssert (dgAbsf (xx[0][1]) < dgFloat32 (1.0e-3f));
		dgAssert (dgAbsf (xx[0][2]) < dgFloat32 (1.0e-3f));
		dgAssert (dgAbsf (xx[1][0]) < dgFloat32 (1.0e-3f));
		dgAssert (dgAbsf (xx[1][2]) < dgFloat32 (1.0e-3f));
		dgAssert (dgAbsf (xx[2][0]) < dgFloat32 (1.0e-3f));
		dgAssert (dgAbsf (xx[2][1]) < dgFloat32 (1.0e-3f));
#endif

		eigenValues = eigenValues.InvSqrt();

		dgMatrix m;
		m.m_front = S.m_front.CompProduct4(eigenValues.BroadcastX());
		m.m_up    = S.m_up.CompProduct4(eigenValues.BroadcastY());
		m.m_right = S.m_right.CompProduct4(eigenValues.BroadcastZ());
		m.m_posit = dgVector::m_wOne;
		dgMatrix invS (S.Transpose4X4() * m);
		dgMatrix R (invS * QiPi);
		dgMatrix A (m_clusterAqqInv[i] * QiPi);

		QiPi.m_front = A.m_front.CompProduct4(beta0) + R.m_front.CompProduct4(beta1);
		QiPi.m_up = A.m_up.CompProduct4(beta0) + R.m_up.CompProduct4(beta1);
		QiPi.m_right = A.m_right.CompProduct4(beta0) + R.m_right.CompProduct4(beta1);
	}

	dgVector invTimeScale (stiffness / timestep);
	dgVector* const veloc = m_particles.m_veloc;

	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		const dgInt32 start = m_clusterPositStart[i];
		const dgInt32 count = m_clusterPositStart[i + 1] - start;

		dgVector v (dgFloat32 (0.0f));
		const dgVector& p = m_posit[i];
		const dgVector& p0 = m_shapePosit[i];
		for (dgInt32 j = 0; j < count; j ++) {
			dgInt32 index = m_clusterPosit[start + j];
			const dgMatrix& matrix = sumQiPi[index];
			dgVector gi (matrix.RotateVector(p0 - m_clusterCom0[index]) + regionCom[index]);
			v += ((gi - p).CompProduct4(invTimeScale).Scale4 (m_clusterWeight[index]));
		}
		veloc[i] += v;
	}


// resolve collisions here
//for now just a hack a collision plane until I get the engine up an running
for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
	dgVector p (m_basePosit + m_posit[i].m_y);
	if (p.m_y < dgFloat32 (0.0f)) {
		m_posit[i].m_y = -m_basePosit.m_y;
		veloc[i].m_y = dgFloat32 (0.0f);
	}
}

	dgVector time (timestep);
	dgVector minBase(dgFloat32 (1.0e10f));
    dgVector minBox (dgFloat32 (1.0e10f));
    dgVector maxBox (dgFloat32 (-1.0e10f));
    dgMatrix matrix (m_myBody->GetCollision()->GetGlobalMatrix().Inverse());
	for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
		m_posit[i] += veloc[i].CompProduct4 (time);
        m_particles.m_posit[i] = matrix.TransformVector(m_posit[i] + m_basePosit);
		minBase = minBase.GetMin (m_posit[i]);
        minBox = minBox.GetMin(m_particles.m_posit[i]);
        maxBox = maxBox.GetMax(m_particles.m_posit[i]);
	}

	minBase = minBase.Floor();
	dgVector mask ((minBase < dgVector (dgFloat32 (0.0f))) | (minBase >= dgVector (dgFloat32 (DG_SOFTBODY_BASE_SIZE))));
	dgInt32 test = mask.GetSignMask();
	if (test & 0x07) {
		dgVector offset (((minBase < dgVector (dgFloat32 (0.0f))) & dgVector (dgFloat32 (DG_SOFTBODY_BASE_SIZE/2))) + 
			             ((minBase >= dgVector (dgFloat32 (DG_SOFTBODY_BASE_SIZE))) & dgVector (dgFloat32 (-DG_SOFTBODY_BASE_SIZE/2))));
		m_basePosit -= offset;
		for (dgInt32 i = 0; i < m_particles.m_count; i ++) {
			m_posit[i] += offset;
		}
	}

	// integrate each particle by the deformation velocity, also calculate the new com
	// calculate the new body average velocity
//	if (m_myBody->m_matrixUpdate) {
//		myBody->m_matrixUpdate (*myBody, myBody->m_matrix, threadIndex);
//	}
	// the collision changed shape, need to update spatial structure 
//	UpdateCollision ();
//	SetCollisionBBox (m_rootNode->m_minBox, m_rootNode->m_maxBox);
    SetCollisionBBox (minBox, maxBox);
}
