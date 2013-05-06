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
#include "dgWorld.h"
#include "dgCollisionBox.h"
#include "dgCollisionMesh.h"
#include "dgMinkowskiConv.h"
#include "dgCollisionConvex.h"
#include "dgCollisionSphere.h"
#include "dgCollisionCapsule.h"
#include "dgWorldDynamicUpdate.h"

#ifdef DG_BUILD_CORE_200_COLLISION

void dgContactSolver::CalculateVelocitiesSimd (dgFloat32 timestep) 
{
	dgAssert (0);
/*
	dgVector refOmega;
	dgVector floatOmega;

	m_referenceBody->CalculateContinueVelocitySimd(timestep, m_referenceBodyVeloc, refOmega);
	m_floatingBody->CalculateContinueVelocitySimd (timestep, m_floatingBodyVeloc, floatOmega);
	dgSimd vRel ((dgSimd&) m_floatingBodyVeloc - (dgSimd&)m_referenceBodyVeloc);
	m_localRelVeloc = m_proxy->m_referenceMatrix.UnrotateVectorSimd(vRel);
*/
}


void dgContactSolver::CalcSupportVertexSimd (const dgVector& dir, dgInt32 entry)
{
	dgAssert ((dir % dir) > dgFloat32 (0.999f));
	dgVector p (m_referenceCollision->SupportVertexSimd (dir));
	dgVector dir1 (m_matrix.UnrotateVectorSimd(dgSimd(dgFloat32 (-1.0f)) * (dgSimd&)dir));
	dgVector q (m_matrix.TransformVectorSimd(m_floatingcollision->SupportVertexSimd (dir1)));
	(dgSimd&)m_hullVertex[entry] = (dgSimd&)p - (dgSimd&)q;
	(dgSimd&)m_averVertex[entry] = (dgSimd&)p + (dgSimd&)q;
}



dgContactSolver::dgMinkFace* dgContactSolver::CalculateClipPlaneSimd ()
{
	dgFloat32 cyclingMem[4];
	dgMinkFace* stackPool[128];
	dgMinkFace* deadFaces[128];
	SilhouetteFaceCap sillueteCap[128];
	dgVector diff[3];
	dgVector aver[3];
	dgInt8  buffer[DG_HEAP_EDGE_COUNT * (sizeof (dgFloat32) + sizeof (dgMinkFace *))];
	dgClosestFace heapSort (buffer, sizeof (buffer));

	m_planeIndex = 4;
	dgMinkFace* closestFace = NULL;
	m_freeFace = NULL;

	dgAssert (m_vertexIndex == 4);
	for (dgInt32 i = 0; i < 4; i ++) {
		dgMinkFace* face = &m_simplex[i];
		face->m_inHeap = 0;
		face->m_isActive = 1;
		if (CalcFacePlaneSimd (face)) {
			face->m_inHeap = 1;
			heapSort.Push(face, face->m_w);
		}
	}

	dgInt32 cycling = 0;
	cyclingMem[0] = dgFloat32 (1.0e10f);
	cyclingMem[1] = dgFloat32 (1.0e10f);
	cyclingMem[2] = dgFloat32 (1.0e10f);
	cyclingMem[3] = dgFloat32 (1.0e10f);

	dgFloat32 minValue = dgFloat32 ( 1.0e10f);
	dgPlane bestPlane (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f),dgFloat32 (0.0f));
	diff[0] = bestPlane;
	diff[1] = bestPlane;
	diff[2] = bestPlane;
	aver[0] = bestPlane;
	aver[1] = bestPlane;
	aver[2] = bestPlane;

	while (heapSort.GetCount()) {
		dgMinkFace* face = heapSort[0];
		face->m_inHeap = 0;
		heapSort.Pop();

		if (face->m_isActive) {
			const dgPlane& plane = *face;

			CalcSupportVertexSimd (plane, m_vertexIndex);
			const dgVector& p = m_hullVertex[m_vertexIndex];
			dgFloat32 dist = plane.Evalue (p);
			m_vertexIndex ++;

			if (m_vertexIndex > 16) {
				if (dist < minValue) {
					if (dist >= dgFloat32 (0.0f)) {
						minValue = dist;
						bestPlane = plane;

						diff[0] = m_hullVertex[face->m_vertex[0]];
						aver[0] = m_averVertex[face->m_vertex[0]];

						diff[1] = m_hullVertex[face->m_vertex[1]];
						aver[1] = m_averVertex[face->m_vertex[1]];

						diff[2] = m_hullVertex[face->m_vertex[2]];
						aver[2] = m_averVertex[face->m_vertex[2]];
					}
				}
			}

			cyclingMem[cycling] = dist;
			cycling = (cycling + 1) & 3;
			dgInt32 cyclingEntry = 0;
			for (; cyclingEntry < 4; cyclingEntry ++) {
				if (dgAbsf (dist - cyclingMem[cyclingEntry]) > dgFloat32 (1.0e-6f)) {
					break;
				}
			}
			if (cyclingEntry == 4) {
				dist = dgFloat32 (0.0f);
			}


			if ((m_vertexIndex > DG_MINK_MAX_POINTS) || (m_planeIndex > DG_MINK_MAX_FACES) || (heapSort.GetCount() > (DG_HEAP_EDGE_COUNT - 24))) {
				dgPlane& plane = *face;
				plane = bestPlane;

				dgInt32 index = face->m_vertex[0];
				face->m_vertex[0] = 0;
				m_hullVertex[index] = diff[0];
				m_averVertex[index] = aver[0];

				index = face->m_vertex[1];
				face->m_vertex[1] = 1;
				m_hullVertex[index] = diff[1];
				m_averVertex[index] = aver[1];

				index = face->m_vertex[2];
				face->m_vertex[2] = 2;
				m_hullVertex[index] = diff[2];
				m_averVertex[index] = aver[2];
				dist = dgFloat32 (0.0f);
			}

			if (dist < (dgFloat32 (DG_IMPULSIVE_CONTACT_PENETRATION) / dgFloat32 (16.0f))) {
				dgAssert (m_planeIndex <= DG_MINK_MAX_FACES_SIZE);
				dgAssert (heapSort.GetCount() <= DG_HEAP_EDGE_COUNT);
				closestFace = face;
				break;
			} else if (dist > dgFloat32 (0.0f)) {
				dgAssert (face->m_inHeap == 0);

				dgInt32 stack = 0;
				dgInt32 deadCount = 1;
				dgMinkFace* silhouette = NULL;
				deadFaces[0] = face;
				closestFace = face;
				face->m_isActive = 0;
				for (dgInt32 i = 0; i < 3; i ++) {
					dgMinkFace* const adjacent = &m_simplex[face->m_adjancentFace[i]];
					dgAssert (adjacent->m_isActive);
					dist = adjacent->Evalue (p);  
					if (dist > dgFloat32 (0.0f)) { 
						adjacent->m_isActive = 0;
						stackPool[stack] = adjacent;
						deadFaces[deadCount] = adjacent;
						stack ++;
						deadCount ++;
					} else {
						silhouette = adjacent;
					}
				}
				while (stack) {
					stack --;
					face = stackPool[stack];
					for (dgInt32 i = 0; i < 3; i ++) {
						dgMinkFace* const adjacent = &m_simplex[face->m_adjancentFace[i]];
						if (adjacent->m_isActive){
							dist = adjacent->Evalue (p);  
							if (dist > dgFloat32 (0.0f)) { 
								adjacent->m_isActive = 0;
								stackPool[stack] = adjacent;
								deadFaces[deadCount] = adjacent;
								stack ++;
								deadCount ++;
								dgAssert (stack < dgInt32 (sizeof (stackPool) / sizeof (stackPool[0])));
								dgAssert (deadCount < dgInt32 (sizeof (deadFaces) / sizeof (deadFaces[0])));

							} else {
								silhouette = adjacent;
							}
						}
					}
				}

				if (!silhouette) {
					closestFace = face;
					break;
				}
				// build silhouette						
				dgAssert (silhouette);
				dgAssert (silhouette->m_isActive);

				dgInt32 i2 = (m_vertexIndex - 1);
				dgMinkFace* const lastSilhouette = silhouette;
				dgAssert ((silhouette->m_adjancentFace[0] != silhouette->m_adjancentFace[1]) &&
					(silhouette->m_adjancentFace[0] != silhouette->m_adjancentFace[2]) &&
					(silhouette->m_adjancentFace[1] != silhouette->m_adjancentFace[2]));


				dgInt32 adjacentIndex = DG_GETADJACENTINDEX_ACTIVE (silhouette);
				face = NewFace();
				dgInt32 i0 = silhouette->m_vertex[adjacentIndex];
				dgInt32 i1 = silhouette->m_vertex[adjacentIndex + 1];

				face->m_vertex[0] = dgInt16 (i1);
				face->m_vertex[1] = dgInt16 (i0);
				face->m_vertex[2] = dgInt16 (i2);
				face->m_vertex[3] = face->m_vertex[0];
				face->m_adjancentFace[0] = dgInt16 (silhouette - m_simplex);
				face->m_inHeap = 0; 
				face->m_isActive = 1; 

				sillueteCap[0].m_face = face;
				sillueteCap[0].m_faceCopling = &silhouette->m_adjancentFace[adjacentIndex];
				dgInt32 silhouetteCapCount = 1;
				dgAssert (silhouetteCapCount < dgInt32 (sizeof (sillueteCap) / sizeof (sillueteCap[0])));
				do {
					silhouette = &m_simplex[silhouette->m_adjancentFace[adjacentIndex]];
					adjacentIndex = (DG_GETADJACENTINDEX_VERTEX(silhouette, i0)); 
				} while (!silhouette->m_isActive);

				dgMinkFace* prevFace = face;
				dgMinkFace* const firstFace = face;
				dgInt32 lastSilhouetteVertex = i0;
				dgInt32 prevEdgeIndex = dgInt32 (face - m_simplex);
				do {
					dgAssert ((silhouette->m_adjancentFace[0] != silhouette->m_adjancentFace[1]) &&
						(silhouette->m_adjancentFace[0] != silhouette->m_adjancentFace[2]) &&
						(silhouette->m_adjancentFace[1] != silhouette->m_adjancentFace[2]));


					adjacentIndex = adjacentIndex ? adjacentIndex - 1 : 2;

					face = NewFace();
					i0 = silhouette->m_vertex[adjacentIndex];
					i1 = silhouette->m_vertex[adjacentIndex + 1];

					face->m_vertex[0] = dgInt16 (i1);
					face->m_vertex[1] = dgInt16 (i0);
					face->m_vertex[2] = dgInt16 (i2);
					face->m_vertex[3] = face->m_vertex[0];
					face->m_adjancentFace[0] = dgInt16 (silhouette - m_simplex);
					face->m_adjancentFace[2] = dgInt16 (prevEdgeIndex);
					face->m_inHeap = 0; 
					face->m_isActive = 1; 

					prevEdgeIndex = dgInt32 (face - m_simplex);
					prevFace->m_adjancentFace[1] = dgInt16 (prevEdgeIndex);
					prevFace = face;

					sillueteCap[silhouetteCapCount].m_face = face;
					sillueteCap[silhouetteCapCount].m_faceCopling = &silhouette->m_adjancentFace[adjacentIndex];
					silhouetteCapCount ++;
					dgAssert (silhouetteCapCount < dgInt32 (sizeof (sillueteCap) / sizeof (sillueteCap[0])));

					do {
						silhouette = &m_simplex[silhouette->m_adjancentFace[adjacentIndex]];
						adjacentIndex = (DG_GETADJACENTINDEX_VERTEX(silhouette, i0)); 
					} while (!silhouette->m_isActive);

				} while ((silhouette != lastSilhouette) || (silhouette->m_vertex[adjacentIndex ? adjacentIndex - 1 : 2] != lastSilhouetteVertex));
				firstFace->m_adjancentFace[2] = dgInt16 (prevEdgeIndex);
				prevFace->m_adjancentFace[1] = dgInt16 (firstFace - m_simplex);


				for (dgInt32 i = 0; i < deadCount; i ++) {
					if (!deadFaces[i]->m_inHeap){
						dgMinkFreeFace* const nextFreeFace = (dgMinkFreeFace*) deadFaces[i];
						nextFreeFace->m_next = m_freeFace;
						m_freeFace = nextFreeFace;
					}
				}

				while (heapSort.GetCount() && (!heapSort[0]->m_isActive)) {
					face = heapSort[0];
					heapSort.Pop();
					dgMinkFreeFace* const nextFreeFace = (dgMinkFreeFace*) face;
					nextFreeFace->m_next = m_freeFace;
					m_freeFace = nextFreeFace;
				}

				for (dgInt32 i = 0; i < silhouetteCapCount; i ++) {
					face = sillueteCap[i].m_face;
					*sillueteCap[i].m_faceCopling = dgInt16 (face - m_simplex);

					if (CalcFacePlane (face)) {
						face->m_inHeap = 1;
						heapSort.Push(face, face->m_w);
					}
				}
			}
		} else {
			dgAssert (0);
			dgMinkFreeFace* const nextFreeFace = (dgMinkFreeFace*) face;
			nextFreeFace->m_next = m_freeFace;
			m_freeFace = nextFreeFace;
		}
	}
	return closestFace;

}


dgInt32 dgContactSolver::HullHullContinueContactsSimd (dgFloat32& timeStep, dgContactPoint* const contactOut, dgInt32 contactID, dgInt32 maxContacts)
{
dgAssert (0);
// remember to complete simd conversion

	dgInt32 count = 0;
	dgMinkFace *face;
	dgMinkReturnCode code = CalcSeparatingPlaneSimd(face);

	m_lastFaceCode = code;
	if (code == dgMinkIntersecting) {
			face = CalculateClipPlaneSimd();
			if (face) {
//				if (conditionalContactCalculationAtOrigin) {
//					dgFloat32 projVeloc;
//					const dgPlane& plane = *face;
//					projVeloc = plane.DotProductSimd(m_localRelVeloc);
//					if (projVeloc >= dgFloat32 (0.0f)) {
//						return 0;
//					}
//				}
				if (maxContacts) {
					count = CalculateContactsSimd (face, contactID, contactOut, maxContacts);
					dgAssert (count <= maxContacts);
				}

				timeStep = dgFloat32 (0.0f);
			}

	} else if (code == dgMinkDisjoint) {
		dgVector saveHull[3];
		dgVector saveAver[3];

		dgAssert (face);
		dgFloat32 t0 = dgFloat32 (0.0f);
		dgInt32 i0 = face->m_vertex[0];
		dgInt32 i1 = face->m_vertex[1];
		dgInt32 i2 = face->m_vertex[2];
		dgVector plane ((m_hullVertex[i1] - m_hullVertex[i0]) * (m_hullVertex[i2] - m_hullVertex[i0]));
		dgAssert (plane % plane > dgFloat32 (0.0f));
		//			dgAssert (dgAbsf (plane % vRel) > dgFloat32 (0.0f));
		dgFloat32 projVeloc = plane.DotProductSimd(m_localRelVeloc);
		if (projVeloc >= dgFloat32 (-1.0e-24f)) {
			code = UpdateSeparatingPlaneSimd(face, dgVector (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f)));
			if (code != dgMinkDisjoint) {
				return 0;
			}
			dgAssert (code == dgMinkDisjoint);

			i0 = face->m_vertex[0];
			i1 = face->m_vertex[1];
			i2 = face->m_vertex[2];
			plane = (m_hullVertex[i1] - m_hullVertex[i0]) * (m_hullVertex[i2] - m_hullVertex[i0]);
			dgAssert (plane % plane > dgFloat32 (0.0f));
			projVeloc = plane.DotProductSimd(m_localRelVeloc);
			if (projVeloc > dgFloat32 (-1.0e-24f)) {
				//dgAssert ((plane % m_localRelVeloc) < dgFloat32 (0.0f));
				return 0;
			}

		}

		//timeOfImpact = - plane.m_w / (plane % vRel + dgFloat32 (1.0e-24f));
		//timeOfImpact = (plane % m_hullVertex[i0]) / (plane % m_localRelVeloc + dgFloat32 (1.0e-24f));
		dgFloat32 timeOfImpact = (plane.DotProductSimd (m_hullVertex[i0])) / (plane.DotProductSimd (m_localRelVeloc) + dgFloat32 (1.0e-24f));
		if (timeOfImpact > 0.0f) {
			saveHull[0] = m_hullVertex[i0];
			saveHull[1] = m_hullVertex[i1];
			saveHull[2] = m_hullVertex[i2];
			saveAver[0] = m_averVertex[i0];
			saveAver[1] = m_averVertex[i1];
			saveAver[2] = m_averVertex[i2];
			dgVector p1 (m_localRelVeloc.Scale (timeOfImpact));

			for (dgInt32 maxPasses = 0; (maxPasses < 32) && (timeOfImpact < timeStep) && (timeOfImpact > t0); maxPasses ++) {
				dgMinkFace* tmpFaceface;
				t0 = timeOfImpact;
				code = UpdateSeparatingPlaneSimd(tmpFaceface, p1);
				dgAssert (code != dgMinkError);
				if (code == dgMinkDisjoint) {
					dgAssert (tmpFaceface);

					face = tmpFaceface;
					i0 = face->m_vertex[0];
					i1 = face->m_vertex[1];
					i2 = face->m_vertex[2];
					//dgPlane plane (m_hullVertex[i0], m_hullVertex[i1], m_hullVertex[i2]);
					dgVector plane ((m_hullVertex[i1] - m_hullVertex[i0]) * (m_hullVertex[i2] - m_hullVertex[i0]));
					dgAssert (plane % plane > dgFloat32 (0.0f));

					dgFloat32 den = plane.DotProductSimd(m_localRelVeloc);
					if (den >= dgFloat32 (-1.0e-24f)) {
						code = UpdateSeparatingPlaneSimd(tmpFaceface, p1);
						dgAssert (code == dgMinkDisjoint);

						i0 = face->m_vertex[0];
						i1 = face->m_vertex[1];
						i2 = face->m_vertex[2];
						//dgPlane plane (m_hullVertex[i0], m_hullVertex[i1], m_hullVertex[i2]);
						dgVector plane ((m_hullVertex[i1] - m_hullVertex[i0]) * (m_hullVertex[i2] - m_hullVertex[i0]));
						dgAssert (plane % plane > dgFloat32 (0.0f));
						//							den = plane % m_localRelVeloc;
						den = plane.DotProductSimd(m_localRelVeloc);
						if (den > dgFloat32 (-1.0e-24f)) {
							return 0;
						}
					}

					saveHull[0] = m_hullVertex[i0];
					saveHull[1] = m_hullVertex[i1];
					saveHull[2] = m_hullVertex[i2];
					saveAver[0] = m_averVertex[i0];
					saveAver[1] = m_averVertex[i1];
					saveAver[2] = m_averVertex[i2];
					if (den < dgFloat32 (-1.0e-24f)) {
						//							timeOfImpact = (plane % m_hullVertex[i0]) / den;
						timeOfImpact = (plane.DotProductSimd(m_hullVertex[i0])) / den;
						if (timeOfImpact < 0.0f) {
							return 0;
						}
						dgAssert (timeOfImpact >= 0.0f);
						//dgAssert (timeOfImpact >= dgFloat32 (-1.0f));
						p1 = m_localRelVeloc.Scale (timeOfImpact);
					}
				}
			}

			if ((timeOfImpact <= timeStep) && (timeOfImpact >= dgFloat32 (0.0f))) {

				if (maxContacts) {
					count = CalculateContactsContinueSimd(contactID, contactOut, maxContacts, saveHull, saveAver, timeOfImpact);
					dgAssert(count <= maxContacts);
				}

				timeStep = timeOfImpact;
				if (count) {
					//dgVector step (vRel.Scale (- timeOfImpact * 0.5f));
					//dgVector step (refVeloc.Scale (timeOfImpact));
					dgVector step (m_referenceBodyVeloc.Scale (timeOfImpact));
					for (i0 = 0; i0 < count; i0 ++) {
						//contactOut[i0].m_point += step;
						(dgSimd&)contactOut[i0].m_point = (dgSimd&)contactOut[i0].m_point + (dgSimd&)step;
					}
				}
			}
		}
	}
	return count;
}

dgInt32 dgContactSolver::CalculateContactsSimd(dgMinkFace* const face, dgInt32 contacID, dgContactPoint* const contactOut, dgInt32 maxContacts)
{
	dgInt32 count = 0;
	// Get the contact form the last face
	const dgPlane& plane = *face;
	dgFloat32 penetration = plane.m_w - m_penetrationPadding;
	dgFloat32 dist = (plane % m_averVertex[face->m_vertex[0]]) * dgFloat32 (0.5f);
	const dgPlane clipPlane (plane.Scale(dgFloat32 (-1.0f)), dist);

	dgVector point1 (clipPlane.Scale (-clipPlane.m_w));
	dgVector* const shape1 = m_hullVertex;
	dgInt32 count1 = m_referenceCollision->CalculatePlaneIntersectionSimd (clipPlane, point1, shape1);
	if (!count1) {
		dgVector p1 (m_referenceCollision->SupportVertexSimd (clipPlane.Scale (dgFloat32 (-1.0f))));
		p1 += clipPlane.Scale (DG_ROBUST_PLANE_CLIP);
		count1 = m_referenceCollision->CalculatePlaneIntersectionSimd (clipPlane, p1, shape1);
		dgVector err (clipPlane.Scale (clipPlane % (point1 - p1)));
		for (dgInt32 i = 0; i < count1; i ++) {
			shape1[i] += err;
		}
	}

	dgAssert (penetration <= dgFloat32 (2.0e-1f));
	dist = dgMax (- (penetration + DG_IMPULSIVE_CONTACT_PENETRATION), dgFloat32 (0.0f));
	if (count1) {
		dgVector* const shape2 = &m_hullVertex[count1];
		const dgPlane clipPlane2 (m_matrix.UntransformPlane(clipPlane));

		dgVector point2 (clipPlane2.Scale (-clipPlane2.m_w));
		dgInt32 count2 = m_floatingcollision->CalculatePlaneIntersectionSimd (clipPlane2, point2, shape2);
		if (!count2) {
			dgVector p2 (m_floatingcollision->SupportVertexSimd (clipPlane2.Scale(dgFloat32 (-1.0f))));
			p2 += clipPlane2.Scale (DG_ROBUST_PLANE_CLIP);
			count2 = m_floatingcollision->CalculatePlaneIntersectionSimd (clipPlane2, p2, shape2);
			dgVector err (clipPlane2.Scale (clipPlane2 % (point2 - p2)));
			for (dgInt32 i = 0; i < count2; i ++) {
				shape2[i] += err;
			}
		}

		if (count2) {

			dgAssert (count1);
			dgAssert (count2);
			//const dgMatrix& matrix1 = m_proxy->m_referenceMatrix;
			const dgMatrix& matrix1 = m_proxy->m_referenceCollision->m_globalMatrix;

			if (count1 == 1) {
				count = 1;
				contactOut[0].m_point = matrix1.TransformVectorSimd (shape1[0]);
				contactOut[0].m_normal = matrix1.RotateVectorSimd (clipPlane);
				contactOut[0].m_userId = contacID;
				contactOut[0].m_penetration = dist;
			} else if (count2 == 1) {
				count = 1;
				//contactOut[0].m_point = matrix2.TransformVectorSimd (shape2[0]);
				contactOut[0].m_point =  matrix1.TransformVectorSimd (m_matrix.TransformVectorSimd(shape2[0]));
				contactOut[0].m_normal = matrix1.RotateVectorSimd (clipPlane);
				contactOut[0].m_userId = contacID;
				contactOut[0].m_penetration = dist;

			} else if ((count1 == 2) && (count2 == 2)) {
				dgVector p0 (shape1[0]); 
				dgVector p1 (shape1[1]); 
				dgVector q0 (m_matrix.TransformVectorSimd(shape2[0])); 
				dgVector q1 (m_matrix.TransformVectorSimd(shape2[1])); 

				dgAssert (0);// later make this part simd
				dgVector p10 (p1 - p0);
				dgVector q10 (q1 - q0);
				p10 = p10.Scale (dgRsqrt (p10 % p10 + dgFloat32 (1.0e-8f)));
				q10 = q10.Scale (dgRsqrt (q10 % q10 + dgFloat32 (1.0e-8f)));
				dgFloat32 dot = q10 % p10;
				if (dgAbsf (dot) > dgFloat32 (0.998f)) {
					dgFloat32 pl0 = p0 % p10;
					dgFloat32 pl1 = p1 % p10;
					dgFloat32 ql0 = q0 % p10;
					dgFloat32 ql1 = q1 % p10;
					if (pl0 > pl1) {
						dgSwap (pl0, pl1);
						dgSwap (p0, p1);
						p10 = p10.Scale (dgFloat32 (-1.0f));
					}
					if (ql0 > ql1) {
						dgSwap (ql0, ql1);
					}
					if ( !((ql0 > pl1) && (ql1 < pl0))) {
						dgFloat32 clip0 = (ql0 > pl0) ? ql0 : pl0;
						dgFloat32 clip1 = (ql1 < pl1) ? ql1 : pl1;

						count = 2;
						contactOut[0].m_point = p0 + p10.Scale (clip0 - pl0);
						contactOut[0].m_normal = matrix1.RotateVector (clipPlane);
						contactOut[0].m_userId = contacID;
						contactOut[0].m_penetration = dist;

						contactOut[1].m_point = p0 + p10.Scale (clip1 - pl0);
						contactOut[1].m_normal = matrix1.RotateVector (clipPlane);
						contactOut[1].m_userId = contacID;
						contactOut[1].m_penetration = dist;
					}

				} else {
					dgVector c0;
					dgVector c1;
					count = 1;
					dgRayToRayDistance (p0, p1, q0, q1, c0, c1);
					contactOut[0].m_point = (c0 + c1).Scale (dgFloat32 (0.5f));
					contactOut[0].m_normal = matrix1.RotateVector (clipPlane);
					contactOut[0].m_userId = contacID;
					contactOut[0].m_penetration = dist;
				}
				if (count > maxContacts) {
					count = maxContacts;
				}
				for (dgInt32 i = 0; i < count; i ++) {
					contactOut[i].m_point = matrix1.TransformVectorSimd(contactOut[i].m_point);
				}

			} else {

				m_matrix.TransformVectorsSimd(shape2, shape2, count2);
				count = CalculateConvexShapeIntersectionSimd (matrix1, clipPlane, dgUnsigned32 (contacID), dist, count1, shape1, count2, shape2, contactOut, maxContacts);
				if (!count) {
					count = CalculateContactAlternateMethod(face, contacID, contactOut, maxContacts);
				}
			}

			dgInt32 edgeContactFlag = (m_floatingcollision->IsEdgeIntersection() | m_referenceCollision->IsEdgeIntersection()) ? 1 : 0;
			for (dgInt32 i = 0; i < count; i ++) {
				contactOut[i].m_isEdgeContact = edgeContactFlag;
			}
		}
	}

	return count;
}


dgInt32 dgContactSolver::CalculateContactsContinueSimd(dgInt32 contacID, dgContactPoint* const contactOut, dgInt32 maxContacts, const dgVector* const diffPoins, const dgVector* const averPoins, dgFloat32 timestep)
{
dgAssert (0);
return 0;
/*
	//dgInt32 i;
	//dgInt32 count;
	//dgMinkFace *face; 
	dgSimd invMag; 

	//		m_planePurge = NULL;
	//		m_facePlaneIndex = 0;
	dgMinkFace* const face = &m_simplex[0];

	//		dgVector step (veloc.Scale (timestep + DG_ROBUST_PLANE_CLIP * dgRsqrt(veloc % veloc)));
	DG_RSQRT_SIMD_S(simd_set1(m_localRelVeloc.DotProductSimd(m_localRelVeloc)), invMag)
		invMag = simd_mul_add_s (simd_set1(timestep), simd_set1(DG_ROBUST_PLANE_CLIP), invMag);
	dgVector step (simd_mul_v((dgSimd&) m_localRelVeloc, simd_permut_v (invMag, invMag, PURMUT_MASK(0, 0, 0, 0))));


	for (dgInt32 i = 0; i < 3; i ++) {
		//			m_hullVertex[i] = diffPoins[i] - step;
		(dgSimd&)m_hullVertex[i] = simd_sub_v ((dgSimd&)diffPoins[i], (dgSimd&)step);

		//			m_averVertex[i] = averPoins[i] + step;
		(dgSimd&)m_averVertex[i] = simd_add_v ((dgSimd&)averPoins[i], (dgSimd&)step);
	}

	CalcFacePlaneSimd (face);
	dgPlane &facePlane  = *face;
	if ((facePlane % m_localRelVeloc) > dgFloat32 (0.0f)) {
		facePlane = facePlane.Scale (dgFloat32 (-1.0f));
	}

	dgVector minkFloatingPosit (m_matrix.m_posit );
	(dgSimd&)m_matrix.m_posit = simd_add_v ((dgSimd&)m_matrix.m_posit, (dgSimd&)step);
	dgInt32 count = CalculateContactsSimd(face, contacID, contactOut, maxContacts);
	dgAssert (count <= maxContacts);

	m_matrix.m_posit = minkFloatingPosit;

	return count;
*/
}



dgInt32 dgContactSolver::CalculateConvexShapeIntersectionSimd (
	const dgMatrix& matrix, const dgVector& shapeNormal, dgUnsigned32 id, dgFloat32 penetration,
	dgInt32 shape1VertexCount, dgVector* const shape1, dgInt32 shape2VertexCount, dgVector* const shape2,
	dgContactPoint* const contactOut, dgInt32 maxContacts)
{
	dgInt32 count = 0;
	if (shape2VertexCount <= 2) {
		count = CalculateConvexShapeIntersectionLine (matrix, shapeNormal, id, penetration, shape1VertexCount, shape1, shape2VertexCount, shape2, contactOut);
		if (count > maxContacts) {
			count = maxContacts;
		}

	} else if (shape1VertexCount <= 2) {
		count = CalculateConvexShapeIntersectionLine (matrix, shapeNormal, id, penetration, shape2VertexCount, shape2, shape1VertexCount, shape1, contactOut);
		if (count > maxContacts) {
			count = maxContacts;
		}

	} else {
		dgPerimenterEdge* edgeClipped[2];
		dgPerimenterEdge subdivision[128];

		dgAssert (shape1VertexCount >= 3);
		dgAssert (shape2VertexCount >= 3);
		dgVector pool[64];
		dgVector* output = &pool[0];
		//dgVector* output = (dgVector*) &m_hullVertex[shape1VertexCount + shape2VertexCount];

		dgAssert ((shape1VertexCount + shape2VertexCount) < dgInt32 (sizeof (subdivision) / (2 * sizeof (subdivision[0]))));
		for (dgInt32 i0 = 0; i0 < shape2VertexCount; i0 ++) {
			subdivision[i0].m_vertex = &shape2[i0];
			subdivision[i0].m_prev = &subdivision[i0 - 1];
			subdivision[i0].m_next = &subdivision[i0 + 1];
		}
		subdivision[0].m_prev = &subdivision[shape2VertexCount - 1];
		subdivision[shape2VertexCount - 1].m_next = &subdivision[0];

		dgPerimenterEdge* poly = &subdivision[0];

		dgSimd zero (dgFloat32 (0.0f));
		dgSimd neg_one (dgFloat32 (-1.0f));
		dgSimd tol_pos_1e_24 (dgFloat32 (1.0e-24f));
		dgSimd tol_neg_1e_24 (dgFloat32 (-1.0e-24f));

		edgeClipped[0] = NULL;
		edgeClipped[1] = NULL;

		dgInt32 i0 = shape1VertexCount - 1;
		dgInt32 edgeIndex = shape2VertexCount;
		dgSimd normal ((dgSimd&)shapeNormal & dgSimd (-1, -1, -1, 0));
		for (dgInt32 i1 = 0; i1 < shape1VertexCount; i1 ++) {
			dgSimd edgePlane (normal.CrossProduct((dgSimd&)shape1[i1] - (dgSimd&)shape1[i0]));
			dgSimd edgePlaneOrigin ((dgSimd&)shape1[i0]);

			i0 = i1;
			count = 0;
			dgPerimenterEdge* tmp = poly;
			dgInt32 isInside = 0;
			dgSimd test0 (edgePlane.DotProduct (*(dgSimd*)tmp->m_vertex - edgePlaneOrigin));
			do {
				dgSimd test1 (edgePlane.DotProduct (*(dgSimd*)tmp->m_next->m_vertex - edgePlaneOrigin));
				if ((test0 >= zero).GetSignMask()) {

					isInside |= 1;
					if ((test1 < zero).GetSignMask()) {

						dgSimd p0 (*(dgSimd*)tmp->m_vertex);
						dgSimd p1 (*(dgSimd*)tmp->m_next->m_vertex);
						dgSimd dp = p1 - p0;
						dgSimd den (edgePlane.DotProduct(dp));
						dgSimd test (den >= zero);
						den = (den.GetMax(tol_pos_1e_24) & test) | den.GetMin(tol_neg_1e_24).AndNot(test);
						den = test0 / den; 
						den = neg_one.GetMax(zero.GetMin(den));
						(dgSimd&)output[0] = p0 - dp * den;

						edgeClipped[0] = tmp;
						count ++;
					}

				} else if ((test1 >= zero).GetSignMask()) {
					isInside |= 1;
					dgSimd p0 (*(dgSimd*)tmp->m_vertex);
					dgSimd p1 (*(dgSimd*)tmp->m_next->m_vertex);
					dgSimd dp = p1 - p0;
					dgSimd den (edgePlane.DotProduct(dp));
					dgSimd test (den >= zero);
					den = (den.GetMax(tol_pos_1e_24) & test) | den.GetMin(tol_neg_1e_24).AndNot(test);
					den = test0 / den; 
					den = neg_one.GetMax(zero.GetMin(den));
					(dgSimd&)output[1] = p0 - dp * den;

					edgeClipped[1] = tmp;
					count ++;
				}
				test0 = test1;
				tmp = tmp->m_next;
			} while (tmp != poly && (count < 2));

			if (!isInside) {
				return 0;
			}

			if (count == 2) {
				dgPerimenterEdge* const newEdge = &subdivision[edgeIndex];
				newEdge->m_next = edgeClipped[1];
				newEdge->m_prev = edgeClipped[0];
				edgeClipped[0]->m_next = newEdge;
				edgeClipped[1]->m_prev = newEdge;

				newEdge->m_vertex = &output[0];
				edgeClipped[1]->m_vertex = &output[1];
				poly = newEdge;

				output += 2;
				edgeIndex ++;
				dgAssert (output < &pool[sizeof (pool)/sizeof (pool[0])]);
				dgAssert (edgeIndex < dgInt32 (sizeof (subdivision) / sizeof (subdivision[0])));
			}
		}

		dgAssert (poly);
		poly = ReduceContacts (poly, maxContacts);

		count = 0;
		dgPerimenterEdge* intersection = poly;
		normal = matrix.RotateVectorSimd(normal);
		do {
			(dgSimd&)contactOut[count].m_point = matrix.TransformVectorSimd(*intersection->m_vertex);
			(dgSimd&)contactOut[count].m_normal = normal;
			contactOut[count].m_penetration = penetration;
			contactOut[count].m_userId = id;
			count ++;
			intersection = intersection->m_next;
		} while (intersection != poly);
	}

	return count;
}



dgContactSolver::dgMinkReturnCode dgContactSolver::UpdateSeparatingPlaneSimd(dgMinkFace*& plane, const dgVector& origin)
{
#if 1
	dgSimd diff[4];
	dgSimd aveg[4];

	plane = NULL;
	dgMinkFace* face = &m_simplex[0];
	dgMinkReturnCode code = dgMinkIntersecting;

	dgInt32 cyclingCount = -1;
	dgMinkFace* lastDescendFace = NULL;

	dgInt32 j = 0;
	dgSimd zero (dgFloat32 (0.0f));
	dgSimd negOne(dgFloat32 (-1.0f));
	dgSimd minDist (dgFloat32 (1.0e20f));
	dgSimd distTolerance (DG_DISTANCE_TOLERANCE);
	dgSimd zeroTolerance (DG_DISTANCE_TOLERANCE_ZERO);
	dgSimd separatingTol (DG_UPDATE_SEPARATING_PLANE_DISTANCE_TOLERANCE1);
	dgSimd index_yx (dgFloat32 (0.0f), dgFloat32 (1.0f), dgFloat32 (0.0f), dgFloat32 (1.0f));
	dgSimd index_wz (dgFloat32 (2.0f), dgFloat32 (3.0f), dgFloat32 (2.0f), dgFloat32 (3.0f));
	for (; face && (j < DG_UPDATE_SEPARATING_PLANE_MAX_ITERATION); j ++) {
		face = NULL;
		dgSimd p0 ((dgSimd&)m_hullVertex[0]);
		dgSimd p1 ((dgSimd&)m_hullVertex[1]);
		dgSimd p2 ((dgSimd&)m_hullVertex[2]);
		dgSimd p3 ((dgSimd&)m_hullVertex[3]);

		dgSimd e10 (p1 - p0);
		dgSimd e20 (p2 - p0);
		dgSimd e30 (p3 - p0);
		dgSimd e12 (p1 - p2);
		dgSimd e32 (p3 - p2);

		dgSimd tmp;
		dgSimd e10_x;
		dgSimd e10_y;
		dgSimd e10_z;
		dgSimd e20_x;
		dgSimd e20_y;
		dgSimd e20_z;
		dgSimd::Transpose4x4 (e10_x, e10_y, e10_z, tmp, e10, e30, e20, e12); 
		dgSimd::Transpose4x4 (e20_x, e20_y, e20_z, tmp, e20, e10, e30, e32); 

		dgSimd nx (e10_y * e20_z - e10_z * e20_y);
		dgSimd ny (e10_z * e20_x - e10_x * e20_z);
		dgSimd nz (e10_x * e20_y - e10_y * e20_x);

		dgSimd dist2 (nx * nx + ny * ny + nz * nz);
		dgSimd mask (dist2 >= zeroTolerance);
		dist2 = dist2.GetMax(zeroTolerance);

		dist2  = dist2.InvSqrt();
		nx = nx * dist2;
		ny = ny * dist2;
		nz = nz * dist2;

		dgSimd origin_P0_xxxx;
		dgSimd origin_P0_yyyy;
		dgSimd origin_P0_zzzz;
		dgSimd origin_P0 ((dgSimd&)origin - p0);
		dgSimd origin_P3 ((dgSimd&)origin - p3);
		dgSimd::Transpose4x4 (origin_P0_xxxx, origin_P0_yyyy, origin_P0_zzzz, tmp, origin_P0, origin_P0, origin_P0, origin_P3); 

		//dist2_ = simd_mul_add_v (simd_mul_add_v (simd_mul_v(nx_, origin_P0_xxxx), ny_, origin_P0_yyyy), nz_, origin_P0_zzzz);
		dist2 = origin_P0_xxxx * nx + origin_P0_yyyy * ny + origin_P0_zzzz * nz; 
		//dist2_ = simd_or_v (simd_and_v(dist2_, mask_), simd_andnot_v (m_negativeOne, mask_));
		dist2 = (dist2 & mask) | negOne.AndNot(mask);

		tmp = dist2.MoveHigh(dist2);
		mask = dist2 > tmp;
		dist2 = (dist2 & mask) | tmp.AndNot(mask);
		dgSimd index ((index_yx & mask) | index_wz.AndNot(mask));

		tmp = dist2.PackLow(dist2);
		tmp = tmp.MoveHigh(tmp);
		mask = dist2 > tmp;
		dist2 = (dist2 & mask) | tmp.AndNot(mask);
		tmp = index.PackLow(index);
		index = ((index & mask) | tmp.MoveHigh(tmp).AndNot(mask));

		mask = dist2 > zero;
		index = (index & mask) | negOne.AndNot(mask);
		dgInt32 faceIndex = index.GetInt();
		if (faceIndex >= 0) {
			dgSimd transposedNormals[4];
			face = &m_simplex[faceIndex];

			dgSimd::Transpose4x4 (transposedNormals[0], transposedNormals[1], transposedNormals[2], transposedNormals[3], nx, ny, nz, zero); 
			dgVector normal (transposedNormals[faceIndex]);

			//i0 = face->m_vertex;
			dgInt32 index = face->m_vertex[0];
			CalcSupportVertexSimd (normal, 4);
			dgSimd dist (((dgSimd&)normal).DotProduct((dgSimd&)m_hullVertex[4] - (dgSimd&)m_hullVertex[index]));

			// if we are doing too many passes it means that it is a skew shape 
			// increasing the tolerance help to resolve the problem
			if ((dist < separatingTol).GetSignMask() & 1) {
				plane = face;
				code = dgMinkDisjoint;
				break;
			}

			if ((dist < minDist).GetSignMask() & 1) {
				minDist = dist;
				lastDescendFace = face;
				cyclingCount = -1;
				for (dgInt32 k = 0; k < 4; k ++) {
					diff[k] = (dgSimd&)m_hullVertex[k];
					aveg[k] = (dgSimd&)m_averVertex[k];
				}
			}

			cyclingCount ++;
			if (cyclingCount > 4) {
				for (dgInt32 k = 0; k < 4; k ++) {
					(dgSimd&)m_hullVertex[k] = diff[k];
					(dgSimd&)m_averVertex[k] = aveg[k];
				}
				code = dgMinkDisjoint;
				plane = lastDescendFace;
				break;
			}

			if ((dist < distTolerance).GetSignMask() & 1) {
				dgInt32 i = 0;
				for (; i < 4; i ++ ) {
					dgVector error (m_hullVertex[i] - m_hullVertex[4]);
					if ((error % error) < (DG_DISTANCE_TOLERANCE * DG_DISTANCE_TOLERANCE)) {
						plane = face;
						//code = dgMinkDisjoint;
						code = UpdateSeparatingPlaneFallbackSolution (plane, origin);
						dgAssert ((code == dgMinkDisjoint) || ((code == dgMinkIntersecting) && (m_vertexIndex == 4)));
						break;
					}
				}
				if (i < 4) {
					break;
				}
			}

			dgInt32 i0 = face->m_vertex[0];
			dgInt32 i1 = face->m_vertex[1];
			dgInt32 i2 = m_faceIndex[face - m_simplex][3];

			dgAssert (i2 != face->m_vertex[0]);
			dgAssert (i2 != face->m_vertex[1]);
			dgAssert (i2 != face->m_vertex[2]);
			dgSwap ((dgSimd&)m_hullVertex[i0], (dgSimd&)m_hullVertex[i1]);
			dgSwap ((dgSimd&)m_averVertex[i0], (dgSimd&)m_averVertex[i1]);

			(dgSimd&)m_hullVertex[i2] = (dgSimd&)m_hullVertex[4];
			(dgSimd&)m_averVertex[i2] = (dgSimd&)m_averVertex[4];
			if (!CheckTetrahedronVolumeSimd ()) {
				dgSwap ((dgSimd&)m_hullVertex[1], (dgSimd&)m_hullVertex[2]);
				dgSwap ((dgSimd&)m_averVertex[1], (dgSimd&)m_averVertex[2]);
				dgAssert (CheckTetrahedronVolumeSimd ());
			}
		}
	} 

	if (j >= DG_UPDATE_SEPARATING_PLANE_MAX_ITERATION) {
		dgAssert (CheckTetrahedronVolumeSimd());
		code = UpdateSeparatingPlaneFallbackSolution (plane, origin);
	}
	return code;

#else 
	dgVector diff[4];
	dgVector aveg[4];

	dgAssert (cache);

	plane = NULL;
	dgInt32 faceIndex = 0;
	dgMinkFace* lastDescendFace = NULL;
	dgMinkReturnCode code = dgMinkIntersecting;

	// this loop can calculate the closest point to the origin usually in 4 to 5 passes,
	dgInt32 j = 0;
	dgInt32 cyclingCount = -1;
	dgFloat32 minDist = dgFloat32 (1.0e20f);
	for (; (faceIndex != -1) && (j < DG_UPDATE_SEPARATING_PLANE_MAX_ITERATION); j ++) {
		faceIndex = -1;
		dgVector normal;
		// initialize distance to zero (very important)
		dgFloat32 maxDist = dgFloat32 (0.0f);
		for (dgInt32 i = 0; i < 4; i ++) {
			dgInt32 i0 = m_faceIndex[i][0];
			dgInt32 i1 = m_faceIndex[i][1];
			dgInt32 i2 = m_faceIndex[i][2];

			dgAssert (i0 == m_simplex[i].m_vertex[0]);
			dgAssert (i1 == m_simplex[i].m_vertex[1]);
			dgAssert (i2 == m_simplex[i].m_vertex[2]);

			const dgVector& p0 = m_hullVertex[i0];
			const dgVector& p1 = m_hullVertex[i1];
			const dgVector& p2 = m_hullVertex[i2];
			dgVector e0 (p1 - p0);
			dgVector e1 (p2 - p0);
			dgVector n (e0 * e1);

			dgFloat32 dist = n % n;
			if (dist > DG_DISTANCE_TOLERANCE_ZERO) {
				n = n.Scale (dgRsqrt (dist));
				dist = n % (origin - p0);

				// find the plane farther away from the origin
				if (dist > maxDist) {
					maxDist = dist;
					normal = n;
					//face = &m_simplex[i];
					faceIndex = i;
				}
			}
		}


		// if we do not have a face at this point it means that the mink shape of the tow convexity face have a very 
		// skew ratios on floating point accuracy is not enough to guarantee convexity of the shape
		if (faceIndex != -1) {
			dgMinkFace* const face = &m_simplex[faceIndex];
			dgInt32 index = face->m_vertex[0];

			CalcSupportVertexSimd (normal, 4);
			dgFloat32 dist = normal % (m_hullVertex[4] - m_hullVertex[index]);

			// if we are doing too many passes it means that it is a skew shape with big and small floats  
			// significant bits may be lost in dist calculation, increasing the tolerance help to resolve the problem
			if(dist < DG_UPDATE_SEPARATING_PLANE_DISTANCE_TOLERANCE1) {
				plane = face;
				code = dgMinkDisjoint;
				break;
			}

			if (dist < minDist) {
				minDist = dist;
				lastDescendFace = face;
				cyclingCount = -1;
				for (dgInt32 k = 0; k < 4; k ++) {
					diff[k] = m_hullVertex[k];
					aveg[k] = m_averVertex[k];
				}
			}

			cyclingCount ++;
			if (cyclingCount > 4) {
				for (dgInt32 k = 0; k < 4; k ++) {
					m_hullVertex[k] = diff[k];
					m_averVertex[k] = aveg[k];
				}
				code = dgMinkDisjoint;
				plane = lastDescendFace;
				break;
			}


			if (dist < DG_DISTANCE_TOLERANCE) {
				dgInt32 i = 0;
				for (; i < 4; i ++ ) {
					dgVector error (m_hullVertex[i] - m_hullVertex[4]);
					if ((error % error) < (DG_DISTANCE_TOLERANCE * DG_DISTANCE_TOLERANCE)) {
						plane = face;
						//code = dgMinkDisjoint;
						code = UpdateSeparatingPlaneFallbackSolution (plane, origin);
						dgAssert ((code == dgMinkDisjoint) || ((code == dgMinkIntersecting) && (m_vertexIndex == 4)));
						break;
					}
				}
				if (i < 4) {
					break;
				}
			}

			//faceIndex = -1;
			dgInt32 i0 = face->m_vertex[0];
			dgInt32 i1 = face->m_vertex[1];
			dgInt32 i2 = m_faceIndex[face - m_simplex][3];

			dgAssert (i2 != face->m_vertex[0]);
			dgAssert (i2 != face->m_vertex[1]);
			dgAssert (i2 != face->m_vertex[2]);
			dgSwap (m_hullVertex[i0], m_hullVertex[i1]);
			dgSwap (m_averVertex[i0], m_averVertex[i1]);
			dgSwap (cache->m_dirs[i0], cache->m_dirs[i1]);
			m_hullVertex[i2] = m_hullVertex[4];
			m_averVertex[i2] = m_averVertex[4];
			cache->m_dirs[i2] = normal;

			if (!CheckTetrahedronVolumeSimd ()) {
				dgSwap (m_hullVertex[1], m_hullVertex[2]);
				dgSwap (m_averVertex[1], m_averVertex[2]);
				dgSwap (cache->m_dirs[1], cache->m_dirs[2]);
				dgAssert (CheckTetrahedronVolume ());
			}
		}
	} 

	if (j >= DG_UPDATE_SEPARATING_PLANE_MAX_ITERATION) {
		dgAssert (CheckTetrahedronVolume());
		code = UpdateSeparatingPlaneFallbackSolution (plane, origin);
	}
	return code;
#endif
}


dgInt32 dgContactSolver::HullHullContactsSimd (dgInt32 contactID)
{
	dgInt32 count = 0;
	dgMinkFace* face;
	dgMinkReturnCode code = CalcSeparatingPlaneSimd (face);
	switch (code)
	{
		case dgMinkIntersecting:
		{
			face = CalculateClipPlaneSimd ();
			if (face) {
				count = CalculateContactsSimd (face, contactID, m_proxy->m_contacts, m_proxy->m_maxContacts);
				dgAssert (count <= m_proxy->m_maxContacts);
			}
			break;
		}

		case dgMinkDisjoint:
		{
			dgAssert (face);
			if (CalcFacePlaneSimd (face)) {
				//dgAssert (face->m_w >= dgFloat32 (0.0f));
				dgAssert ((*face) % (*face) > dgFloat32 (0.0f));
				if (face->m_w < m_penetrationPadding) {

					dgSimd step (*(dgSimd*)face * dgSimd (-(face->m_w + DG_IMPULSIVE_CONTACT_PENETRATION)));
					step = step & dgSimd(-1, -1, -1, 0);
					
					dgInt32 i0 = face->m_vertex[0];
					(dgSimd&)m_hullVertex[i0] = (dgSimd&)m_hullVertex[i0] - step;
					(dgSimd&)m_averVertex[i0] = (dgSimd&)m_averVertex[i0] + step;

					(dgSimd&)m_matrix.m_posit = (dgSimd&)m_matrix.m_posit + step;
					dgSimd stepWorld (m_proxy->m_referenceCollision->m_globalMatrix.RotateVectorSimd(step));

					dgCollisionInstance* const saveShape = m_proxy->m_floatingCollision; 
					dgCollisionInstance tmpInstance (*m_proxy->m_floatingCollision);
					m_proxy->m_floatingCollision = &tmpInstance;
					(dgSimd&)tmpInstance.m_globalMatrix.m_posit = (dgSimd&)tmpInstance.m_globalMatrix.m_posit + stepWorld;

					m_proxy->m_floatingCollision = saveShape;

					count = CalculateContactsSimd (face, contactID, m_proxy->m_contacts, m_proxy->m_maxContacts);
					dgAssert (count < m_proxy->m_maxContacts);
					stepWorld = stepWorld * dgSimd (dgFloat32 (0.5f));

					dgContactPoint* const contactOut = m_proxy->m_contacts; 
					for (dgInt32 i0 = 0; i0 < count; i0 ++ ) {
						(dgSimd&)contactOut[i0].m_point = (dgSimd&)contactOut[i0].m_point - stepWorld;
					}

					return count;
				}
			}
		}
		case dgMinkError:
		default:;
	}
	return count;
}


dgContactSolver::dgMinkReturnCode dgContactSolver::CalcSeparatingPlaneSimd(dgMinkFace*& plane, const dgVector& origin)
{
	dgVector e1;
	dgVector e2;
	dgVector e3;

	dgFloat32 error2 = dgFloat32 (0.0f);
	dgVector normal (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));

	CalcSupportVertexSimd (m_dir[0], 0);
	dgInt32 i = 1;
	for (; i < dgInt32(sizeof(m_dir) / sizeof(m_dir[0])); i ++) {
		CalcSupportVertexSimd (m_dir[i], 1);
		e1 = m_hullVertex[1] - m_hullVertex[0];
		error2 = e1 % e1;
		if (error2 > DG_CALCULATE_SEPARATING_PLANE_ERROR) {
			break;
		}
	}

	for (i ++; i < dgInt32(sizeof(m_dir) / sizeof(m_dir[0])); i ++) {
		CalcSupportVertexSimd (m_dir[i], 2);
		e2 = m_hullVertex[2] - m_hullVertex[0];
		normal = e1 * e2;
		error2 = normal % normal;
		if (error2 > DG_CALCULATE_SEPARATING_PLANE_ERROR1) {
			break;
		}
	}

	error2 = dgFloat32 (0.0f);
	for (i ++; i < dgInt32(sizeof(m_dir) / sizeof(m_dir[0])); i ++) {
		CalcSupportVertexSimd (m_dir[i], 3);
		e3 = m_hullVertex[3] - m_hullVertex[0];
		error2 = normal % e3;
		if (dgAbsf (error2) > DG_CALCULATE_SEPARATING_PLANE_ERROR1) {
			break;
		}
	}

	if (i >= dgInt32(sizeof(m_dir) / sizeof(m_dir[0]))) {
		dgAssert (0);
		dgInt32 best = 0;
		dgFloat32 maxErr = dgFloat32 (0.0f);
		for (dgInt32 i = 1; i < dgInt32(sizeof(m_dir) / sizeof(m_dir[0])); i ++) {
			CalcSupportVertexSimd (m_dir[i], 1);
			e1 = m_hullVertex[1] - m_hullVertex[0];
			error2 = e1 % e1;
			if (error2 > maxErr) {
				best = i;
				maxErr = error2;
			}
		}
		CalcSupportVertexSimd (m_dir[best], 1);
		e1 = m_hullVertex[1] - m_hullVertex[0];

		best = 0;
		maxErr = dgFloat32 (0.0f);
		for (dgInt32 i = 1; i < dgInt32(sizeof(m_dir) / sizeof(m_dir[0])); i ++) {
			CalcSupportVertexSimd (m_dir[i], 2);
			e2 = m_hullVertex[2] - m_hullVertex[0];
			normal = e1 * e2;
			error2 = normal % normal;
			if (error2 > maxErr) {
				best = i;
				maxErr = error2;
			}
		}

		CalcSupportVertexSimd (m_dir[best], 2);
		e2 = m_hullVertex[2] - m_hullVertex[0];
		normal = e1 * e2;

		best = 0;
		maxErr = dgFloat32 (0.0f);
		for (dgInt32 i = 1; i < dgInt32(sizeof(m_dir) / sizeof(m_dir[0])); i ++) {
			CalcSupportVertexSimd (m_dir[i], 3);

			e3 = m_hullVertex[3] - m_hullVertex[0];
			error2 = normal % e3;
			if (dgAbsf (error2) > dgAbsf (maxErr)) {
				best = i;
				maxErr = error2;
			}
		}
		error2 = maxErr;
		CalcSupportVertexSimd (m_dir[best], 3);
	}

	m_vertexIndex = 4;
	if (error2 > dgFloat32 (0.0f)) {
		dgSwap (m_hullVertex[1], m_hullVertex[2]);
		dgSwap (m_averVertex[1], m_averVertex[2]);
	}

	dgAssert (CheckTetrahedronVolume ());
	//dgAssert (CheckTetrahedronVolumeSimd ());

	dgAssert ( (((dgUnsigned64)&m_simplex[0]) & 0x0f)== 0);
	dgAssert ( (((dgUnsigned64)&m_simplex[1]) & 0x0f)== 0);

	// face 0
	m_simplex[0].m_vertex[0] = 0;
	m_simplex[0].m_vertex[1] = 1;
	m_simplex[0].m_vertex[2] = 2;
	m_simplex[0].m_vertex[3] = 0;
	m_simplex[0].m_adjancentFace[0] = 1;	
	m_simplex[0].m_adjancentFace[1] = 3;	
	m_simplex[0].m_adjancentFace[2] = 2;	

	// face 1
	m_simplex[1].m_vertex[0] = 1;
	m_simplex[1].m_vertex[1] = 0;
	m_simplex[1].m_vertex[2] = 3;
	m_simplex[1].m_vertex[3] = 1;
	m_simplex[1].m_adjancentFace[0] = 0;	
	m_simplex[1].m_adjancentFace[1] = 2;	
	m_simplex[1].m_adjancentFace[2] = 3;	

	// face 2
	m_simplex[2].m_vertex[0] = 0;
	m_simplex[2].m_vertex[1] = 2;
	m_simplex[2].m_vertex[2] = 3;
	m_simplex[2].m_vertex[3] = 0;
	m_simplex[2].m_adjancentFace[0] = 0;	
	m_simplex[2].m_adjancentFace[1] = 3;	
	m_simplex[2].m_adjancentFace[2] = 1;	


	// face 3
	m_simplex[3].m_vertex[0] = 2;
	m_simplex[3].m_vertex[1] = 1;
	m_simplex[3].m_vertex[2] = 3;
	m_simplex[3].m_vertex[3] = 2;
	m_simplex[3].m_adjancentFace[0] = 0;	
	m_simplex[3].m_adjancentFace[1] = 1;	
	m_simplex[3].m_adjancentFace[2] = 2;	

	return UpdateSeparatingPlaneSimd(plane, origin);
}
#endif