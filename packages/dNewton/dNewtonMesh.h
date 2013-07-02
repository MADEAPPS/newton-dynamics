/* Copyright (c) <2003-2013> <Julio Jerez, Newton Game Dynamics>
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

#ifndef _D_NEWTON_MESH_H_
#define _D_NEWTON_MESH_H_

#include "dStdAfxNewton.h"
#include "dNewtonAlloc.h"

class dNewton;
class dNewtonCollision;

class dNewtonMesh: public dNewtonAlloc
{
	public:
	CNEWTON_API dNewtonMesh(dNewton* const world);
	CNEWTON_API dNewtonMesh(const dNewtonMesh& clone);
	CNEWTON_API dNewtonMesh(const dNewtonCollision& collision);
	CNEWTON_API dNewtonMesh(dNewton* const world, int pointCount, const dFloat* const vertexCloud, int strideInBytes, dFloat tolerance);

	CNEWTON_API void CreateVoronoiConvexDecomposition (const dNewtonMesh& convexMesh);
	CNEWTON_API void CreateApproximateConvexDecomposition (const dNewtonMesh& mesh, dFloat maxConcavity, dFloat backFaceDistanceFactor, int maxCount, int maxVertexPerHull);
	
	CNEWTON_API virtual ~dNewtonMesh();

	protected:
	NewtonMesh* m_mesh;
};

#endif
