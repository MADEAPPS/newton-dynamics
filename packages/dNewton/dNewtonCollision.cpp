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

#include "dStdAfxNewton.h"
#include "dNewton.h"
#include "dNewtonMesh.h"
#include "dNewtonCollision.h"


dNewtonCollision::dNewtonCollision(dCollsionType type)
	:m_shape (NULL)
	,m_userData(NULL)
	,m_type (type)
{
}


dNewtonCollision::dNewtonCollision(const dNewtonCollision& srcCollision, NewtonCollision* const shape)
	:m_shape (shape)
	,m_userData(srcCollision.m_userData)
	,m_type (srcCollision.m_type)
{
	SetShape(shape);
}


dNewtonCollision::~dNewtonCollision()
{
	if (m_shape) {
		NewtonCollisionSetUserData (m_shape, NULL);
		NewtonDestroyCollision(m_shape);
	}
}

void* dNewtonCollision::GetUserData() const
{
	return m_userData;
}

void dNewtonCollision::SetUserData(void* const userData)
{
	m_userData = userData;
}

void dNewtonCollision::SetMatrix (const dFloat* const matrix)
{
	NewtonCollisionSetMatrix(m_shape, matrix);
}

void dNewtonCollision::GetMatrix (dFloat* const matrix) const
{
	NewtonCollisionGetMatrix(m_shape, matrix);
}


void dNewtonCollision::SetScale(dFloat x, dFloat y, dFloat z)
{
	NewtonCollisionSetScale(m_shape, x, y, z);
}
void dNewtonCollision::GetScale(dFloat& x, dFloat& y, dFloat& z) const
{
	NewtonCollisionGetScale(m_shape, &x, &y, &z);
}


NewtonCollision* dNewtonCollision::GetShape() const
{
	return m_shape;
}

void dNewtonCollision::SetShape (NewtonCollision* const shape)
{
	m_shape = shape;
	NewtonCollisionSetUserData (m_shape, this);
}

void dNewtonCollision::DebugRender (void* userData, int vertexCount, const dFloat* faceVertec, int id)
{
	dDebugRenderer* const renderer = (dDebugRenderer*) userData;
	renderer->OnDrawFace (vertexCount, faceVertec, id);
}


void dNewtonCollision::DebugRender (const dFloat* const matrix, dDebugRenderer* const renderer) const
{
	NewtonCollisionForEachPolygonDo (m_shape, matrix, DebugRender, renderer);
}


void dNewtonCollision::CalculateAABB (const dFloat* const matrix, dFloat* const p0, dFloat* const p1) const
{
	NewtonCollisionCalculateAABB (m_shape, matrix, p0, p1);
}



dNewtonCollisionMesh::dNewtonCollisionMesh(dNewton* const world)
	:dNewtonCollision(m_mesh)
{
	SetShape (NewtonCreateTreeCollision(world->GetNewton(), 0));
}

dNewtonCollisionMesh::dNewtonCollisionMesh (dNewton* const world, const dNewtonMesh& mesh, int id)
	:dNewtonCollision(m_mesh)
{
	SetShape (NewtonCreateTreeCollisionFromMesh (world->GetNewton(), mesh.GetMesh(), id));
}



void dNewtonCollisionMesh::BeginFace()
{
	NewtonTreeCollisionBeginBuild(m_shape);
}

void dNewtonCollisionMesh::AddFace(int vertexCount, const dFloat* const vertexPtr, int strideInBytes, int faceAttribute)
{
	NewtonTreeCollisionAddFace (m_shape, vertexCount, vertexPtr, strideInBytes, faceAttribute);
}

void dNewtonCollisionMesh::EndFace()
{
	//NewtonTreeCollisionEndBuild (m_shape, 1);
	NewtonTreeCollisionEndBuild (m_shape, 0);
}


dNewtonCollisionScene::dNewtonCollisionScene(dNewton* const world)
	:dNewtonCollision(m_scene)
{
	SetShape (NewtonCreateSceneCollision(world->GetNewton(), 0));
}


void dNewtonCollisionScene::BeginAddRemoveCollision()
{
	NewtonSceneCollisionBeginAddRemove (m_shape);	
}

void* dNewtonCollisionScene::AddCollision(const dNewtonCollision* const collision)
{
	return NewtonSceneCollisionAddSubCollision (m_shape, collision->GetShape());
}

void dNewtonCollisionScene::RemoveCollision (void* const handle)
{
	NewtonSceneCollisionRemoveSubCollision (m_shape, handle);
}

void dNewtonCollisionScene::EndAddRemoveCollision()
{
	NewtonSceneCollisionEndAddRemove(m_shape);	
}


dNewtonCollisionCompound::dNewtonCollisionCompound (dNewton* const world, const dNewtonMesh& mesh, int shapeID)
	:dNewtonCollision(m_compound)
{
	SetShape (NewtonCreateCompoundCollisionFromMesh (world->GetNewton(), mesh.GetMesh(), 0.001f, shapeID, shapeID));
}


void dNewtonCollisionCompound::BeginAddRemoveCollision()
{
	NewtonCompoundCollisionBeginAddRemove (m_shape);	
}

void* dNewtonCollisionCompound::AddCollision(const dNewtonCollision* const collision)
{
	return NewtonCompoundCollisionAddSubCollision(m_shape, collision->GetShape());
}

void dNewtonCollisionCompound::RemoveCollision (void* const handle)
{
	NewtonCompoundCollisionRemoveSubCollision (m_shape, handle);
}

void dNewtonCollisionCompound::EndAddRemoveCollision()
{
	NewtonCompoundCollisionEndAddRemove(m_shape);	
}

