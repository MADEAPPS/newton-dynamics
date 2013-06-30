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
#include "dNewtonCollision.h"


dNewtonCollision::dNewtonCollision(dCollsionType type)
	:m_shape (NULL)
	,m_type (type)
{
}



dNewtonCollision::dNewtonCollision(NewtonCollision* const shape, dCollsionType type)
	:m_shape (shape)
	,m_type (type)
{
	SetShape (shape);
}


dNewtonCollision::~dNewtonCollision()
{
	if (m_shape) {
		NewtonCollisionSetUserData (m_shape, NULL);
		NewtonDestroyCollision(m_shape);
	}
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

void* dNewtonCollision::operator new (size_t size)
{
	return NewtonAlloc(int (size));
}

void dNewtonCollision::operator delete (void* ptr)
{
	NewtonFree(ptr);
}



dNewtonCollisionMesh::dNewtonCollisionMesh(dNewton* const world)
	:dNewtonCollision(m_mesh)
{
	SetShape (NewtonCreateTreeCollision(world->GetNewton(), 0));
}

dNewtonCollision* dNewtonCollisionMesh::Clone(NewtonCollision* const shape) const 
{
	dAssert(0);
	return 0;
//	dAssert (m_type == m_mesh);
//	return new dNewtonCollisionScene (shape);
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

dNewtonCollision* dNewtonCollisionScene::Clone(NewtonCollision* const shape) const 
{
	dAssert (m_type == m_scene);
	return new dNewtonCollisionScene (shape, m_type);
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

