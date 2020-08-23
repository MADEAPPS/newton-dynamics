/* 
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


#include "stdafx.h"
#include "dNewtonBody.h"
#include "dNewtonWorld.h"
#include "dNewtonCollision.h"

class DebugCallBack
{
	public:
	dVector m_eyePoint;
	OnDrawFaceCallback m_callback;
};

dMatrix dNewtonCollision::m_primitiveAligment(dVector(0.0f, 1.0f, 0.0f, 0.0f), dVector(-1.0f, 0.0f, 0.0f, 0.0f), dVector(0.0f, 0.0f, 1.0f, 0.0f), dVector(0.0f, 0.0f, 0.0f, 1.0f));


dNewtonCollision::dNewtonCollision(dNewtonWorld* const world, dLong collisionMask)
	:dAlloc()
	,m_shape(NULL)
	,m_myWorld(world)
	,m_materialID(0)
{
}

dNewtonCollision::~dNewtonCollision()
{
	DeleteShape();
}

bool dNewtonCollision::IsValid()
{
	return m_shape ? true : false;
}

void dNewtonCollision::SetAsTrigger(bool mode)
{
	if (m_shape) {
		NewtonCollisionSetMode(m_shape, mode ? 0 : 1);
	}
}

void dNewtonCollision::SetMaterialID(int materialId)
{
	m_materialID = materialId;
}

void dNewtonCollision::SetLayer(int layer)
{
	m_layer = layer;
}

void dNewtonCollision::DeleteShape()
{
	if (m_shape) {
		NewtonWaitForUpdateToFinish(m_myWorld->m_world);
		NewtonCollisionSetUserData(m_shape, NULL);
		NewtonDestroyCollision(m_shape);

		m_myWorld->m_collisionCache.Remove(m_collisionCacheNode);
		m_shape = NULL;
		m_collisionCacheNode = NULL;
	}
}

void dNewtonCollision::SetShape(NewtonCollision* const shape)
{
	m_shape = shape;
	NewtonCollisionSetUserData(m_shape, this);
	m_collisionCacheNode = m_myWorld->m_collisionCache.Append(this);
}

void dNewtonCollision::DebugRenderCallback(void* userData, int vertexCount, const dFloat* faceVertec, int id)
{
	DebugCallBack* const callbackInfo = (DebugCallBack*)userData;

	dVector normal = dVector(0.0f);
	dVector p0 (faceVertec[0], faceVertec[1], faceVertec[2], 0.0f);
	dVector p1 (faceVertec[3], faceVertec[4], faceVertec[5], 0.0f);
	dVector p1p0(p1 - p0);
	for (int i = 2; i < vertexCount; i++) {
		dVector p2(faceVertec[i * 3 + 0], faceVertec[i * 3 + 1], faceVertec[i * 3 + 2], 0.0f);
		dVector p2p0(p2 - p0);
		normal += p1p0.CrossProduct(p2p0);
		p1p0 = p2p0;
	}

	dFloat side = normal.DotProduct3 (callbackInfo->m_eyePoint - p0);
	if (side > 0.0f) {
		callbackInfo->m_callback(faceVertec, vertexCount);
	}
}

void dNewtonCollision::DebugRender(OnDrawFaceCallback callback, const dVector eyePoint)
{
	DebugCallBack callbackInfo;
	callbackInfo.m_eyePoint = eyePoint;

	callbackInfo.m_callback = callback;
	dMatrix matrix(dGetIdentityMatrix());
	NewtonCollisionForEachPolygonDo(m_shape, &matrix[0][0], DebugRenderCallback, &callbackInfo);
}

void dNewtonCollision::SetScale(dFloat scaleX, dFloat scaleY, dFloat scaleZ)
{
	scaleX = dMax(0.01f, dAbs(scaleX));
	scaleY = dMax(0.01f, dAbs(scaleY));
	scaleZ = dMax(0.01f, dAbs(scaleZ));
	NewtonCollisionSetScale(m_shape, scaleX, scaleY, scaleZ);
}

void dNewtonCollision::SetMatrix(const dMatrix matrix)
{
	NewtonCollisionSetMatrix(m_shape, &matrix[0][0]);
}


dNewtonCollisionNull::dNewtonCollisionNull(dNewtonWorld* const world)
	:dNewtonCollision(world, 0)
{
	NewtonWaitForUpdateToFinish(m_myWorld->m_world);
	SetShape(NewtonCreateNull(m_myWorld->m_world));
}

dNewtonCollisionSphere::dNewtonCollisionSphere(dNewtonWorld* const world, dFloat r)
	:dNewtonCollision(world, 0)
{
	NewtonWaitForUpdateToFinish(m_myWorld->m_world);
	SetShape(NewtonCreateSphere(m_myWorld->m_world, r, 0, NULL));
}


dNewtonCollisionBox::dNewtonCollisionBox(dNewtonWorld* const world, dFloat x, dFloat y, dFloat z)
	:dNewtonCollision(world, 0)
{
	NewtonWaitForUpdateToFinish(m_myWorld->m_world);
	SetShape(NewtonCreateBox(m_myWorld->m_world, x, y, z, 0, NULL));
}


dNewtonAlignedShapes::dNewtonAlignedShapes(dNewtonWorld* const world, dLong collisionMask)
	:dNewtonCollision(world, 0)
{
}

void dNewtonAlignedShapes::SetScale(dFloat x, dFloat y, dFloat z)
{
	dVector scale(m_primitiveAligment.RotateVector(dVector(x, y, z, 0.0f)));
	dNewtonCollision::SetScale(scale.m_x, scale.m_y, scale.m_z);
}

void dNewtonAlignedShapes::SetMatrix(const dMatrix matrix)
{
	dNewtonCollision::SetMatrix(m_primitiveAligment * matrix);
}

dNewtonCollisionCapsule::dNewtonCollisionCapsule(dNewtonWorld* const world, dFloat radio0, dFloat radio1, dFloat height)
	:dNewtonAlignedShapes(world, 0)
{
	NewtonWaitForUpdateToFinish(m_myWorld->m_world);
	SetShape(NewtonCreateCapsule(m_myWorld->m_world, radio0, radio1, height, 0, NULL));
}


dNewtonCollisionCylinder::dNewtonCollisionCylinder(dNewtonWorld* const world, dFloat radio0, dFloat radio1, dFloat height)
	:dNewtonAlignedShapes(world, 0)
{
	NewtonWaitForUpdateToFinish(m_myWorld->m_world);
	SetShape(NewtonCreateCylinder(m_myWorld->m_world, radio0, radio1, height, 0, NULL));
}

dNewtonCollisionCone::dNewtonCollisionCone(dNewtonWorld* const world, dFloat radio, dFloat height)
	:dNewtonAlignedShapes(world, 0)
{
	NewtonWaitForUpdateToFinish(m_myWorld->m_world);
	SetShape(NewtonCreateCone(m_myWorld->m_world, radio, height, 0, NULL));
}


dNewtonCollisionChamferedCylinder::dNewtonCollisionChamferedCylinder(dNewtonWorld* const world, dFloat radio, dFloat height)
	:dNewtonAlignedShapes(world, 0)
{
	NewtonWaitForUpdateToFinish(m_myWorld->m_world);
	SetShape(NewtonCreateChamferCylinder(m_myWorld->m_world, radio, height, 0, NULL));
}

dNewtonCollisionConvexHull::dNewtonCollisionConvexHull(dNewtonWorld* const world, int vertexCount, const dFloat* const vertexCloud, dFloat tolerance)
	:dNewtonCollision(world, 0)
{
	NewtonWaitForUpdateToFinish(m_myWorld->m_world);
	SetShape(NewtonCreateConvexHull(m_myWorld->m_world, vertexCount, vertexCloud, 3 * sizeof (dFloat), tolerance, 0, NULL));
}


dNewtonCollisionMesh::dNewtonCollisionMesh(dNewtonWorld* const world)
	:dNewtonCollision(world, 0)
{
	NewtonWaitForUpdateToFinish(m_myWorld->m_world);
	SetShape(NewtonCreateTreeCollision(m_myWorld->m_world, 0));
}


void dNewtonCollisionMesh::BeginFace()
{
	NewtonTreeCollisionBeginBuild(m_shape);
}

void dNewtonCollisionMesh::AddFace(int vertexCount, const dFloat* const vertexPtr, int strideInBytes, int faceAttribute)
{
	NewtonTreeCollisionAddFace(m_shape, vertexCount, vertexPtr, strideInBytes, faceAttribute);
}

void dNewtonCollisionMesh::EndFace(bool optimize)
{
	NewtonTreeCollisionEndBuild(m_shape, optimize ? 1 : 0);
}

dNewtonCollisionCompound::dNewtonCollisionCompound(dNewtonWorld* const world)
	:dNewtonCollision(world, 0)
{
	NewtonWaitForUpdateToFinish(m_myWorld->m_world);
	SetShape(NewtonCreateCompoundCollision(m_myWorld->m_world, 0));
}

void dNewtonCollisionCompound::BeginAddRemoveCollision()
{
	NewtonCompoundCollisionBeginAddRemove(m_shape);
}

void* dNewtonCollisionCompound::AddCollision(dNewtonCollision* const collision)
{
	return NewtonCompoundCollisionAddSubCollision(m_shape, collision->m_shape);
}

void dNewtonCollisionCompound::RemoveCollision(void* const handle)
{
	NewtonCompoundCollisionRemoveSubCollision(m_shape, handle);
}

void dNewtonCollisionCompound::EndAddRemoveCollision()
{
	NewtonCompoundCollisionEndAddRemove(m_shape);
}

dNewtonCollisionHeightField::dNewtonCollisionHeightField(dNewtonWorld* const world, const dFloat* const elevations, int resolution, dVector scale)
	:dNewtonCollision(world, 0)
{
	char* const attibute = new char[resolution * resolution];
	memset(attibute, 0, sizeof(char) * resolution * resolution);

	dFloat scaleFactor = 1.0f / (resolution - 1);
	NewtonCollision* const shape = NewtonCreateHeightFieldCollision(
		m_myWorld->m_world, resolution, resolution, 1, 0, elevations, attibute, 1.0f, scale.m_x * scaleFactor, scale.m_z * scaleFactor, 0);

	delete[] attibute;
	SetShape(shape);
}


dNewtonCollisionScene::dNewtonCollisionScene(dNewtonWorld* const world)
	:dNewtonCollision(world, 0)
{
	NewtonWaitForUpdateToFinish(m_myWorld->m_world);
	SetShape(NewtonCreateSceneCollision(m_myWorld->m_world, 0));
}

void dNewtonCollisionScene::BeginAddRemoveCollision()
{
	NewtonSceneCollisionBeginAddRemove(m_shape);
}

void* dNewtonCollisionScene::AddCollision(const dNewtonCollision* const collision)
{
	return NewtonSceneCollisionAddSubCollision(m_shape, collision->m_shape);
}

void dNewtonCollisionScene::RemoveCollision(void* const handle)
{
	NewtonSceneCollisionRemoveSubCollision(m_shape, handle);
}

void dNewtonCollisionScene::EndAddRemoveCollision()
{
	NewtonSceneCollisionEndAddRemove(m_shape);
}
