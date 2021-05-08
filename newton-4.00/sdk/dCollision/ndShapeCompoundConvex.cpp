/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#include "dCoreStdafx.h"
#include "ndCollisionStdafx.h"
#include "ndShapeInstance.h"
#include "ndContactSolver.h"
#include "ndCollisionStdafx.h"
#include "ndShapeCompoundConvex.h"


ndShapeCompoundConvex::ndTreeArray::ndTreeArray()
	:dTree<ndNodeBase*, dInt32, dContainersFreeListAlloc<ndNodeBase*>>()
{
	dAssert(0);
}

//void ndShapeCompoundConvex::ndTreeArray::AddNode(ndNodeBase* const node, dInt32 index, const ndShapeInstance* const parent)
void ndShapeCompoundConvex::ndTreeArray::AddNode(ndNodeBase* const, dInt32, const ndShapeInstance* const)
{
	dAssert(0);
}


ndShapeCompoundConvex::ndShapeCompoundConvex()
	:ndShape(m_compoundConvex)
{
}

ndShapeCompoundConvex::ndShapeCompoundConvex(const nd::TiXmlNode* const xmlNode)
	:ndShape(m_compoundConvex)
{
	dAssert(0);
	xmlGetInt(xmlNode, "xxxx");
}

ndShapeCompoundConvex::~ndShapeCompoundConvex()
{
	dAssert(0);
}

/*
void ndShapeCompoundConvex::CalcAABB(const dMatrix& matrix, dVector &p0, dVector &p1) const
{
	dAssert(0);
	dVector origin(matrix.TransformVector(m_boxOrigin));
	dVector size(matrix.m_front.Abs().Scale(m_boxSize.m_x) + matrix.m_up.Abs().Scale(m_boxSize.m_y) + matrix.m_right.Abs().Scale(m_boxSize.m_z));

	p0 = (origin - size) & dVector::m_triplexMask;
	p1 = (origin + size) & dVector::m_triplexMask;
}


//dInt32 ndShapeCompoundConvex::CalculatePlaneIntersection(const dFloat32* const vertex, const dInt32* const index, dInt32 indexCount, dInt32 stride, const dPlane& localPlane, dVector* const contactsOut) const
dInt32 ndShapeCompoundConvex::CalculatePlaneIntersection(const dFloat32* const, const dInt32* const, dInt32, dInt32, const dPlane&, dVector* const) const
{
	dAssert(0);
	return 0;
	//dInt32 count = 0;
	//dInt32 j = index[indexCount - 1] * stride;
	//dVector p0(&vertex[j]);
	//p0 = p0 & dVector::m_triplexMask;
	//dFloat32 side0 = localPlane.Evalue(p0);
	//for (dInt32 i = 0; i < indexCount; i++) {
	//	j = index[i] * stride;
	//	dVector p1(&vertex[j]);
	//	p1 = p1 & dVector::m_triplexMask;
	//	dFloat32 side1 = localPlane.Evalue(p1);
	//
	//	if (side0 < dFloat32(0.0f)) {
	//		if (side1 >= dFloat32(0.0f)) {
	//			dVector dp(p1 - p0);
	//			dAssert(dp.m_w == dFloat32(0.0f));
	//			dFloat32 t = localPlane.DotProduct(dp).GetScalar();
	//			dAssert(dgAbs(t) >= dFloat32(0.0f));
	//			if (dgAbs(t) < dFloat32(1.0e-8f)) {
	//				t = dgSign(t) * dFloat32(1.0e-8f);
	//			}
	//			dAssert(0);
	//			contactsOut[count] = p0 - dp.Scale(side0 / t);
	//			count++;
	//
	//		}
	//	}
	//	else if (side1 <= dFloat32(0.0f)) {
	//		dVector dp(p1 - p0);
	//		dAssert(dp.m_w == dFloat32(0.0f));
	//		dFloat32 t = localPlane.DotProduct(dp).GetScalar();
	//		dAssert(dgAbs(t) >= dFloat32(0.0f));
	//		if (dgAbs(t) < dFloat32(1.0e-8f)) {
	//			t = dgSign(t) * dFloat32(1.0e-8f);
	//		}
	//		dAssert(0);
	//		contactsOut[count] = p0 - dp.Scale(side0 / t);
	//		count++;
	//	}
	//
	//	side0 = side1;
	//	p0 = p1;
	//}
	//
	//return count;
}
*/

ndShapeInfo ndShapeCompoundConvex::GetShapeInfo() const
{
	ndShapeInfo info(ndShape::GetShapeInfo());

	dAssert(0);
	return info;
}

//void ndShapeCompoundConvex::DebugShape(const dMatrix& matrix, ndShapeDebugCallback& debugCallback) const
void ndShapeCompoundConvex::DebugShape(const dMatrix&, ndShapeDebugCallback&) const
{
	dAssert(0);
}

dFloat32 ndShapeCompoundConvex::GetVolume() const
{
	dAssert(0);
	return dFloat32(0.0f);
}

dFloat32 ndShapeCompoundConvex::GetBoxMinRadius() const
{
	dAssert(0);
	return dFloat32(0.0f);
}

dFloat32 ndShapeCompoundConvex::GetBoxMaxRadius() const
{
	dAssert(0);
	return dFloat32(0.0f);
}

//void ndShapeCompoundConvex::CalcAABB(const dMatrix& matrix, dVector& p0, dVector& p1) const
void ndShapeCompoundConvex::CalcAABB(const dMatrix&, dVector&, dVector&) const
{
	dAssert(0);
}

dVector ndShapeCompoundConvex::SupportVertex(const dVector&, dInt32* const) const
{
	dAssert(0);
	return dVector::m_zero;
}

//dVector ndShapeCompoundConvex::SupportVertexSpecialProjectPoint(const dVector& point, const dVector&) const
dVector ndShapeCompoundConvex::SupportVertexSpecialProjectPoint(const dVector&, const dVector&) const
{ 
	dAssert(0);
	return dVector::m_zero;
}

dVector ndShapeCompoundConvex::SupportVertexSpecial(const dVector& dir, dFloat32, dInt32* const vertexIndex) const
{
	dAssert(0);
	return SupportVertex(dir, vertexIndex);
}

dInt32 ndShapeCompoundConvex::CalculatePlaneIntersection(const dVector&, const dVector&, dVector* const) const
{
	dAssert(0);
	return 0;
}

dVector ndShapeCompoundConvex::CalculateVolumeIntegral(const dMatrix&, const dVector&, const ndShapeInstance&) const
{
	dAssert(0);
	return dVector::m_zero;
}

//dFloat32 ndShapeCompoundConvex::RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, dFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const
dFloat32 ndShapeCompoundConvex::RayCast(ndRayCastNotify&, const dVector&, const dVector&, dFloat32, const ndBody* const, ndContactPoint&) const
{
	dAssert(0);
	return 0;
}

void ndShapeCompoundConvex::BeginAddRemove()
{
}

void ndShapeCompoundConvex::EndAddRemove(bool flushCache)
{
	dAssert(0);
	flushCache = false;
}

//ndShapeCompoundConvex::ndTreeArray::dTreeNode* ndShapeCompoundConvex::AddCollision(ndShapeInstance* const part)
ndShapeCompoundConvex::ndTreeArray::dTreeNode* ndShapeCompoundConvex::AddCollision(ndShapeInstance* const)
{
	dAssert(0);
	return nullptr;
}