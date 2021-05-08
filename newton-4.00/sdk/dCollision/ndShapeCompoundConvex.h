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

#ifndef __D_SHAPE_COMPOUND_CONVEX_H__
#define __D_SHAPE_COMPOUND_CONVEX_H__

#include "ndCollisionStdafx.h"
#include "ndShape.h"


class ndShapeCompoundConvex: public ndShape
{
	public:
	enum ndNodeType
	{
		m_leaf,
		m_node,
	};

	protected:
	class ndNodeBase;
	public:

	class ndTreeArray : public dTree<ndNodeBase*, dInt32, dContainersFreeListAlloc<ndNodeBase*>>
	{
		public:
		ndTreeArray();
		void AddNode(ndNodeBase* const node, dInt32 index, const ndShapeInstance* const parent);
	};


	D_COLLISION_API ndShapeCompoundConvex();
	D_COLLISION_API ndShapeCompoundConvex(const nd::TiXmlNode* const xmlNode);
	D_COLLISION_API virtual ~ndShapeCompoundConvex();

	void SetOwner(const ndShapeInstance* const myInstance);

	D_COLLISION_API virtual void BeginAddRemove();
	D_COLLISION_API virtual ndTreeArray::dTreeNode* AddCollision(ndShapeInstance* const part);
	D_COLLISION_API virtual void EndAddRemove();

	protected:
	class ndNodeBase
	{
		public:
		//ndNodeBase();
		//ndNodeBase(const ndNodeBase& copyFrom);
		ndNodeBase(ndShapeInstance* const instance);
		ndNodeBase(ndNodeBase* const left, ndNodeBase* const right);
		//~ndNodeBase();

		void CalculateAABB();
		void SetBox(const dVector& p0, const dVector& p1);
		//bool BoxTest(const dgOOBBTestData& data) const;
		//bool BoxTest(const dgOOBBTestData& data, const ndNodeBase* const otherNode) const;
		//dFloat32 RayBoxDistance(const dgOOBBTestData& data, const dgFastRayTest& myRay, const dgFastRayTest& otherRay, const ndNodeBase* const otherNode) const;
		//
		//DG_INLINE ndShapeInstance* GetShape() const
		//{
		//	return m_shape;
		//}
		//
		//DG_INLINE dgInt32 BoxIntersect(const dgFastRayTest& ray, const dVector& boxP0, const dVector& boxP1) const
		//{
		//	dVector minBox(m_p0 - boxP1);
		//	dVector maxBox(m_p1 - boxP0);
		//	return ray.BoxTest(minBox, maxBox);
		//}
		
		dVector m_p0;
		dVector m_p1;
		dVector m_size;
		dVector m_origin;
		dFloat32 m_area;
		dInt32 m_type;
		ndNodeBase* m_left;
		ndNodeBase* m_right;
		ndNodeBase* m_parent;
		ndShapeInstance* m_shape;
		ndTreeArray::dTreeNode* m_myNode;
	};

	virtual ndShapeInfo GetShapeInfo() const;
	virtual void DebugShape(const dMatrix& matrix, ndShapeDebugCallback& debugCallback) const;
	virtual dFloat32 RayCast(ndRayCastNotify& callback, const dVector& localP0, const dVector& localP1, dFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const;

	virtual dFloat32 GetVolume() const;
	virtual dFloat32 GetBoxMinRadius() const;
	virtual dFloat32 GetBoxMaxRadius() const;

	virtual ndShapeCompoundConvex* GetAsShapeCompoundConvex();
	virtual dVector SupportVertex(const dVector& dir, dInt32* const vertexIndex) const;
	virtual dVector SupportVertexSpecial(const dVector& dir, dFloat32 skinThickness, dInt32* const vertexIndex) const;
	virtual dVector SupportVertexSpecialProjectPoint(const dVector& point, const dVector& dir) const;
	virtual dInt32 CalculatePlaneIntersection(const dVector& normal, const dVector& point, dVector* const contactsOut) const;
	virtual dVector CalculateVolumeIntegral(const dMatrix& globalMatrix, const dVector& plane, const ndShapeInstance& parentScale) const;
	
	D_COLLISION_API virtual void CalcAABB(const dMatrix& matrix, dVector& p0, dVector& p1) const;
	//D_COLLISION_API dInt32 CalculatePlaneIntersection(const dFloat32* const vertex, const dInt32* const index, dInt32 indexCount, dInt32 strideInFloat, const dPlane& localPlane, dVector* const contactsOut) const;

	void ImproveNodeFitness(ndNodeBase* const node) const;
	dFloat64 CalculateEntropy(dInt32 count, ndNodeBase** array);
	dFloat32 CalculateSurfaceArea(ndNodeBase* const node0, ndNodeBase* const node1, dVector& minBox, dVector& maxBox) const;

	ndTreeArray m_array;
	dFloat64 m_treeEntropy;
	dFloat32 m_boxMinRadius;
	dFloat32 m_boxMaxRadius;
	ndNodeBase* m_root;
	const ndShapeInstance* m_myInstance;
	dInt32 m_idIndex;
};

inline ndShapeCompoundConvex* ndShapeCompoundConvex::GetAsShapeCompoundConvex()
{ 
	return this; 
}

inline void ndShapeCompoundConvex::SetOwner(const ndShapeInstance* const instance)
{
	m_myInstance = instance;
}

inline dFloat32 ndShapeCompoundConvex::CalculateSurfaceArea(ndNodeBase* const node0, ndNodeBase* const node1, dVector& minBox, dVector& maxBox) const
{
	minBox = node0->m_p0.GetMin(node1->m_p0);
	maxBox = node0->m_p1.GetMax(node1->m_p1);
	dVector side0(dVector::m_half * (maxBox - minBox));
	return side0.DotProduct(side0.ShiftTripleRight()).GetScalar();
}


#endif 



