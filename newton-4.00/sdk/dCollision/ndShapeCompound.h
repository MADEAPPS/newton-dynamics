/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __ND_SHAPE_COMPOUND_H__
#define __ND_SHAPE_COMPOUND_H__

#include "ndCollisionStdafx.h"
#include "ndShape.h"

class ndBodyKinematic;

#define D_COMPOUND_STACK_DEPTH	256

class ndShapeCompound: public ndShape
{
	public:
	enum ndNodeType
	{
		m_leaf,
		m_node,
	};

	class ndNodeBase;
	class ndTreeArray : public ndTree<ndNodeBase*, ndInt32, ndContainersFreeListAlloc<ndNodeBase*>>
	{
		public:
		ndTreeArray();
		void AddNode(ndNodeBase* const node, ndInt32 index, const ndShapeInstance* const parent);
	};

	D_CLASS_REFLECTION(ndShapeCompound);
	D_COLLISION_API ndShapeCompound();
	D_COLLISION_API ndShapeCompound(const ndLoadSaveBase::dLoadDescriptor& desc);
	D_COLLISION_API virtual ~ndShapeCompound();

	void SetOwner(const ndShapeInstance* const myInstance);

	const ndTreeArray& GetTree() const;

	D_COLLISION_API virtual void BeginAddRemove();
	D_COLLISION_API virtual ndTreeArray::ndNode* AddCollision(ndShapeInstance* const part);
	D_COLLISION_API virtual void EndAddRemove();

	protected:
	class ndSpliteInfo;
	ndShapeCompound(const ndShapeCompound& source, const ndShapeInstance* const myInstance);
	virtual ndShapeInfo GetShapeInfo() const;
	virtual void DebugShape(const ndMatrix& matrix, ndShapeDebugNotify& debugCallback) const;
	virtual ndFloat32 RayCast(ndRayCastNotify& callback, const ndVector& localP0, const ndVector& localP1, ndFloat32 maxT, const ndBody* const body, ndContactPoint& contactOut) const;
	virtual void Save(const ndLoadSaveBase::ndSaveDescriptor& desc) const;

	virtual ndFloat32 GetVolume() const;
	virtual ndFloat32 GetBoxMinRadius() const;
	virtual ndFloat32 GetBoxMaxRadius() const;

	virtual ndShapeCompound* GetAsShapeCompound();
	virtual ndVector SupportVertex(const ndVector& dir, ndInt32* const vertexIndex) const;
	virtual ndVector SupportVertexSpecial(const ndVector& dir, ndFloat32 skinThickness, ndInt32* const vertexIndex) const;
	virtual ndVector SupportVertexSpecialProjectPoint(const ndVector& point, const ndVector& dir) const;
	virtual ndInt32 CalculatePlaneIntersection(const ndVector& normal, const ndVector& point, ndVector* const contactsOut) const;
	virtual ndVector CalculateVolumeIntegral(const ndMatrix& globalMatrix, const ndVector& plane, const ndShapeInstance& parentScale) const;
	
	D_COLLISION_API virtual void CalculateAabb(const ndMatrix& matrix, ndVector& p0, ndVector& p1) const;
	//D_COLLISION_API ndInt32 CalculatePlaneIntersection(const ndFloat32* const vertex, const ndInt32* const index, ndInt32 indexCount, ndInt32 strideInFloat, const dPlane& localPlane, dVector* const contactsOut) const;

	virtual void MassProperties();
	void ApplyScale(const ndVector& scale);
	void SetSubShapeOwner(ndBodyKinematic* const body);
	void ImproveNodeFitness(ndNodeBase* const node) const;
	ndFloat64 CalculateEntropy(ndInt32 count, ndNodeBase** array);
	ndNodeBase* BuildTopDown(ndNodeBase** const leafArray, ndInt32 firstBox, ndInt32 lastBox, ndNodeBase** rootNodesMemory, ndInt32& rootIndex);
	ndNodeBase* BuildTopDownBig(ndNodeBase** const leafArray, ndInt32 firstBox, ndInt32 lastBox, ndNodeBase** rootNodesMemory, ndInt32& rootIndex);
	ndFloat32 CalculateSurfaceArea(ndNodeBase* const node0, ndNodeBase* const node1, ndVector& minBox, ndVector& maxBox) const;
	ndMatrix CalculateInertiaAndCenterOfMass(const ndMatrix& alignMatrix, const ndVector& localScale, const ndMatrix& matrix) const;
	ndFloat32 CalculateMassProperties(const ndMatrix& offset, ndVector& inertia, ndVector& crossInertia, ndVector& centerOfMass) const;

	ndTreeArray m_array;
	ndFloat64 m_treeEntropy;
	ndFloat32 m_boxMinRadius;
	ndFloat32 m_boxMaxRadius;
	ndNodeBase* m_root;
	const ndShapeInstance* m_myInstance;
	ndInt32 m_idIndex;

	friend class ndBodyKinematic;
	friend class ndShapeInstance;
	friend class ndContactSolver;
};

inline ndShapeCompound* ndShapeCompound::GetAsShapeCompound()
{ 
	return this; 
}

inline void ndShapeCompound::SetOwner(const ndShapeInstance* const instance)
{
	m_myInstance = instance;
}

class ndShapeCompound::ndNodeBase: public ndClassAlloc
{
	public:
	ndNodeBase();
	ndNodeBase(const ndNodeBase& copyFrom);
	ndNodeBase(ndShapeInstance* const instance);
	ndNodeBase(ndNodeBase* const left, ndNodeBase* const right);
	~ndNodeBase();

	//void Sanity(int level = 0);
	void CalculateAABB();
	ndShapeInstance* GetShape() const;
	void SetBox(const ndVector& p0, const ndVector& p1);

	ndVector m_p0;
	ndVector m_p1;
	ndVector m_size;
	ndVector m_origin;
	ndFloat32 m_area;
	ndInt32 m_type;
	ndNodeBase* m_left;
	ndNodeBase* m_right;
	ndNodeBase* m_parent;
	ndTreeArray::ndNode* m_myNode;
	ndShapeInstance* m_shapeInstance;
};

inline ndShapeCompound::ndNodeBase::ndNodeBase()
	:ndClassAlloc()
	,m_type(m_node)
	,m_left(nullptr)
	,m_right(nullptr)
	,m_parent(nullptr)
	,m_myNode(nullptr)
	,m_shapeInstance(nullptr)
{
}

inline ndShapeCompound::ndNodeBase::ndNodeBase(const ndNodeBase& copyFrom)
	:ndClassAlloc()
	,m_p0(copyFrom.m_p0)
	,m_p1(copyFrom.m_p1)
	,m_size(copyFrom.m_size)
	,m_origin(copyFrom.m_origin)
	,m_area(copyFrom.m_area)
	,m_type(copyFrom.m_type)
	,m_left(nullptr)
	,m_right(nullptr)
	,m_parent(nullptr)
	,m_myNode(nullptr)
	,m_shapeInstance(nullptr)
{
	dAssert(!copyFrom.m_shapeInstance);
}

inline ndShapeCompound::ndNodeBase::ndNodeBase(ndShapeInstance* const instance)
	:ndClassAlloc()
	,m_type(m_leaf)
	,m_left(nullptr)
	,m_right(nullptr)
	,m_parent(nullptr)
	,m_myNode(nullptr)
	,m_shapeInstance(new ndShapeInstance(*instance))
{
	CalculateAABB();
}

inline ndShapeCompound::ndNodeBase::ndNodeBase(ndNodeBase* const left, ndNodeBase* const right)
	:ndClassAlloc()
	,m_type(m_node)
	,m_left(left)
	,m_right(right)
	,m_parent(nullptr)
	,m_myNode(nullptr)
	,m_shapeInstance(nullptr)
{
	m_left->m_parent = this;
	m_right->m_parent = this;

	ndVector p0(left->m_p0.GetMin(right->m_p0));
	ndVector p1(left->m_p1.GetMax(right->m_p1));
	SetBox(p0, p1);
}

inline ndShapeInstance* ndShapeCompound::ndNodeBase::GetShape() const
{
	return m_shapeInstance;
}

inline void ndShapeCompound::ndNodeBase::CalculateAABB()
{
	ndVector p0;
	ndVector p1;
	m_shapeInstance->CalculateAabb(m_shapeInstance->GetLocalMatrix(), p0, p1);
	SetBox(p0, p1);
}

inline void ndShapeCompound::ndNodeBase::SetBox(const ndVector& p0, const ndVector& p1)
{
	m_p0 = p0;
	m_p1 = p1;
	dAssert(m_p0.m_w == ndFloat32(0.0f));
	dAssert(m_p1.m_w == ndFloat32(0.0f));
	m_size = ndVector::m_half * (m_p1 - m_p0);
	m_origin = ndVector::m_half * (m_p1 + m_p0);
	m_area = m_size.DotProduct(m_size.ShiftTripleRight()).m_x;
}

inline const ndShapeCompound::ndTreeArray& ndShapeCompound::GetTree() const
{
	return m_array;
}

#endif 



