/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#ifndef __ND_MESH_H__
#define __ND_MESH_H__

#include "ndCollisionStdafx.h"
#include "ndMeshEffect.h"

class ndMesh : public ndClassAlloc
{
	public:
	enum ndNodeType
	{
		m_node,
		m_bone,
		m_boneEnd
	};
	
	class ndCurveValue
	{
		public:
		ndReal m_x;
		ndReal m_y;
		ndReal m_z;
		ndReal m_time;
	};

	class ndCurve: public ndList<ndCurveValue>
	{
		public:
		ndCurve()
			:ndList<ndCurveValue>()
			,m_lenght(ndFloat32 (0.0f))
		{
		}
		ndReal m_lenght;
	};

	ndMesh();
	ndMesh(const ndMesh& src);
	ndMesh(const ndShapeInstance& src);

	virtual ~ndMesh();
	ndMesh* CreateClone() const;

	void AddChild(const ndSharedPtr<ndMesh>& child);
	void RemoveChild(const ndSharedPtr<ndMesh>& child);

	ndMesh* GetParent();
	const ndMesh* GetParent() const;

	ndList<ndSharedPtr<ndMesh>>& GetChildren();
	const ndList<ndSharedPtr<ndMesh>>& GetChildren() const;

	ndMesh* IteratorNext();
	ndMesh* IteratorFirst();

	ndMesh* FindByName(const ndString& name) const;
	ndMesh* FindByClosestMatch(const ndString& name) const;

	ndSharedPtr<ndMeshEffect>& GetMesh();
	const ndSharedPtr<ndMeshEffect>& GetMesh() const;
	void SetMesh(const ndSharedPtr<ndMeshEffect>& mesh);

	const ndString& GetName() const;
	void SetName(const ndString& name);

	ndNodeType GetNodeType() const;
	void SetNodeType(ndNodeType type);

	ndVector GetBoneTarget() const;
	void SetBoneTarget(const ndVector& target);

	ndCurve& GetScaleCurve();
	ndCurve& GetPositCurve();
	ndCurve& GetRotationCurve();

	const ndCurve& GetScaleCurve() const;
	const ndCurve& GetPositCurve() const;
	const ndCurve& GetRotationCurve() const;

	void ApplyTransform(const ndMatrix& transform);
	ndMatrix CalculateGlobalMatrix(ndMesh* const parent = nullptr) const;

	ndSharedPtr<ndShapeInstance> CreateCollision();
	ndSharedPtr<ndShapeInstance> CreateCollisionFromChildren();

	ndSharedPtr<ndShapeInstance> CreateCollisionBox();
	ndSharedPtr<ndShapeInstance> CreateCollisionTire();
	ndSharedPtr<ndShapeInstance> CreateCollisionSphere();
	ndSharedPtr<ndShapeInstance> CreateCollisionConvex();
	ndSharedPtr<ndShapeInstance> CreateCollisionCapsule();
	ndSharedPtr<ndShapeInstance> CreateCollisionChamferCylinder();
	ndSharedPtr<ndShapeInstance> CreateCollisionTree(bool optimize = true);
	ndSharedPtr<ndShapeInstance> CreateCollisionCompound(bool lowDetail = false);

	ndMatrix m_matrix;

	protected:
	ndMatrix CalculateLocalMatrix(ndVector& size) const;

	ndString m_name;
	ndSharedPtr<ndMeshEffect> m_mesh;
	ndCurve m_scale;
	ndCurve m_posit;
	ndCurve m_rotation;
	ndMesh* m_parent;
	ndList<ndSharedPtr<ndMesh>> m_children;
	ndList<ndSharedPtr<ndMesh>>::ndNode* m_childNode;
	ndVector m_boneTarget;
	ndNodeType m_type;

	friend class ndMeshFile;
	friend class ndMeshLoader;
};
#endif

