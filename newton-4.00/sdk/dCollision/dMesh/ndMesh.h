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

#include "ndCore.h"
//#include "ndCollisionStdafx.h"
//#include "ndMatrix.h"
//#include "ndClassAlloc.h"
//#include "ndSharedPtr.h"

class ndMeshEffect;
class ndShapeInstance;

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

	D_COLLISION_API ndMesh();
	D_COLLISION_API ndMesh(const ndMesh& src);
	D_COLLISION_API ndMesh(const ndShapeInstance& src);

	D_COLLISION_API virtual ~ndMesh();
	D_COLLISION_API ndMesh* CreateClone() const;

	D_COLLISION_API void AddChild(const ndSharedPtr<ndMesh>& child);
	D_COLLISION_API void RemoveChild(const ndSharedPtr<ndMesh>& child);

	D_COLLISION_API ndMesh* GetParent();
	D_COLLISION_API const ndMesh* GetParent() const;

	D_COLLISION_API ndList<ndSharedPtr<ndMesh>>& GetChildren();
	D_COLLISION_API const ndList<ndSharedPtr<ndMesh>>& GetChildren() const;

	D_COLLISION_API ndMesh* IteratorNext();
	D_COLLISION_API ndMesh* IteratorFirst();

	D_COLLISION_API ndMesh* FindByName(const ndString& name) const;
	D_COLLISION_API ndMesh* FindByClosestMatch(const ndString& name) const;

	D_COLLISION_API ndSharedPtr<ndMeshEffect>& GetMesh();
	D_COLLISION_API const ndSharedPtr<ndMeshEffect>& GetMesh() const;
	D_COLLISION_API void SetMesh(const ndSharedPtr<ndMeshEffect>& mesh);

	D_COLLISION_API const ndString& GetName() const;
	D_COLLISION_API void SetName(const ndString& name);

	D_COLLISION_API ndNodeType GetNodeType() const;
	D_COLLISION_API void SetNodeType(ndNodeType type);

	D_COLLISION_API ndVector GetBoneTarget() const;
	D_COLLISION_API void SetBoneTarget(const ndVector& target);

	D_COLLISION_API ndCurve& GetScaleCurve();
	D_COLLISION_API ndCurve& GetPositCurve();
	D_COLLISION_API ndCurve& GetRotationCurve();

	D_COLLISION_API const ndCurve& GetScaleCurve() const;
	D_COLLISION_API const ndCurve& GetPositCurve() const;
	D_COLLISION_API const ndCurve& GetRotationCurve() const;

	D_COLLISION_API void ApplyTransform(const ndMatrix& transform);
	D_COLLISION_API ndMatrix CalculateGlobalMatrix(ndMesh* const parent = nullptr) const;

	D_COLLISION_API ndSharedPtr<ndShapeInstance> CreateCollision();
	D_COLLISION_API ndSharedPtr<ndShapeInstance> CreateCollisionFromChildren();

	D_COLLISION_API ndSharedPtr<ndShapeInstance> CreateCollisionBox();
	D_COLLISION_API ndSharedPtr<ndShapeInstance> CreateCollisionTire();
	D_COLLISION_API ndSharedPtr<ndShapeInstance> CreateCollisionSphere();
	D_COLLISION_API ndSharedPtr<ndShapeInstance> CreateCollisionConvex();
	D_COLLISION_API ndSharedPtr<ndShapeInstance> CreateCollisionCapsule();
	D_COLLISION_API ndSharedPtr<ndShapeInstance> CreateCollisionChamferCylinder();
	D_COLLISION_API ndSharedPtr<ndShapeInstance> CreateCollisionTree(bool optimize = true);
	D_COLLISION_API ndSharedPtr<ndShapeInstance> CreateCollisionCompound(bool lowDetail = false);

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

