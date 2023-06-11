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

#ifndef __ND_MESH_EFFECT_NODE_H__
#define __ND_MESH_EFFECT_NODE_H__

#include "ndCollisionStdafx.h"
#include "ndMeshEffect.h"

class ndMesh : public ndNodeHierarchy<ndMesh>
{
	public:
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

	D_COLLISION_API ndMesh(ndMesh* const parent);
	D_COLLISION_API ndMesh(const ndMesh& src);

	D_COLLISION_API ~ndMesh();
	D_COLLISION_API ndMesh* CreateClone() const;

	D_COLLISION_API ndSharedPtr<ndMeshEffect>& GetMesh();
	D_COLLISION_API const ndSharedPtr<ndMeshEffect>& GetMesh() const;
	D_COLLISION_API void SetMesh(const ndSharedPtr<ndMeshEffect>& mesh);

	D_COLLISION_API const ndString& GetName() const;
	D_COLLISION_API void SetName(const ndString& name);

	D_COLLISION_API ndCurve& GetScaleCurve();
	D_COLLISION_API ndCurve& GetPositCurve();
	D_COLLISION_API ndCurve& GetRotationCurve();

	D_COLLISION_API const ndCurve& GetScaleCurve() const;
	D_COLLISION_API const ndCurve& GetPositCurve() const;
	D_COLLISION_API const ndCurve& GetRotationCurve() const;

	D_COLLISION_API void ApplyTransform(const ndMatrix& transform);
	D_COLLISION_API ndMatrix CalculateGlobalMatrix(ndMesh* const parent = nullptr) const;

	D_COLLISION_API static ndMesh* Load(const char* const fullPathName);
	D_COLLISION_API static void Save(const ndMesh* const mesh, const char* const fullPathName);

	ndMatrix m_matrix;
	ndMatrix m_meshMatrix;

	private:
	void Load(FILE* const file, const ndTree<ndSharedPtr<ndMeshEffect>, ndInt32>& meshEffects);
	void Save(FILE* const file, const ndTree<ndInt32, const ndMeshEffect*>& meshEffects, ndInt32 level = 0) const;

	protected:
	ndString m_name;
	ndSharedPtr<ndMeshEffect> m_mesh;
	ndCurve m_scale;
	ndCurve m_posit;
	ndCurve m_rotation;
};

#endif

