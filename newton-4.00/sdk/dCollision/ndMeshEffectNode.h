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

class ndMeshEffectNode : public ndNodeHierarchy<ndMeshEffectNode>
{
	public:
	class ndCurveValue
	{
		public:
		ndFloat32 m_x;
		ndFloat32 m_y;
		ndFloat32 m_z;
		ndFloat32 m_time;
	};

	class ndCurve: public ndList<ndCurveValue>
	{
		public:
		ndCurve()
			:ndList<ndCurveValue>()
		{
		}
	};

	D_COLLISION_API ndMeshEffectNode(ndMeshEffectNode* const parent);
	D_COLLISION_API ndMeshEffectNode(const ndMeshEffectNode& src);

	D_COLLISION_API ~ndMeshEffectNode();
	D_COLLISION_API ndMeshEffectNode* CreateClone() const;

	D_COLLISION_API ndSharedPtr<ndMeshEffect> GetMesh();
	D_COLLISION_API void SetMesh(const ndSharedPtr<ndMeshEffect>& mesh);

	D_COLLISION_API const ndString& GetName() const;
	D_COLLISION_API void SetName(const ndString& name);

	D_COLLISION_API ndCurve& GetScaleCurve();
	D_COLLISION_API ndCurve& GetPositCurve();
	D_COLLISION_API ndCurve& GetRotationCurve();

	ndMatrix m_matrix;
	ndMatrix m_meshMatrix;

	protected:
	ndString m_name;
	ndSharedPtr<ndMeshEffect> m_mesh;
	ndCurve m_scale;
	ndCurve m_posit;
	ndCurve m_rotation;
};

#endif

