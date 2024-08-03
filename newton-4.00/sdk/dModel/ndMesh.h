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

	
	ndMesh(const ndMesh& src);
	ndMesh(ndMesh* const parent);
	ndMesh(const ndShapeInstance& src);

	~ndMesh();
	ndMesh* CreateClone() const;

	ndSharedPtr<ndMeshEffect>& GetMesh();
	const ndSharedPtr<ndMeshEffect>& GetMesh() const;
	void SetMesh(const ndSharedPtr<ndMeshEffect>& mesh);

	const ndString& GetName() const;
	void SetName(const ndString& name);

	ndCurve& GetScaleCurve();
	ndCurve& GetPositCurve();
	ndCurve& GetRotationCurve();

	const ndCurve& GetScaleCurve() const;
	const ndCurve& GetPositCurve() const;
	const ndCurve& GetRotationCurve() const;

	void ApplyTransform(const ndMatrix& transform);
	ndMatrix CalculateGlobalMatrix(ndMesh* const parent = nullptr) const;

	ndMatrix m_matrix;
	ndMatrix m_meshMatrix;

	protected:
	ndString m_name;
	ndSharedPtr<ndMeshEffect> m_mesh;
	ndCurve m_scale;
	ndCurve m_posit;
	ndCurve m_rotation;

	friend class ndMeshFile;
};
#endif

