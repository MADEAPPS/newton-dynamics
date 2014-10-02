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

#ifndef _DG_AMP_INSTANCE_H_
#define _DG_AMP_INSTANCE_H_

#include "dgAMP.h"

using namespace concurrency;
using namespace concurrency::graphics;

class dgAmpInstance
{
	public:
	class Jacobian
	{
		public:
		float_4 m_linear;
		float_4 m_angular;
	};

	class Matrix4x4 
	{
		public:
		Matrix4x4 (const float_4& v0, const float_4& v1, const float_4& v2, const float_4& v3)  restrict(amp,cpu)
		{
			m_row[0] = v0;
			m_row[1] = v1;
			m_row[2] = v2;
			m_row[3] = v3;
		}

		float_4 m_row[4];
	};


	class dgAcceleratorDescription
	{
		public:
		char m_path[128];
		char m_description[128];
	};

	DG_CLASS_ALLOCATOR(allocator);

	dgAmpInstance(dgWorld* const world);
	~dgAmpInstance(void);

	void CleanUp();
	dgInt32 GetPlatformsCount() const;
	void SelectPlaform(dgInt32 deviceIndex);
	void GetVendorString(dgInt32 deviceIndex, char* const name, dgInt32 maxlength) const;

	void ConstraintSolver (dgInt32 islandCount, const dgIsland* const islandArray, dgFloat32 timestep);
	
	private:
	float_4 ToFloat4 (const dgVector& v) const;
	static Matrix4x4 Transpose (const Matrix4x4& matrix) restrict(amp,cpu);
	static Matrix4x4 Multiply (const Matrix4x4& matrixA, const Matrix4x4& matrixB) restrict(amp,cpu);
	
	static float Dot (const float_4& vectorA, const float_4& vectorB) restrict(amp,cpu);
	static float_4 Scale (const float_4& vector, float scale) restrict(amp,cpu);
	static float_4 RotateVector (const Matrix4x4& matrix, const float_4& vector) restrict(amp,cpu);
	static float_4 UnrotateVector (const Matrix4x4& matrix, const float_4& vector) restrict(amp,cpu);
	static float_4 TransformVector (const Matrix4x4& matrix, const float_4& vector) restrict(amp,cpu);
	static float_4 UntransformVector (const Matrix4x4& matrix, const float_4& vector) restrict(amp,cpu);


	void InitilizeBodyArrayParallel (dgParallelSolverSyncData* const syncData); 
	void CreateParallelArrayBatchArrays (dgParallelSolverSyncData* const syncData, const dgIsland* const islandArray, dgInt32 islandCount);

	void AllocateBodyArray (dgInt32 count);
	
	static void AddDamingAccelKernel (const Matrix4x4& bodyMatrix, const Jacobian& damping, Jacobian& veloc) restrict(amp,cpu);
	static void CalcuateInvInertiaMatrixKernel (const Matrix4x4& bodyMatrix, Matrix4x4& invInertiaMatrix, const float_4& invInertia) restrict(amp,cpu);

	dgWorld* m_world;
	accelerator m_accelerator;
	dgList<dgAcceleratorDescription> m_acceleratorList;
	
	array<float_4, 1> m_bodyInvMass;
	array<Jacobian, 1> m_bodyDamp;
	array<Jacobian, 1> m_bodyVelocity;
	array<Jacobian, 1> m_bodyNetForce;
	array<Jacobian, 1> m_bodyInternalForce;
	array<Matrix4x4 , 1> m_bodyMatrix;
	array<Matrix4x4 , 1> m_bodyInvInertiaMatrix;
	
	array_view<Jacobian, 1> m_bodyDamp_view;
	array_view<float_4, 1> m_bodyInvMass_view;
	array_view<Jacobian, 1> m_bodyVelocity_view;
	array_view<Matrix4x4, 1> m_bodyMatrix_view;
};

inline float_4 dgAmpInstance::ToFloat4 (const dgVector& v) const 
{ 
	return float_4 (v[0], v[1], v[2], v[3]);
}

inline float dgAmpInstance::Dot (const float_4& vectorA, const float_4& vectorB) restrict(amp,cpu)
{
	return vectorA.get_x() * vectorB.get_x() + vectorA.get_y() * vectorB.get_y() + vectorA.get_z() * vectorB.get_z() + vectorA.get_w() * vectorB.get_w();
}

inline dgAmpInstance::Matrix4x4 dgAmpInstance::Transpose (const Matrix4x4& matrix) restrict(amp,cpu)
{
	return Matrix4x4 (float_4 (matrix.m_row[0].get_x(), matrix.m_row[1].get_x(), matrix.m_row[2].get_x(), matrix.m_row[3].get_x()),  
					  float_4 (matrix.m_row[0].get_y(), matrix.m_row[1].get_y(), matrix.m_row[2].get_y(), matrix.m_row[3].get_y()),
					  float_4 (matrix.m_row[0].get_z(), matrix.m_row[1].get_z(), matrix.m_row[2].get_z(), matrix.m_row[3].get_z()),
					  float_4 (matrix.m_row[0].get_w(), matrix.m_row[1].get_w(), matrix.m_row[2].get_w(), matrix.m_row[3].get_w()));
}


inline float_4 dgAmpInstance::Scale (const float_4& vector, float scale) restrict(amp,cpu)
{
	return float_4 (vector * float_4 (scale, scale, scale, scale));
}

inline float_4 dgAmpInstance::RotateVector (const Matrix4x4& matrix, const float_4& vector) restrict(amp,cpu)
{
	return float_4 (Scale (matrix.m_row[0], vector.get_x()) + Scale (matrix.m_row[1], vector.get_y()) + Scale (matrix.m_row[2], vector.get_z()));
}

inline float_4 dgAmpInstance::UnrotateVector (const Matrix4x4& matrix, const float_4& vector) restrict(amp,cpu)
{
	return float_4 (Dot (matrix.m_row[0], vector), Dot (matrix.m_row[1], vector), Dot (matrix.m_row[2], vector), vector.get_w());
}

inline float_4 dgAmpInstance::TransformVector (const Matrix4x4& matrix, const float_4& vector) restrict(amp,cpu)
{
	return float_4 (Scale (matrix.m_row[0], vector.get_x()) + Scale (matrix.m_row[1], vector.get_y()) + Scale (matrix.m_row[2], vector.get_z()) + Scale (matrix.m_row[3], vector.get_w()));
}

inline float_4 dgAmpInstance::UntransformVector (const Matrix4x4& matrix, const float_4& vector) restrict(amp,cpu)
{
	return UnrotateVector (matrix, vector - matrix.m_row[3]);
}

inline dgAmpInstance::Matrix4x4 dgAmpInstance::Multiply (const Matrix4x4& matrixA, const Matrix4x4& matrixB) restrict(amp,cpu)
{
	return Matrix4x4 (TransformVector(matrixB, matrixA.m_row[0]), TransformVector(matrixB, matrixA.m_row[1]), TransformVector(matrixB, matrixA.m_row[2]), TransformVector(matrixB, matrixA.m_row[3])); 
}

#endif