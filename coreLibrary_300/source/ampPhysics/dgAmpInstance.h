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
#include "dgAmpAllocator.h"

using namespace concurrency;
using namespace concurrency::graphics;

class dgAmpJacobian
{
	public:
	float_4 m_linear;
	float_4 m_angular;
};

class dgAmpJacobianPair
{
	public:
	dgAmpJacobian m_jacobianM0;
	dgAmpJacobian m_jacobianM1;
};

class dgAmpMatrixRightSide
{
	public:
	union 
	{
		struct 
		{
//	float m_force;
//	float m_accel;
//	float m_deltaAccel;
//	float m_deltaForce;
//	float m_invDJMinvJt;
//	float m_maxImpact;
			float m_diagDamp;
			float m_restitution;
			float m_penetration;
			float m_penetrationStiffness;

			float m_coordenateAccel;
			float m_lowerBoundFrictionCoefficent;
			float m_upperBoundFrictionCoefficent;

			//dgForceImpactPair* m_jointFeebackForce;
			int m_jointFeebackForce[2];
			int m_normalForceIndex;
			int m_accelIsMotor;
		};
		float m_data[3][4];
	};
};


class dgAmpMatrix4x4 
{
	public:
	dgAmpMatrix4x4()
	{
	}

	dgAmpMatrix4x4 (const float_4& v0, const float_4& v1, const float_4& v2, const float_4& v3)  restrict(amp,cpu)
	{
		m_row[0] = v0;
		m_row[1] = v1;
		m_row[2] = v2;
		m_row[3] = v3;
	}
	float_4 m_row[4];
};


class dgAmpBodyData
{
	public:
	dgAmpBodyData (dgMemoryAllocator* const allocator);
	void Alloc (dgInt32 count);
	void CLean ();

	dgInt32 m_currentSize;
	array<float_4, 1> m_bodyInvMass;
	array<dgAmpJacobian, 1> m_bodyDamp;
	array<dgAmpJacobian, 1> m_bodyVelocity;
	array<dgAmpJacobian, 1> m_bodyNetForce;
	array<dgAmpJacobian, 1> m_bodyInternalForce;
	array<dgAmpMatrix4x4 , 1> m_bodyMatrix;
	array<dgAmpMatrix4x4 , 1> m_bodyInvInertiaMatrix;
	
	//std::vector<float_4, dgAmpAllocator<float_4>> m_bodyInvMassCpu;
	//std::vector<dgAmpJacobian, dgAmpAllocator<dgAmpJacobian>> m_bodyDampCpu;
	//std::vector<dgAmpMatrix4x4, dgAmpAllocator<dgAmpJacobian>> m_bodyMatrixCpu;
	//std::vector<dgAmpJacobian, dgAmpAllocator<dgAmpJacobian>> m_bodyVelocityCpu;
	array_view<float_4, 1> m_bodyInvMassCpu;
	array_view<dgAmpJacobian, 1> m_bodyDampCpu;
	array_view<dgAmpMatrix4x4, 1> m_bodyMatrixCpu;
	array_view<dgAmpJacobian, 1> m_bodyVelocityCpu;
};

class dgAmpConstraintData
{
	public:
	class dgAmpJacobianMatrixElement
	{
		public:
		dgAmpJacobianPair m_Jt;
		dgAmpMatrixRightSide m_data;
	};

	dgAmpConstraintData (dgMemoryAllocator* const allocator);
	void Alloc (dgInt32 size);
	void CLean ();

	dgInt32 m_currentSize;
	array<dgAmpJacobianMatrixElement, 1> m_matrixData;

	//std::vector<dgAmpJacobianMatrixElement, dgAmpAllocator<dgAmpJacobianMatrixElement>> m_matrixDataCpu;
	array_view<dgAmpJacobianMatrixElement, 1> m_matrixDataCpu;
};


class dgAmpInstance: public dgAmpBodyData, public dgAmpConstraintData
{
	public:
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
	static float_4 ToFloat4 (const dgVector& v);

	static dgAmpMatrix4x4 Transpose (const dgAmpMatrix4x4& matrix) restrict(amp,cpu);
	static dgAmpMatrix4x4 Multiply (const dgAmpMatrix4x4& matrixA, const dgAmpMatrix4x4& matrixB) restrict(amp,cpu);
	
	static float Dot (const float_4& vectorA, const float_4& vectorB) restrict(amp,cpu);
	static float_4 Scale (const float_4& vector, float scale) restrict(amp,cpu);
	static float_4 RotateVector (const dgAmpMatrix4x4& matrix, const float_4& vector) restrict(amp,cpu);
	static float_4 UnrotateVector (const dgAmpMatrix4x4& matrix, const float_4& vector) restrict(amp,cpu);
	static float_4 TransformVector (const dgAmpMatrix4x4& matrix, const float_4& vector) restrict(amp,cpu);
	static float_4 UntransformVector (const dgAmpMatrix4x4& matrix, const float_4& vector) restrict(amp,cpu);

	
	static void InitializeBodyArrayParallelKernel (void* const context, void* const worldContext, dgInt32 threadID);
	static void BuildJacobianMatrixParallelKernel (void* const context, void* const worldContext, dgInt32 threadID);
	void GetJacobianDerivativesParallel (dgJointInfo* const jointInfo, dgInt32 threadIndex, dgInt32 rowBase, dgFloat32 timestep);

	void InitilizeBodyArrayParallel (dgParallelSolverSyncData* const syncData); 
	void BuildJacobianMatrixParallel (dgParallelSolverSyncData* const syncData);
	void CreateParallelArrayBatchArrays (dgParallelSolverSyncData* const syncData, const dgIsland* const islandArray, dgInt32 islandCount);
	
	static void AddDamingAccelKernel (const dgAmpMatrix4x4& bodyMatrix, const dgAmpJacobian& damping, dgAmpJacobian& veloc) restrict(amp,cpu);
	static void CalcuateInvInertiaMatrixKernel (const dgAmpMatrix4x4& bodyMatrix, dgAmpMatrix4x4& invInertiaMatrix, const float_4& invInertia) restrict(amp,cpu);

	dgWorld* m_world;
	accelerator m_accelerator;
	dgList<dgAcceleratorDescription> m_acceleratorList;
};

inline float_4 dgAmpInstance::ToFloat4 (const dgVector& v)
{ 
	return float_4 (v[0], v[1], v[2], v[3]);
}

inline float dgAmpInstance::Dot (const float_4& vectorA, const float_4& vectorB) restrict(amp,cpu)
{
	return vectorA.get_x() * vectorB.get_x() + vectorA.get_y() * vectorB.get_y() + vectorA.get_z() * vectorB.get_z() + vectorA.get_w() * vectorB.get_w();
}

inline dgAmpMatrix4x4 dgAmpInstance::Transpose (const dgAmpMatrix4x4& matrix) restrict(amp,cpu)
{
	return dgAmpMatrix4x4 (float_4 (matrix.m_row[0].get_x(), matrix.m_row[1].get_x(), matrix.m_row[2].get_x(), matrix.m_row[3].get_x()),  
					  float_4 (matrix.m_row[0].get_y(), matrix.m_row[1].get_y(), matrix.m_row[2].get_y(), matrix.m_row[3].get_y()),
					  float_4 (matrix.m_row[0].get_z(), matrix.m_row[1].get_z(), matrix.m_row[2].get_z(), matrix.m_row[3].get_z()),
					  float_4 (matrix.m_row[0].get_w(), matrix.m_row[1].get_w(), matrix.m_row[2].get_w(), matrix.m_row[3].get_w()));
}


inline float_4 dgAmpInstance::Scale (const float_4& vector, float scale) restrict(amp,cpu)
{
	return float_4 (vector * float_4 (scale, scale, scale, scale));
}

inline float_4 dgAmpInstance::RotateVector (const dgAmpMatrix4x4& matrix, const float_4& vector) restrict(amp,cpu)
{
	return float_4 (Scale (matrix.m_row[0], vector.get_x()) + Scale (matrix.m_row[1], vector.get_y()) + Scale (matrix.m_row[2], vector.get_z()));
}

inline float_4 dgAmpInstance::UnrotateVector (const dgAmpMatrix4x4& matrix, const float_4& vector) restrict(amp,cpu)
{
	return float_4 (Dot (matrix.m_row[0], vector), Dot (matrix.m_row[1], vector), Dot (matrix.m_row[2], vector), vector.get_w());
}

inline float_4 dgAmpInstance::TransformVector (const dgAmpMatrix4x4& matrix, const float_4& vector) restrict(amp,cpu)
{
	return float_4 (Scale (matrix.m_row[0], vector.get_x()) + Scale (matrix.m_row[1], vector.get_y()) + Scale (matrix.m_row[2], vector.get_z()) + Scale (matrix.m_row[3], vector.get_w()));
}

inline float_4 dgAmpInstance::UntransformVector (const dgAmpMatrix4x4& matrix, const float_4& vector) restrict(amp,cpu)
{
	return UnrotateVector (matrix, vector - matrix.m_row[3]);
}

inline dgAmpMatrix4x4 dgAmpInstance::Multiply (const dgAmpMatrix4x4& matrixA, const dgAmpMatrix4x4& matrixB) restrict(amp,cpu)
{
	return dgAmpMatrix4x4 (TransformVector(matrixB, matrixA.m_row[0]), TransformVector(matrixB, matrixA.m_row[1]), TransformVector(matrixB, matrixA.m_row[2]), TransformVector(matrixB, matrixA.m_row[3])); 
}

#endif