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

#include "ndDynamicsUpdateAvx2.h"

#define D_AVX_WORK_GROUP			8 
#define D_AVX_DEFAULT_BUFFER_SIZE	1024

#ifdef D_NEWTON_USE_DOUBLE
	D_MSV_NEWTON_ALIGN_32
	class ndAvxFloat
	{
		public:
		inline ndAvxFloat()
		{
		}

		inline ndAvxFloat(const ndFloat32 val)
			:m_low(_mm256_set1_pd(val))
			,m_high(_mm256_set1_pd(val))
		{
		}

		inline ndAvxFloat(const ndInt32 val)
			:m_low(_mm256_castsi256_pd(_mm256_set1_epi64x(dInt64(val))))
			,m_high(_mm256_castsi256_pd(_mm256_set1_epi64x(dInt64(val))))
		{
		}
				
		inline ndAvxFloat(const __m256d low, const __m256d high)
			:m_low(low)
			,m_high(high)
		{
		}

		inline ndAvxFloat(const ndAvxFloat& copy)
			:m_low(copy.m_low)
			,m_high(copy.m_high)
		{
		}

		#ifdef D_USE_VECTOR_AVX
			inline ndAvxFloat(const ndVector& low, const ndVector& high)
				:m_low(low.m_type)
				,m_high(high.m_type)
			{
			}
		#else
			inline ndAvxFloat(const ndVector& low, const ndVector& high)
				:m_low(_mm256_set_m128d(low.m_typeHigh, low.m_typeLow))
				,m_high(_mm256_set_m128d(high.m_typeHigh, high.m_typeLow))
			{
			}
		#endif

		inline ndAvxFloat(const ndAvxFloat* const baseAddr, const ndAvxFloat& index)
			:m_low(_mm256_i64gather_pd(&(*baseAddr)[0], index.m_lowInt, 8))
			,m_high(_mm256_i64gather_pd(&(*baseAddr)[0], index.m_highInt, 8))
		{
		}

		inline ndFloat32& operator[] (ndInt32 i)
		{
			dAssert(i >= 0);
			dAssert(i < D_AVX_WORK_GROUP);
			ndFloat32* const ptr = (ndFloat32*)&m_low;
			return ptr[i];
		}

		inline const ndFloat32& operator[] (ndInt32 i) const
		{
			dAssert(i >= 0);
			dAssert(i < D_AVX_WORK_GROUP);
			const ndFloat32* const ptr = (ndFloat32*)&m_low;
			return ptr[i];
		}

		inline ndAvxFloat operator+ (const ndAvxFloat& A) const
		{
			return ndAvxFloat(_mm256_add_pd(m_low, A.m_low), _mm256_add_pd(m_high, A.m_high));
		}

		inline ndAvxFloat operator- (const ndAvxFloat& A) const
		{
			return ndAvxFloat(_mm256_sub_pd(m_low, A.m_low), _mm256_sub_pd(m_high, A.m_high));
		}

		inline ndAvxFloat operator* (const ndAvxFloat& A) const
		{
			return ndAvxFloat(_mm256_mul_pd(m_low, A.m_low), _mm256_mul_pd(m_high, A.m_high));
		}

		inline ndAvxFloat MulAdd(const ndAvxFloat& A, const ndAvxFloat& B) const
		{
			return ndAvxFloat(_mm256_fmadd_pd(A.m_low, B.m_low, m_low), _mm256_fmadd_pd(A.m_high, B.m_high, m_high));
		}

		inline ndAvxFloat MulSub(const ndAvxFloat& A, const ndAvxFloat& B) const
		{
			return ndAvxFloat(_mm256_fnmadd_pd(A.m_low, B.m_low, m_low), _mm256_fnmadd_pd(A.m_high, B.m_high, m_high));
		}

		inline ndAvxFloat operator> (const ndAvxFloat& A) const
		{
			return ndAvxFloat(_mm256_cmp_pd(m_low, A.m_low, _CMP_GT_OQ), _mm256_cmp_pd(m_high, A.m_high, _CMP_GT_OQ));
		}

		inline ndAvxFloat operator< (const ndAvxFloat& A) const
		{
			return ndAvxFloat(_mm256_cmp_pd(m_low, A.m_low, _CMP_LT_OQ), _mm256_cmp_pd(m_high, A.m_high, _CMP_LT_OQ));
		}

		inline ndAvxFloat operator| (const ndAvxFloat& A) const
		{
			return ndAvxFloat(_mm256_or_pd(m_low, A.m_low), _mm256_or_pd(m_high, A.m_high));
		}

		inline ndAvxFloat operator& (const ndAvxFloat& A) const
		{
			return ndAvxFloat(_mm256_and_pd(m_low, A.m_low), _mm256_and_pd(m_high, A.m_high));
		}

		inline ndAvxFloat GetMin(const ndAvxFloat& A) const
		{
			return ndAvxFloat(_mm256_min_pd(m_low, A.m_low), _mm256_min_pd(m_high, A.m_high));
		}

		inline ndAvxFloat GetMax(const ndAvxFloat& A) const
		{
			return ndAvxFloat(_mm256_max_pd(m_low, A.m_low), _mm256_max_pd(m_high, A.m_high));
		}

		inline ndAvxFloat Select(const ndAvxFloat& data, const ndAvxFloat& mask) const
		{
			// (((b ^ a) & mask)^a)
			//return  _mm_or_ps (_mm_and_ps (mask.m_type, data.m_type), _mm_andnot_ps(mask.m_type, m_type));
			//return  _mm256_xor_ps(m_type, _mm256_and_ps(mask.m_type, _mm256_xor_ps(m_type, data.m_type)));
			__m256d low (_mm256_xor_pd(m_low, _mm256_and_pd(mask.m_low, _mm256_xor_pd(m_low, data.m_low))));
			__m256d high(_mm256_xor_pd(m_high, _mm256_and_pd(mask.m_high, _mm256_xor_pd(m_high, data.m_high))));
			return ndAvxFloat(low, high);
		}

		inline ndFloat32 AddHorizontal() const
		{
			//__m256 tmp0(_mm256_add_ps(m_type, _mm256_permute2f128_ps(m_type, m_type, 1)));
			//__m256 tmp1(_mm256_hadd_ps(tmp0, tmp0));
			//__m256 tmp2(_mm256_hadd_ps(tmp1, tmp1));
			//return *((ndFloat32*)&tmp2);
			__m256d tmp0(_mm256_add_pd(m_low, m_high));
			__m256d tmp1(_mm256_hadd_pd(tmp0, tmp0));
			__m256d tmp2(_mm256_add_pd(tmp1, _mm256_permute2f128_pd(tmp1, tmp1, 1)));
			return *((ndFloat32*)&tmp2);
		}

		static inline void FlushRegisters()
		{
			_mm256_zeroall();
		}

		union
		{
			struct
			{
				__m256d m_low;
				__m256d m_high;
			};
			struct
			{
				__m256i m_lowInt;
				__m256i m_highInt;
			};
			ndJacobian m_vector8;
			dInt64 m_ints[D_AVX_WORK_GROUP];
		};
	} D_GCC_NEWTON_ALIGN_32;

#else
	D_MSV_NEWTON_ALIGN_32
	class ndAvxFloat
	{
		public:
		inline ndAvxFloat()
		{
		}

		inline ndAvxFloat(const ndFloat32 val)
			:m_type(_mm256_set1_ps(val))
		{
		}

		inline ndAvxFloat(const ndInt32 val)
			:m_type(_mm256_castsi256_ps(_mm256_set1_epi32(val)))
		{
		}

		inline ndAvxFloat(const __m256 type)
			: m_type(type)
		{
		}

		inline ndAvxFloat(const ndAvxFloat& copy)
			: m_type(copy.m_type)
		{
		}

		inline ndAvxFloat(const ndVector& low, const ndVector& high)
			#ifdef D_SCALAR_VECTOR_CLASS
			:m_type(_mm256_set_m128(_mm_set_ps(low.m_w, low.m_y, low.m_z, low.m_x), _mm_set_ps(high.m_w, high.m_y, high.m_z, high.m_x)))
			#else
			:m_type(_mm256_set_m128(high.m_type, low.m_type))
			#endif
		{
		}

		inline ndAvxFloat(const ndAvxFloat* const baseAddr, const ndAvxFloat& index)
			: m_type(_mm256_i32gather_ps(&(*baseAddr)[0], index.m_typeInt, 4))
		{
		}

		inline ndFloat32& operator[] (ndInt32 i)
		{
			dAssert(i >= 0);
			dAssert(i < D_AVX_WORK_GROUP);
			ndFloat32* const ptr = (ndFloat32*)&m_type;
			return ptr[i];
		}

		inline const ndFloat32& operator[] (ndInt32 i) const
		{
			dAssert(i >= 0);
			dAssert(i < D_AVX_WORK_GROUP);
			const ndFloat32* const ptr = (ndFloat32*)&m_type;
			return ptr[i];
		}

		inline ndAvxFloat operator+ (const ndAvxFloat& A) const
		{
			return _mm256_add_ps(m_type, A.m_type);
		}

		inline ndAvxFloat operator- (const ndAvxFloat& A) const
		{
			return _mm256_sub_ps(m_type, A.m_type);
		}

		inline ndAvxFloat operator* (const ndAvxFloat& A) const
		{
			return _mm256_mul_ps(m_type, A.m_type);
		}

		inline ndAvxFloat MulAdd(const ndAvxFloat& A, const ndAvxFloat& B) const
		{
			return _mm256_fmadd_ps(A.m_type, B.m_type, m_type);
		}

		inline ndAvxFloat MulSub(const ndAvxFloat& A, const ndAvxFloat& B) const
		{
			return _mm256_fnmadd_ps(A.m_type, B.m_type, m_type);
		}

		inline ndAvxFloat operator> (const ndAvxFloat& A) const
		{
			return _mm256_cmp_ps(m_type, A.m_type, _CMP_GT_OQ);
		}

		inline ndAvxFloat operator< (const ndAvxFloat& A) const
		{
			return _mm256_cmp_ps(m_type, A.m_type, _CMP_LT_OQ);
		}

		inline ndAvxFloat operator| (const ndAvxFloat& A) const
		{
			return _mm256_or_ps(m_type, A.m_type);
		}

		inline ndAvxFloat operator& (const ndAvxFloat& A) const
		{
			return _mm256_and_ps(m_type, A.m_type);
		}

		inline ndAvxFloat GetMin(const ndAvxFloat& A) const
		{
			return _mm256_min_ps(m_type, A.m_type);
		}

		inline ndAvxFloat GetMax(const ndAvxFloat& A) const
		{
			return _mm256_max_ps(m_type, A.m_type);
		}

		inline ndAvxFloat Select(const ndAvxFloat& data, const ndAvxFloat& mask) const
		{
			// (((b ^ a) & mask)^a)
			//return  _mm_or_ps (_mm_and_ps (mask.m_type, data.m_type), _mm_andnot_ps(mask.m_type, m_type));
			return  _mm256_xor_ps(m_type, _mm256_and_ps(mask.m_type, _mm256_xor_ps(m_type, data.m_type)));
		}


		inline ndFloat32 AddHorizontal() const
		{
			__m256 tmp0(_mm256_add_ps(m_type, _mm256_permute2f128_ps(m_type, m_type, 1)));
			__m256 tmp1(_mm256_hadd_ps(tmp0, tmp0));
			__m256 tmp2(_mm256_hadd_ps(tmp1, tmp1));
			return *((ndFloat32*)&tmp2);
		}

		static inline void FlushRegisters()
		{
			_mm256_zeroall();
		}

		union
		{
			__m256 m_type;
			__m256i m_typeInt;
			ndJacobian m_vector8;
			ndInt32 m_ints[D_AVX_WORK_GROUP];
		};

		static ndAvxFloat m_one;
		static ndAvxFloat m_zero;
		static ndAvxFloat m_mask;
		static ndAvxFloat m_ordinals;
	} D_GCC_NEWTON_ALIGN_32;
#endif

ndAvxFloat ndAvxFloat::m_one(ndFloat32(1.0f));
ndAvxFloat ndAvxFloat::m_zero(ndFloat32 (0.0f));
ndAvxFloat ndAvxFloat::m_ordinals(ndVector(0, 1, 2, 3), ndVector(4, 5, 6, 7));
ndAvxFloat ndAvxFloat::m_mask(ndVector(-1, -1, -1, -1), ndVector(-1, -1, -1, -1));

D_MSV_NEWTON_ALIGN_32
class ndAvxVector3
{
	public:
	ndAvxFloat m_x;
	ndAvxFloat m_y;
	ndAvxFloat m_z;
} D_GCC_NEWTON_ALIGN_32;

D_MSV_NEWTON_ALIGN_32
class ndAvxVector6
{
	public:
	ndAvxVector3 m_linear;
	ndAvxVector3 m_angular;
} D_GCC_NEWTON_ALIGN_32;

D_MSV_NEWTON_ALIGN_32
class ndOpenclJacobianPair
{
	public:
	ndAvxVector6 m_jacobianM0;
	ndAvxVector6 m_jacobianM1;
}D_GCC_NEWTON_ALIGN_32;

D_MSV_NEWTON_ALIGN_32
class ndSoaMatrixElement
{
	public:
	ndOpenclJacobianPair m_Jt;
	ndOpenclJacobianPair m_JMinv;

	ndAvxFloat m_force;
	ndAvxFloat m_diagDamp;
	ndAvxFloat m_invJinvMJt;
	ndAvxFloat m_coordenateAccel;
	ndAvxFloat m_normalForceIndex;
	ndAvxFloat m_lowerBoundFrictionCoefficent;
	ndAvxFloat m_upperBoundFrictionCoefficent;
} D_GCC_NEWTON_ALIGN_32;

class dAvxMatrixArray : public ndArray<ndSoaMatrixElement>
{
};

ndDynamicsUpdateAvx2::ndDynamicsUpdateAvx2(ndWorld* const world)
	:ndDynamicsUpdate(world)
	,m_avxJointRows(D_AVX_DEFAULT_BUFFER_SIZE * 4)
	,m_avxMassMatrixArray(new dAvxMatrixArray)
{
}

ndDynamicsUpdateAvx2::~ndDynamicsUpdateAvx2()
{
	Clear();
	m_avxJointRows.Resize(D_AVX_DEFAULT_BUFFER_SIZE * 4);
	delete m_avxMassMatrixArray;
}

const char* ndDynamicsUpdateAvx2::GetStringId() const
{
	return "avx2";
}

void ndDynamicsUpdateAvx2::DetermineSleepStates()
{
	D_TRACKTIME();
	class ndDetermineSleepStates : public ndScene::ndBaseJob
	{
		public:
		void UpdateIslandState(const ndIsland& island)
		{
			ndWorld* const world = m_owner->GetWorld();
			ndFloat32 velocityDragCoeff = D_FREEZZING_VELOCITY_DRAG;

			const ndInt32 count = island.m_count;
			if (count <= D_SMALL_ISLAND_COUNT)
			{
				velocityDragCoeff = ndFloat32(0.9999f);
			}

			ndFloat32 maxAccel = ndFloat32(0.0f);
			ndFloat32 maxAlpha = ndFloat32(0.0f);
			ndFloat32 maxSpeed = ndFloat32(0.0f);
			ndFloat32 maxOmega = ndFloat32(0.0f);

			const ndFloat32 speedFreeze = world->m_freezeSpeed2;
			const ndFloat32 accelFreeze = world->m_freezeAccel2 * ((count <= D_SMALL_ISLAND_COUNT) ? ndFloat32(0.01f) : ndFloat32(1.0f));
			const ndFloat32 acc2 = D_SOLVER_MAX_ERROR * D_SOLVER_MAX_ERROR;
			const ndFloat32 maxAccNorm2 = (count > 4) ? acc2 : acc2 * ndFloat32(0.0625f);
			const ndVector velocDragVect(velocityDragCoeff, velocityDragCoeff, velocityDragCoeff, ndFloat32(0.0f));

			ndInt32 stackSleeping = 1;
			ndInt32 sleepCounter = 10000;

			ndDynamicsUpdate* const me = world->m_solver;
			ndBodyKinematic** const bodyIslands = &me->GetBodyIslandOrder()[island.m_start];
			for (ndInt32 i = 0; i < count; ++i)
			{
				ndBodyDynamic* const dynBody = bodyIslands[i]->GetAsBodyDynamic();
				if (dynBody)
				{
					dAssert(dynBody->m_accel.m_w == ndFloat32(0.0f));
					dAssert(dynBody->m_alpha.m_w == ndFloat32(0.0f));
					dAssert(dynBody->m_veloc.m_w == ndFloat32(0.0f));
					dAssert(dynBody->m_omega.m_w == ndFloat32(0.0f));

					ndVector accelTest((dynBody->m_accel.DotProduct(dynBody->m_accel) > maxAccNorm2) | (dynBody->m_alpha.DotProduct(dynBody->m_alpha) > maxAccNorm2));
					dynBody->m_accel = dynBody->m_accel & accelTest;
					dynBody->m_alpha = dynBody->m_alpha & accelTest;

					ndUnsigned8 equilibrium = dynBody->m_isStatic | dynBody->m_autoSleep;
					dAssert(equilibrium == ((dynBody->m_invMass.m_w == ndFloat32(0.0f)) ? 1 : dynBody->m_autoSleep));
					const ndVector isMovingMask(dynBody->m_veloc + dynBody->m_omega + dynBody->m_accel + dynBody->m_alpha);
					const ndVector mask(isMovingMask.TestZero());
					const ndInt32 test = mask.GetSignMask() & 7;
					if (test != 7)
					{
						const ndFloat32 accel2 = dynBody->m_accel.DotProduct(dynBody->m_accel).GetScalar();
						const ndFloat32 alpha2 = dynBody->m_alpha.DotProduct(dynBody->m_alpha).GetScalar();
						const ndFloat32 speed2 = dynBody->m_veloc.DotProduct(dynBody->m_veloc).GetScalar();
						const ndFloat32 omega2 = dynBody->m_omega.DotProduct(dynBody->m_omega).GetScalar();

						maxAccel = dMax(maxAccel, accel2);
						maxAlpha = dMax(maxAlpha, alpha2);
						maxSpeed = dMax(maxSpeed, speed2);
						maxOmega = dMax(maxOmega, omega2);
						ndUnsigned32 equilibriumTest = (accel2 < accelFreeze) && (alpha2 < accelFreeze) && (speed2 < speedFreeze) && (omega2 < speedFreeze);

						if (equilibriumTest)
						{
							const ndVector veloc(dynBody->m_veloc * velocDragVect);
							const ndVector omega(dynBody->m_omega * velocDragVect);
							const ndVector velocMask(veloc.DotProduct(veloc) > m_velocTol);
							const ndVector omegaMask(omega.DotProduct(omega) > m_velocTol);
							dynBody->m_veloc = velocMask & veloc;
							dynBody->m_omega = omegaMask & omega;
						}

						equilibrium &= equilibriumTest;
						stackSleeping &= equilibrium;
						sleepCounter = dMin(sleepCounter, dynBody->m_sleepingCounter);
						dynBody->m_sleepingCounter++;
					}
					if (dynBody->m_equilibrium != equilibrium)
					{
						dynBody->m_equilibrium = equilibrium;
					}
				}
				else
				{
					ndBodyKinematic* const kinBody = bodyIslands[i]->GetAsBodyKinematic();
					dAssert(kinBody);
					ndUnsigned8 equilibrium = (kinBody->m_invMass.m_w == ndFloat32(0.0f)) ? 1 : (kinBody->m_autoSleep & ~kinBody->m_equilibriumOverride);
					const ndVector isMovingMask(kinBody->m_veloc + kinBody->m_omega);
					const ndVector mask(isMovingMask.TestZero());
					const ndInt32 test = mask.GetSignMask() & 7;
					if (test != 7)
					{
						const ndFloat32 speed2 = kinBody->m_veloc.DotProduct(kinBody->m_veloc).GetScalar();
						const ndFloat32 omega2 = kinBody->m_omega.DotProduct(kinBody->m_omega).GetScalar();

						maxSpeed = dMax(maxSpeed, speed2);
						maxOmega = dMax(maxOmega, omega2);
						ndUnsigned32 equilibriumTest = (speed2 < speedFreeze) && (omega2 < speedFreeze);

						if (equilibriumTest)
						{
							const ndVector veloc(kinBody->m_veloc * velocDragVect);
							const ndVector omega(kinBody->m_omega * velocDragVect);
							const ndVector velocMask(veloc.DotProduct(veloc) > m_velocTol);
							const ndVector omegaMask(omega.DotProduct(omega) > m_velocTol);
							kinBody->m_veloc = velocMask & veloc;
							kinBody->m_omega = omegaMask & omega;
						}

						equilibrium &= equilibriumTest;
						stackSleeping &= equilibrium;
						sleepCounter = dMin(sleepCounter, kinBody->m_sleepingCounter);
					}
					if (kinBody->m_equilibrium != equilibrium)
					{
						kinBody->m_equilibrium = equilibrium;
					}
				}
			}

			if (stackSleeping)
			{
				for (ndInt32 i = 0; i < count; ++i)
				{
					// force entire island to equilibriumTest
					ndBodyDynamic* const body = bodyIslands[i]->GetAsBodyDynamic();
					if (body)
					{
						body->m_accel = ndVector::m_zero;
						body->m_alpha = ndVector::m_zero;
						body->m_veloc = ndVector::m_zero;
						body->m_omega = ndVector::m_zero;
						body->m_equilibrium = body->m_isStatic | body->m_autoSleep;
					}
					else
					{
						ndBodyKinematic* const kinBody = bodyIslands[i]->GetAsBodyKinematic();
						dAssert(kinBody);
						kinBody->m_veloc = ndVector::m_zero;
						kinBody->m_omega = ndVector::m_zero;
						kinBody->m_equilibrium = kinBody->m_isStatic | kinBody->m_autoSleep;
					}
				}
			}
			else if ((count > 1) || bodyIslands[0]->m_bodyIsConstrained)
			{
				const bool state =
					(maxAccel > world->m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxAccel) ||
					(maxAlpha > world->m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxAlpha) ||
					(maxSpeed > world->m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxVeloc) ||
					(maxOmega > world->m_sleepTable[D_SLEEP_ENTRIES - 1].m_maxOmega);

				if (state)
				{
					for (ndInt32 i = 0; i < count; ++i)
					{
						ndBodyDynamic* const body = bodyIslands[i]->GetAsBodyDynamic();
						if (body)
						{
							body->m_sleepingCounter = 0;
						}
					}
				}
				else
				{
					if (count < D_SMALL_ISLAND_COUNT)
					{
						// delay small islandArray for about 10 seconds
						sleepCounter >>= 8;
						for (ndInt32 i = 0; i < count; ++i)
						{
							ndBodyKinematic* const body = bodyIslands[i];
							body->m_equilibrium = 0;
						}
					}
					ndInt32 timeScaleSleepCount = ndInt32(ndFloat32(60.0f) * sleepCounter * m_timestep);

					ndInt32 sleepIndex = D_SLEEP_ENTRIES;
					for (ndInt32 i = 1; i < D_SLEEP_ENTRIES; ++i)
					{
						if (world->m_sleepTable[i].m_steps > timeScaleSleepCount)
						{
							sleepIndex = i;
							break;
						}
					}
					sleepIndex--;

					bool state1 =
						(maxAccel < world->m_sleepTable[sleepIndex].m_maxAccel) &&
						(maxAlpha < world->m_sleepTable[sleepIndex].m_maxAlpha) &&
						(maxSpeed < world->m_sleepTable[sleepIndex].m_maxVeloc) &&
						(maxOmega < world->m_sleepTable[sleepIndex].m_maxOmega);
					if (state1)
					{
						for (ndInt32 i = 0; i < count; ++i)
						{
							ndBodyKinematic* const body = bodyIslands[i];
							body->m_veloc = ndVector::m_zero;
							body->m_omega = ndVector::m_zero;
							body->m_equilibrium = body->m_autoSleep;
							ndBodyDynamic* const dynBody = body->GetAsBodyDynamic();
							if (dynBody)
							{
								dynBody->m_accel = ndVector::m_zero;
								dynBody->m_alpha = ndVector::m_zero;
								dynBody->m_sleepingCounter = 0;
							}
						}
					}
				}
			}
		}


		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdate* const me = world->m_solver;
			const ndArray<ndIsland>& islandArray = me->GetIslands();

			const ndInt32 threadIndex = GetThreadId();
			const ndInt32 threadCount = m_owner->GetThreadCount();
			const ndInt32 islandCount = islandArray.GetCount();

			m_zero = ndVector::m_zero;
			m_velocTol = me->GetVelocTol();
			for (ndInt32 i = threadIndex; i < islandCount; i += threadCount)
			{
				const ndIsland& island = islandArray[i];
				UpdateIslandState(island);
			}
		}

		ndVector m_zero;
		ndVector m_velocTol;
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndDetermineSleepStates>();
}

void ndDynamicsUpdateAvx2::SortJoints()
{
	D_TRACKTIME();
	class ndRowsCount
	{
		public:
		ndInt32 m_rowsCount;
		ndInt32 m_soaJointRowCount;
	};

	class ndSetRowStarts : public ndScene::ndBaseJob
	{
		public:
		void SetRowsCount()
		{
			ndConstraintArray& jointArray = m_owner->GetActiveContactArray();
			const ndInt32 count = jointArray.GetCount();

			ndInt32 rowCount = 1;
			for (ndInt32 i = 0; i < count; i++)
			{
				ndConstraint* const joint = jointArray[i];
				joint->m_rowStart = rowCount;
				rowCount += joint->m_rowCount;
			}
			ndRowsCount* const counters = (ndRowsCount*)m_context;
			counters->m_rowsCount = rowCount;
		}

		void SetSoaRowsCount()
		{
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateAvx2* const me = (ndDynamicsUpdateAvx2*)world->m_solver;
			ndConstraintArray& jointArray = m_owner->GetActiveContactArray();

			ndInt32 soaJointRowCount = 0;
			ndArray<ndInt32>& soaJointRows = me->m_avxJointRows;
			ndInt32 soaJointCountBatches = soaJointRows.GetCount();
			for (ndInt32 i = 0; i < soaJointCountBatches; i++)
			{
				const ndConstraint* const joint = jointArray[i * D_AVX_WORK_GROUP];
				soaJointRows[i] = soaJointRowCount;
				soaJointRowCount += joint->m_rowCount;
			}

			ndRowsCount* const counters = (ndRowsCount*)m_context;
			counters->m_soaJointRowCount = soaJointRowCount;
		}

		virtual void Execute()
		{
			D_TRACKTIME();
			const ndInt32 threadIndex = GetThreadId();
			const ndInt32 threadCount = m_owner->GetThreadCount();

			if (threadCount == 1)
			{
				SetRowsCount();
				SetSoaRowsCount();
			}
			else if (threadIndex == 0)
			{
				SetRowsCount();
			}
			else if (threadIndex == (threadCount - 1))
			{
				SetSoaRowsCount();
			}
		}
	};

	SortJointsScan();
	if (!m_activeJointCount)
	{
		return;
	}

	ndScene* const scene = m_world->GetScene();
	ndConstraintArray& jointArray = scene->GetActiveContactArray();

	#ifdef _DEBUG
		for (ndInt32 i = 1; i < m_activeJointCount; i++)
		{
			ndConstraint* const joint0 = jointArray[i - 1];
			ndConstraint* const joint1 = jointArray[i - 0];
			dAssert(!joint0->m_resting);
			dAssert(!joint1->m_resting);
			dAssert(joint0->m_rowCount >= joint1->m_rowCount);
			dAssert(!(joint0->GetBody0()->m_equilibrium0 & joint0->GetBody1()->m_equilibrium0));
			dAssert(!(joint1->GetBody0()->m_equilibrium0 & joint1->GetBody1()->m_equilibrium0));
		}

		for (ndInt32 i = m_activeJointCount + 1; i < jointArray.GetCount(); i++)
		{
			ndConstraint* const joint0 = jointArray[i - 1];
			ndConstraint* const joint1 = jointArray[i - 0];
			dAssert(joint0->m_resting);
			dAssert(joint1->m_resting);
			dAssert(joint0->m_rowCount >= joint1->m_rowCount);
			dAssert(joint0->GetBody0()->m_equilibrium0 & joint0->GetBody1()->m_equilibrium0);
			dAssert(joint1->GetBody0()->m_equilibrium0 & joint1->GetBody1()->m_equilibrium0);
		}
	#endif

	const ndInt32 mask = -ndInt32(D_AVX_WORK_GROUP);
	const ndInt32 jointCount = jointArray.GetCount();
	const ndInt32 soaJointCount = (jointCount + D_AVX_WORK_GROUP - 1) & mask;
	dAssert(jointArray.GetCapacity() > soaJointCount);
	ndConstraint** const jointArrayPtr = &jointArray[0];
	for (ndInt32 i = jointCount; i < soaJointCount; i++)
	{
		jointArrayPtr[i] = nullptr;
	}

	if (m_activeJointCount - jointArray.GetCount())
	{
		const ndInt32 base = m_activeJointCount & mask;
		const ndInt32 count = jointArrayPtr[base + D_AVX_WORK_GROUP - 1] ? D_AVX_WORK_GROUP : jointArray.GetCount() - base;
		dAssert(count <= D_AVX_WORK_GROUP);
		ndConstraint** const array = &jointArrayPtr[base];
		for (ndInt32 j = 1; j < count; j++)
		{
			ndInt32 slot = j;
			ndConstraint* const joint = array[slot];
			for (; (slot > 0) && (array[slot - 1]->m_rowCount < joint->m_rowCount); slot--)
			{
				array[slot] = array[slot - 1];
			}
			array[slot] = joint;
		}
	}

	ndRowsCount rowsCount;
	const ndInt32 soaJointCountBatches = soaJointCount / D_AVX_WORK_GROUP;
	m_avxJointRows.SetCount(soaJointCountBatches);
	scene->SubmitJobs<ndSetRowStarts>(&rowsCount);
	
	ndInt32 rowCount = 1;
	for (ndInt32 i = 0; i < jointArray.GetCount(); i++)
	{
		ndConstraint* const joint = jointArray[i];
		joint->m_rowStart = rowCount;
		rowCount += joint->m_rowCount;
	}
	
	m_leftHandSide.SetCount(rowsCount.m_rowsCount);
	m_rightHandSide.SetCount(rowsCount.m_rowsCount);
	m_avxMassMatrixArray->SetCount(rowsCount.m_soaJointRowCount);

	#ifdef _DEBUG
		dAssert(m_activeJointCount <= jointArray.GetCount());
		const ndInt32 maxRowCount = m_leftHandSide.GetCount();
		for (ndInt32 i = 0; i < jointArray.GetCount(); i++)
		{
			ndConstraint* const joint = jointArray[i];
			dAssert(joint->m_rowStart < m_leftHandSide.GetCount());
			dAssert((joint->m_rowStart + joint->m_rowCount) <= maxRowCount);
		}

		for (ndInt32 i = 0; i < jointCount; i += D_AVX_WORK_GROUP)
		{
			const ndInt32 count = jointArrayPtr[i + D_AVX_WORK_GROUP - 1] ? D_AVX_WORK_GROUP : jointCount - i;
			for (ndInt32 j = 1; j < count; j++)
			{
				ndConstraint* const joint0 = jointArrayPtr[i + j - 1];
				ndConstraint* const joint1 = jointArrayPtr[i + j - 0];
				dAssert(joint0->m_rowCount >= joint1->m_rowCount);
			}
		}
	#endif
	SortBodyJointScan();
}

void ndDynamicsUpdateAvx2::SortIslands()
{
	D_TRACKTIME();
	class ndIslandKey
	{
		public:
		ndUnsigned32 GetKey(const ndBodyIndexPair& pair) const
		{
			const ndBodyKinematic* const body = pair.m_root;
			const ndInt32 key = 1 - body->m_bodyIsConstrained;
			return key;
		}
	};

	class ndEvaluateKey
	{
		public:
		ndUnsigned32 GetKey(const ndIsland& island) const
		{
			ndUnsigned32 key = island.m_count * 2 + island.m_root->m_bodyIsConstrained;
			const ndUnsigned32 maxVal = 1 << (D_MAX_BODY_RADIX_BIT * 2);
			dAssert(key < maxVal);
			return maxVal - key;
		}
	};
	ndScene* const scene = m_world->GetScene();
	const ndArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	GetInternalForces().SetCount(bodyArray.GetCount());

	ndInt32 bodyCount = 0;
	const ndInt32 totalBodyCount = bodyArray.GetCount() - 1;
	ndBodyIndexPair* const buffer0 = (ndBodyIndexPair*)&GetInternalForces()[0];
	for (ndInt32 i = 0; i < totalBodyCount; i++)
	{
		ndBodyKinematic* const body = bodyArray[i];
		if (!(body->m_equilibrium0 & body->m_islandSleep) || body->GetAsBodyPlayerCapsule())
		{
			buffer0[bodyCount].m_body = body;
			//if (body->m_invMass.m_w > ndFloat32(0.0f))
			if (!body->m_isStatic)
			{
				ndBodyKinematic* root = body->m_islandParent;
				while (root != root->m_islandParent)
				{
					root = root->m_islandParent;
				}
		
				buffer0[bodyCount].m_root = root;
				if (root->m_rank != -1)
				{
					root->m_rank = -1;
				}
			}
			else
			{
				buffer0[bodyCount].m_root = body;
				body->m_rank = -1;
			}
			bodyCount++;
		}
	}

	ndArray<ndIsland>& islands = GetIslands();
	ndArray<ndBodyKinematic*>& activeBodyArray = GetBodyIslandOrder();

	islands.SetCount(0);
	activeBodyArray.SetCount(bodyCount);
	
	ndInt32 unConstrainedCount = 0;
	if (bodyCount)
	{
		scene->CountingSort<ndBodyIndexPair, 1, ndIslandKey>(buffer0, buffer0 + bodyCount, bodyCount, 0);
		for (ndInt32 i = 0; i < bodyCount; ++i)
		{
			dAssert((i == bodyCount - 1) || (buffer0[i].m_root->m_bodyIsConstrained >= buffer0[i + 1].m_root->m_bodyIsConstrained));

			activeBodyArray[i] = buffer0[i].m_body;
			if (buffer0[i].m_root->m_rank == -1)
			{
				buffer0[i].m_root->m_rank = 0;
				ndIsland island(buffer0[i].m_root);
				islands.PushBack(island);
			}
			buffer0[i].m_root->m_rank += 1;
		}
	
		ndInt32 start = 0;
		ndInt32 islandMaxKeySize = 0;
		for (ndInt32 i = 0; i < islands.GetCount(); ++i)
		{
			ndIsland& island = islands[i];
			island.m_start = start;
			island.m_count = island.m_root->m_rank;
			islandMaxKeySize = dMax(islandMaxKeySize, island.m_count);
			start += island.m_count;
			unConstrainedCount -= island.m_root->m_bodyIsConstrained;
		}
		unConstrainedCount += islands.GetCount();

		scene->CountingSort<ndIsland, D_MAX_BODY_RADIX_BIT, ndEvaluateKey>(&islands[0], (ndIsland*)GetTempBuffer(), islands.GetCount(), 0);
		if (islandMaxKeySize >= (1 << (D_MAX_BODY_RADIX_BIT - 1)))
		{
			scene->CountingSort<ndIsland, D_MAX_BODY_RADIX_BIT, ndEvaluateKey>(&islands[0], (ndIsland*)GetTempBuffer(), islands.GetCount(), 1);
		}
	}
	m_unConstrainedBodyCount = unConstrainedCount;
}

void ndDynamicsUpdateAvx2::BuildIsland()
{
	ndScene* const scene = m_world->GetScene();
	const ndArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	dAssert(bodyArray.GetCount() >= 1);
	if (bodyArray.GetCount() - 1)
	{
		D_TRACKTIME();
		SortJoints();
		SortIslands();
	}
}

void ndDynamicsUpdateAvx2::IntegrateUnconstrainedBodies()
{
	class ndIntegrateUnconstrainedBodies : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateSoa* const me = (ndDynamicsUpdateSoa*)world->m_solver;
			ndArray<ndBodyKinematic*>& bodyArray = me->GetBodyIslandOrder();

			const ndFloat32 timestep = m_timestep;
			const ndInt32 base = bodyArray.GetCount() - me->GetUnconstrainedBodyCount();

			const ndStartEnd startEnd(me->GetUnconstrainedBodyCount(), GetThreadId(), m_owner->GetThreadCount());
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				ndBodyKinematic* const body = bodyArray[base + i];
				dAssert(body);
				body->UpdateInvInertiaMatrix();
				body->AddDampingAcceleration(timestep);
				body->IntegrateExternalForce(timestep);
			}
		}
	};

	if (GetUnconstrainedBodyCount())
	{
		D_TRACKTIME();
		ndScene* const scene = m_world->GetScene();
		scene->SubmitJobs<ndIntegrateUnconstrainedBodies>();
	}
}

void ndDynamicsUpdateAvx2::IntegrateBodies()
{
	D_TRACKTIME();
	class ndIntegrateBodies : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateAvx2* const me = (ndDynamicsUpdateAvx2*)world->m_solver;
			ndArray<ndBodyKinematic*>& bodyArray = me->GetBodyIslandOrder();

			const ndVector invTime(me->m_invTimestep);
			const ndFloat32 timestep = m_timestep;

			const ndStartEnd startEnd(bodyArray.GetCount(), GetThreadId(), m_owner->GetThreadCount());
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				ndBodyKinematic* const body = bodyArray[i];
				if (!body->m_equilibrium)
				{
					body->m_accel = invTime * (body->m_veloc - body->m_accel);
					body->m_alpha = invTime * (body->m_omega - body->m_alpha);
					body->IntegrateVelocity(timestep);
				}
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndIntegrateBodies>();
}

void ndDynamicsUpdateAvx2::InitWeights()
{
	D_TRACKTIME();
	class ndInitWeights : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdate* const me = (ndDynamicsUpdate*)world->m_solver;

			ndBodyKinematic** const bodyArray = &m_owner->GetActiveBodyArray()[0];
			const ndArray<ndInt32>& jointForceIndexBuffer = me->GetJointForceIndexBuffer();
			const ndArray<ndJointBodyPairIndex>& jointBodyPairIndex = me->GetJointBodyPairIndexBuffer();

			ndInt32 maxExtraPasses = 1;

			const ndStartEnd startEnd(jointForceIndexBuffer.GetCount() - 2, GetThreadId(), m_owner->GetThreadCount());
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				const ndInt32 index = jointForceIndexBuffer[i];
				const ndJointBodyPairIndex& scan = jointBodyPairIndex[index];
				ndBodyKinematic* const body = bodyArray[scan.m_body];
				dAssert(body->m_index == scan.m_body);
				dAssert(body->m_bodyIsConstrained <= 1);
				const ndInt32 count = jointForceIndexBuffer[i + 1] - index - 1;
				const ndInt32 mask = -ndInt32(body->m_bodyIsConstrained & ~body->m_isStatic);
				const ndInt32 weigh = 1 + (mask & count);
				dAssert(weigh >= 0);
				if (weigh)
				{
					body->m_weigh = ndFloat32(weigh);
				}
				maxExtraPasses = dMax(weigh, maxExtraPasses);
			}
			ndInt32* const extraPasses = (ndInt32*)m_context;
			extraPasses[GetThreadId()] = maxExtraPasses;
		}
	};

	ndScene* const scene = m_world->GetScene();
	m_invTimestep = ndFloat32(1.0f) / m_timestep;
	m_invStepRK = ndFloat32(0.25f);
	m_timestepRK = m_timestep * m_invStepRK;
	m_invTimestepRK = m_invTimestep * ndFloat32(4.0f);

	const ndArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	const ndInt32 bodyCount = bodyArray.GetCount();
	GetInternalForces().SetCount(bodyCount);

	ndInt32 extraPassesArray[D_MAX_THREADS_COUNT];
	scene->SubmitJobs<ndInitWeights>(extraPassesArray);

	ndInt32 extraPasses = 0;
	const ndInt32 threadCount = scene->GetThreadCount();
	for (ndInt32 i = 0; i < threadCount; i++)
	{
		extraPasses = dMax(extraPasses, extraPassesArray[i]);
	}

	const ndInt32 conectivity = 7;
	m_solverPasses = m_world->GetSolverIterations() + 2 * extraPasses / conectivity + 1;
}

void ndDynamicsUpdateAvx2::InitBodyArray()
{
	D_TRACKTIME();
	class ndInitBodyArray : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdate* const me = world->m_solver;
			ndArray<ndBodyKinematic*>& bodyArray = me->GetBodyIslandOrder();

			const ndFloat32 timestep = m_timestep;

			const ndStartEnd startEnd(bodyArray.GetCount() - me->GetUnconstrainedBodyCount(), GetThreadId(), m_owner->GetThreadCount());
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				ndBodyKinematic* const body = bodyArray[i];
				dAssert(body);
				dAssert(body->m_bodyIsConstrained | body->m_isStatic);

				body->UpdateInvInertiaMatrix();
				body->AddDampingAcceleration(timestep);
				const ndVector angularMomentum(body->CalculateAngularMomentum());
				body->m_gyroTorque = body->m_omega.CrossProduct(angularMomentum);
				body->m_gyroAlpha = body->m_invWorldInertiaMatrix.RotateVector(body->m_gyroTorque);

				body->m_accel = body->m_veloc;
				body->m_alpha = body->m_omega;
				body->m_gyroRotation = body->m_rotation;
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndInitBodyArray>();
}

void ndDynamicsUpdateAvx2::GetJacobianDerivatives(ndConstraint* const joint)
{
	ndConstraintDescritor constraintParam;
	dAssert(joint->GetRowsCount() <= D_CONSTRAINT_MAX_ROWS);
	for (ndInt32 i = joint->GetRowsCount() - 1; i >= 0; i--)
	{
		constraintParam.m_forceBounds[i].m_low = D_MIN_BOUND;
		constraintParam.m_forceBounds[i].m_upper = D_MAX_BOUND;
		constraintParam.m_forceBounds[i].m_jointForce = nullptr;
		constraintParam.m_forceBounds[i].m_normalIndex = D_INDEPENDENT_ROW;
	}

	constraintParam.m_rowsCount = 0;
	constraintParam.m_timestep = m_timestep;
	constraintParam.m_invTimestep = m_invTimestep;
	joint->JacobianDerivative(constraintParam);
	const ndInt32 dof = constraintParam.m_rowsCount;
	dAssert(dof <= joint->m_rowCount);

	if (joint->GetAsContact())
	{
		ndContact* const contactJoint = joint->GetAsContact();
		contactJoint->m_isInSkeletonLoop = 0;
		ndSkeletonContainer* const skeleton0 = contactJoint->GetBody0()->GetSkeleton();
		ndSkeletonContainer* const skeleton1 = contactJoint->GetBody1()->GetSkeleton();
		if (skeleton0 && (skeleton0 == skeleton1))
		{
			if (contactJoint->IsSkeletonSelftCollision())
			{
				contactJoint->m_isInSkeletonLoop = 1;
				skeleton0->AddSelfCollisionJoint(contactJoint);
			}
		}
		else if (contactJoint->IsSkeletonIntraCollision())
		{
			if (skeleton0 && !skeleton1)
			{
				contactJoint->m_isInSkeletonLoop = 1;
				skeleton0->AddSelfCollisionJoint(contactJoint);
			}
			else if (skeleton1 && !skeleton0)
			{
				contactJoint->m_isInSkeletonLoop = 1;
				skeleton1->AddSelfCollisionJoint(contactJoint);
			}
		}
	}
	else
	{
		ndJointBilateralConstraint* const bilareral = joint->GetAsBilateral();
		dAssert(bilareral);
		if (!bilareral->m_isInSkeleton && (bilareral->GetSolverModel() == m_jointkinematicAttachment))
		{
			ndSkeletonContainer* const skeleton0 = bilareral->m_body0->GetSkeleton();
			ndSkeletonContainer* const skeleton1 = bilareral->m_body1->GetSkeleton();
			if (skeleton0 || skeleton1)
			{
				if (skeleton0 && !skeleton1)
				{
					bilareral->m_isInSkeletonLoop = 1;
					skeleton0->AddSelfCollisionJoint(bilareral);
				}
				else if (skeleton1 && !skeleton0)
				{
					bilareral->m_isInSkeletonLoop = 1;
					skeleton1->AddSelfCollisionJoint(bilareral);
				}
			}
		}
	}

	joint->m_rowCount = dof;
	const ndInt32 baseIndex = joint->m_rowStart;
	for (ndInt32 i = 0; i < dof; i++)
	{
		dAssert(constraintParam.m_forceBounds[i].m_jointForce);

		ndLeftHandSide* const row = &m_leftHandSide[baseIndex + i];
		ndRightHandSide* const rhs = &m_rightHandSide[baseIndex + i];

		row->m_Jt = constraintParam.m_jacobian[i];
		rhs->m_diagDamp = ndFloat32(0.0f);
		rhs->m_diagonalRegularizer = dMax(constraintParam.m_diagonalRegularizer[i], ndFloat32(1.0e-5f));

		rhs->m_coordenateAccel = constraintParam.m_jointAccel[i];
		rhs->m_restitution = constraintParam.m_restitution[i];
		rhs->m_penetration = constraintParam.m_penetration[i];
		rhs->m_penetrationStiffness = constraintParam.m_penetrationStiffness[i];
		rhs->m_lowerBoundFrictionCoefficent = constraintParam.m_forceBounds[i].m_low;
		rhs->m_upperBoundFrictionCoefficent = constraintParam.m_forceBounds[i].m_upper;
		rhs->m_jointFeebackForce = constraintParam.m_forceBounds[i].m_jointForce;

		dAssert(constraintParam.m_forceBounds[i].m_normalIndex >= -1);
		rhs->m_normalForceIndex = constraintParam.m_forceBounds[i].m_normalIndex;
	}
}

void ndDynamicsUpdateAvx2::InitJacobianMatrix()
{
	class ndInitJacobianMatrix : public ndScene::ndBaseJob
	{
		public:
		void BuildJacobianMatrix(ndConstraint* const joint, ndInt32 jointIndex)
		{
			dAssert(joint->GetBody0());
			dAssert(joint->GetBody1());
			const ndBodyKinematic* const body0 = joint->GetBody0();
			const ndBodyKinematic* const body1 = joint->GetBody1();

			ndAvxFloat force0(body0->GetForce(), body0->GetTorque());
			ndAvxFloat force1(body1->GetForce(), body1->GetTorque());

			const ndInt32 index = joint->m_rowStart;
			const ndInt32 count = joint->m_rowCount;

			const bool isBilateral = joint->IsBilateral();

			const ndMatrix& invInertia0 = body0->m_invWorldInertiaMatrix;
			const ndMatrix& invInertia1 = body1->m_invWorldInertiaMatrix;
			const ndVector invMass0(body0->m_invMass[3]);
			const ndVector invMass1(body1->m_invMass[3]);

			joint->m_preconditioner0 = ndFloat32(1.0f);
			joint->m_preconditioner1 = ndFloat32(1.0f);
			
			const bool test = !((body0->m_isStatic | body1->m_isStatic) || (body0->GetSkeleton() && body1->GetSkeleton()));
			dAssert(test == ((invMass0.GetScalar() > ndFloat32(0.0f)) && (invMass1.GetScalar() > ndFloat32(0.0f)) && !(body0->GetSkeleton() && body1->GetSkeleton())));
			if (test)
			{
				const ndFloat32 mass0 = body0->GetMassMatrix().m_w;
				const ndFloat32 mass1 = body1->GetMassMatrix().m_w;
				if (mass0 > (D_DIAGONAL_PRECONDITIONER * mass1))
				{
					joint->m_preconditioner0 = mass0 / (mass1 * D_DIAGONAL_PRECONDITIONER);
				}
				else if (mass1 > (D_DIAGONAL_PRECONDITIONER * mass0))
				{
					joint->m_preconditioner1 = mass1 / (mass0 * D_DIAGONAL_PRECONDITIONER);
				}
			}

			ndAvxFloat forceAcc0(ndAvxFloat::m_zero);
			ndAvxFloat forceAcc1(ndAvxFloat::m_zero);

			const ndAvxFloat weigh0(body0->m_weigh * joint->m_preconditioner0);
			const ndAvxFloat weigh1(body1->m_weigh * joint->m_preconditioner1);

			const ndFloat32 preconditioner0 = joint->m_preconditioner0;
			const ndFloat32 preconditioner1 = joint->m_preconditioner1;

			for (ndInt32 i = 0; i < count; i++)
			{
				ndLeftHandSide* const row = &m_leftHandSide[index + i];
				ndRightHandSide* const rhs = &m_rightHandSide[index + i];

				row->m_JMinv.m_jacobianM0.m_linear = row->m_Jt.m_jacobianM0.m_linear * invMass0;
				row->m_JMinv.m_jacobianM0.m_angular = invInertia0.RotateVector(row->m_Jt.m_jacobianM0.m_angular);
				row->m_JMinv.m_jacobianM1.m_linear = row->m_Jt.m_jacobianM1.m_linear * invMass1;
				row->m_JMinv.m_jacobianM1.m_angular = invInertia1.RotateVector(row->m_Jt.m_jacobianM1.m_angular);

				const ndAvxFloat& JMinvM0 = (ndAvxFloat&)row->m_JMinv.m_jacobianM0;
				const ndAvxFloat& JMinvM1 = (ndAvxFloat&)row->m_JMinv.m_jacobianM1;

				const ndAvxFloat tmpAccel((JMinvM0 * force0).MulAdd(JMinvM1, force1));

				ndFloat32 extenalAcceleration = -tmpAccel.AddHorizontal();
				rhs->m_deltaAccel = extenalAcceleration;
				rhs->m_coordenateAccel += extenalAcceleration;
				dAssert(rhs->m_jointFeebackForce);
				const ndFloat32 force = rhs->m_jointFeebackForce->GetInitialGuess();

				rhs->m_force = isBilateral ? dClamp(force, rhs->m_lowerBoundFrictionCoefficent, rhs->m_upperBoundFrictionCoefficent) : force;
				rhs->m_maxImpact = ndFloat32(0.0f);

				const ndAvxFloat& JtM0 = (ndAvxFloat&)row->m_Jt.m_jacobianM0;
				const ndAvxFloat& JtM1 = (ndAvxFloat&)row->m_Jt.m_jacobianM1;
				const ndAvxFloat tmpDiag(weigh0 * JMinvM0 * JtM0 + weigh1 * JMinvM1 * JtM1);

				ndFloat32 diag = tmpDiag.AddHorizontal();
				dAssert(diag > ndFloat32(0.0f));
				rhs->m_diagDamp = diag * rhs->m_diagonalRegularizer;

				diag *= (ndFloat32(1.0f) + rhs->m_diagonalRegularizer);
				rhs->m_invJinvMJt = ndFloat32(1.0f) / diag;

				forceAcc0 = forceAcc0.MulAdd(JtM0, ndAvxFloat (rhs->m_force * preconditioner0));
				forceAcc1 = forceAcc1.MulAdd(JtM1, ndAvxFloat (rhs->m_force * preconditioner1));
			}

			const ndInt32 index0 = jointIndex * 2 + 0;
			ndAvxFloat& outBody0 = m_internalForces[index0];
			outBody0 = forceAcc0;

			const ndInt32 index1 = jointIndex * 2 + 1;
			ndAvxFloat& outBody1 = m_internalForces[index1];
			outBody1 = forceAcc1;
		}

		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateAvx2* const me = (ndDynamicsUpdateAvx2*)world->m_solver;
			m_leftHandSide = &me->GetLeftHandSide()[0];
			m_rightHandSide = &me->GetRightHandSide()[0];
			m_internalForces = (ndAvxFloat*)&me->GetTempInternalForces()[0];
			m_jointBodyPairIndexBuffer = &me->GetJointBodyPairIndexBuffer()[0];
			ndConstraint** const jointArray = &m_owner->GetActiveContactArray()[0];

			const ndInt32 jointCount = m_owner->GetActiveContactArray().GetCount();
			const ndInt32 threadIndex = GetThreadId();
			const ndInt32 threadCount = m_owner->GetThreadCount();
			
			for (ndInt32 i = threadIndex; i < jointCount; i += threadCount)
			{
				ndConstraint* const joint = jointArray[i];
				me->GetJacobianDerivatives(joint);
				BuildJacobianMatrix(joint, i);
			}
		}

		ndAvxFloat* m_internalForces;
		ndLeftHandSide* m_leftHandSide;
		ndRightHandSide* m_rightHandSide;
		const ndJointBodyPairIndex* m_jointBodyPairIndexBuffer;
	};

	class ndTransposeMassMatrix : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateAvx2* const me = (ndDynamicsUpdateAvx2*)world->m_solver;
			ndConstraint** const jointArray = &m_owner->GetActiveContactArray()[0];
			const ndInt32 jointCount = m_owner->GetActiveContactArray().GetCount();

			const ndLeftHandSide* const leftHandSide = &me->GetLeftHandSide()[0];
			const ndRightHandSide* const rightHandSide = &me->GetRightHandSide()[0];
			dAvxMatrixArray& massMatrix = *me->m_avxMassMatrixArray;

			const ndAvxFloat zero(ndAvxFloat::m_zero);
			const ndAvxFloat ordinals(ndAvxFloat::m_ordinals);
			const ndInt32 mask = -ndInt32(D_AVX_WORK_GROUP);
			const ndInt32 soaJointCount = ((jointCount + D_AVX_WORK_GROUP - 1) & mask) / D_AVX_WORK_GROUP;
			const ndInt32* const soaJointRows = &me->m_avxJointRows[0];

			const ndInt32 threadIndex = GetThreadId();
			const ndInt32 threadCount = m_owner->GetThreadCount();
			for (ndInt32 i = threadIndex; i < soaJointCount; i += threadCount)
			{
				const ndInt32 index = i * D_AVX_WORK_GROUP;
				for (ndInt32 j = 1; j < D_AVX_WORK_GROUP; j++)
				{
					ndConstraint* const joint = jointArray[index + j];
					if (joint)
					{
						ndInt32 slot = j;
						for (; (slot > 0) && (jointArray[index + slot - 1]->m_rowCount < joint->m_rowCount); slot--)
						{
							jointArray[index + slot] = jointArray[index + slot - 1];
						}
						jointArray[index + slot] = joint;
					}
				}

				const ndInt32 soaRowBase = soaJointRows[i];
				const ndConstraint* const lastJoint = jointArray[index + D_AVX_WORK_GROUP - 1];
				if (lastJoint && (lastJoint->m_rowCount == jointArray[index]->m_rowCount))
				{
					const ndConstraint* const joint0 = jointArray[index + 0];
					const ndConstraint* const joint1 = jointArray[index + 1];
					const ndConstraint* const joint2 = jointArray[index + 2];
					const ndConstraint* const joint3 = jointArray[index + 3];
					const ndConstraint* const joint4 = jointArray[index + 4];
					const ndConstraint* const joint5 = jointArray[index + 5];
					const ndConstraint* const joint6 = jointArray[index + 6];
					const ndConstraint* const joint7 = jointArray[index + 7];
					
					const ndConstraint* const joint = jointArray[index];
					const ndInt32 rowCount = joint->m_rowCount;
					
					for (ndInt32 j = 0; j < rowCount; j++)
					{
						ndVector tmp[D_AVX_WORK_GROUP];
						const ndLeftHandSide* const row0 = &leftHandSide[joint0->m_rowStart + j];
						const ndLeftHandSide* const row1 = &leftHandSide[joint1->m_rowStart + j];
						const ndLeftHandSide* const row2 = &leftHandSide[joint2->m_rowStart + j];
						const ndLeftHandSide* const row3 = &leftHandSide[joint3->m_rowStart + j];
						const ndLeftHandSide* const row4 = &leftHandSide[joint4->m_rowStart + j];
						const ndLeftHandSide* const row5 = &leftHandSide[joint5->m_rowStart + j];
						const ndLeftHandSide* const row6 = &leftHandSide[joint6->m_rowStart + j];
						const ndLeftHandSide* const row7 = &leftHandSide[joint7->m_rowStart + j];
						ndSoaMatrixElement& row = massMatrix[soaRowBase + j];
					
						ndVector::Transpose4x4(tmp[0], tmp[1], tmp[2], tmp[3],
							row0->m_Jt.m_jacobianM0.m_linear,
							row1->m_Jt.m_jacobianM0.m_linear,
							row2->m_Jt.m_jacobianM0.m_linear,
							row3->m_Jt.m_jacobianM0.m_linear);
						ndVector::Transpose4x4(tmp[4], tmp[5], tmp[6], tmp[7],
							row4->m_Jt.m_jacobianM0.m_linear,
							row5->m_Jt.m_jacobianM0.m_linear,
							row6->m_Jt.m_jacobianM0.m_linear,
							row7->m_Jt.m_jacobianM0.m_linear);
						row.m_Jt.m_jacobianM0.m_linear.m_x = ndAvxFloat(tmp[0], tmp[4]);
						row.m_Jt.m_jacobianM0.m_linear.m_y = ndAvxFloat(tmp[1], tmp[5]);
						row.m_Jt.m_jacobianM0.m_linear.m_z = ndAvxFloat(tmp[2], tmp[6]);
						ndVector::Transpose4x4(tmp[0], tmp[1], tmp[2], tmp[3],
							row0->m_Jt.m_jacobianM0.m_angular,
							row1->m_Jt.m_jacobianM0.m_angular,
							row2->m_Jt.m_jacobianM0.m_angular,
							row3->m_Jt.m_jacobianM0.m_angular);
						ndVector::Transpose4x4(tmp[4], tmp[5], tmp[6], tmp[7],
							row4->m_Jt.m_jacobianM0.m_angular,
							row5->m_Jt.m_jacobianM0.m_angular,
							row6->m_Jt.m_jacobianM0.m_angular,
							row7->m_Jt.m_jacobianM0.m_angular);
						row.m_Jt.m_jacobianM0.m_angular.m_x = ndAvxFloat(tmp[0], tmp[4]);
						row.m_Jt.m_jacobianM0.m_angular.m_y = ndAvxFloat(tmp[1], tmp[5]);
						row.m_Jt.m_jacobianM0.m_angular.m_z = ndAvxFloat(tmp[2], tmp[6]);
					
						ndVector::Transpose4x4(tmp[0], tmp[1], tmp[2], tmp[3],
							row0->m_Jt.m_jacobianM1.m_linear,
							row1->m_Jt.m_jacobianM1.m_linear,
							row2->m_Jt.m_jacobianM1.m_linear,
							row3->m_Jt.m_jacobianM1.m_linear);
						ndVector::Transpose4x4(tmp[4], tmp[5], tmp[6], tmp[7],
							row4->m_Jt.m_jacobianM1.m_linear,
							row5->m_Jt.m_jacobianM1.m_linear,
							row6->m_Jt.m_jacobianM1.m_linear,
							row7->m_Jt.m_jacobianM1.m_linear);
						row.m_Jt.m_jacobianM1.m_linear.m_x = ndAvxFloat(tmp[0], tmp[4]);
						row.m_Jt.m_jacobianM1.m_linear.m_y = ndAvxFloat(tmp[1], tmp[5]);
						row.m_Jt.m_jacobianM1.m_linear.m_z = ndAvxFloat(tmp[2], tmp[6]);
						ndVector::Transpose4x4(tmp[0], tmp[1], tmp[2], tmp[3],
							row0->m_Jt.m_jacobianM1.m_angular,
							row1->m_Jt.m_jacobianM1.m_angular,
							row2->m_Jt.m_jacobianM1.m_angular,
							row3->m_Jt.m_jacobianM1.m_angular);
						ndVector::Transpose4x4(tmp[4], tmp[5], tmp[6], tmp[7],
							row4->m_Jt.m_jacobianM1.m_angular,
							row5->m_Jt.m_jacobianM1.m_angular,
							row6->m_Jt.m_jacobianM1.m_angular,
							row7->m_Jt.m_jacobianM1.m_angular);
						row.m_Jt.m_jacobianM1.m_angular.m_x = ndAvxFloat(tmp[0], tmp[4]);
						row.m_Jt.m_jacobianM1.m_angular.m_y = ndAvxFloat(tmp[1], tmp[5]);
						row.m_Jt.m_jacobianM1.m_angular.m_z = ndAvxFloat(tmp[2], tmp[6]);
					
						ndVector::Transpose4x4(tmp[0], tmp[1], tmp[2], tmp[3],
							row0->m_JMinv.m_jacobianM0.m_linear,
							row1->m_JMinv.m_jacobianM0.m_linear,
							row2->m_JMinv.m_jacobianM0.m_linear,
							row3->m_JMinv.m_jacobianM0.m_linear);
						ndVector::Transpose4x4(tmp[4], tmp[5], tmp[6], tmp[7],
							row4->m_JMinv.m_jacobianM0.m_linear,
							row5->m_JMinv.m_jacobianM0.m_linear,
							row6->m_JMinv.m_jacobianM0.m_linear,
							row7->m_JMinv.m_jacobianM0.m_linear);
						row.m_JMinv.m_jacobianM0.m_linear.m_x = ndAvxFloat(tmp[0], tmp[4]);
						row.m_JMinv.m_jacobianM0.m_linear.m_y = ndAvxFloat(tmp[1], tmp[5]);
						row.m_JMinv.m_jacobianM0.m_linear.m_z = ndAvxFloat(tmp[2], tmp[6]);
						ndVector::Transpose4x4(tmp[0], tmp[1], tmp[2], tmp[3],
							row0->m_JMinv.m_jacobianM0.m_angular,
							row1->m_JMinv.m_jacobianM0.m_angular,
							row2->m_JMinv.m_jacobianM0.m_angular,
							row3->m_JMinv.m_jacobianM0.m_angular);
						ndVector::Transpose4x4(tmp[4], tmp[5], tmp[6], tmp[7],
							row4->m_JMinv.m_jacobianM0.m_angular,
							row5->m_JMinv.m_jacobianM0.m_angular,
							row6->m_JMinv.m_jacobianM0.m_angular,
							row7->m_JMinv.m_jacobianM0.m_angular);
						row.m_JMinv.m_jacobianM0.m_angular.m_x = ndAvxFloat(tmp[0], tmp[4]);
						row.m_JMinv.m_jacobianM0.m_angular.m_y = ndAvxFloat(tmp[1], tmp[5]);
						row.m_JMinv.m_jacobianM0.m_angular.m_z = ndAvxFloat(tmp[2], tmp[6]);
					
						ndVector::Transpose4x4(tmp[0], tmp[1], tmp[2], tmp[3],
							row0->m_JMinv.m_jacobianM1.m_linear,
							row1->m_JMinv.m_jacobianM1.m_linear,
							row2->m_JMinv.m_jacobianM1.m_linear,
							row3->m_JMinv.m_jacobianM1.m_linear);
						ndVector::Transpose4x4(tmp[4], tmp[5], tmp[6], tmp[7],
							row4->m_JMinv.m_jacobianM1.m_linear,
							row5->m_JMinv.m_jacobianM1.m_linear,
							row6->m_JMinv.m_jacobianM1.m_linear,
							row7->m_JMinv.m_jacobianM1.m_linear);
						row.m_JMinv.m_jacobianM1.m_linear.m_x = ndAvxFloat(tmp[0], tmp[4]);
						row.m_JMinv.m_jacobianM1.m_linear.m_y = ndAvxFloat(tmp[1], tmp[5]);
						row.m_JMinv.m_jacobianM1.m_linear.m_z = ndAvxFloat(tmp[2], tmp[6]);
						ndVector::Transpose4x4(tmp[0], tmp[1], tmp[2], tmp[3],
							row0->m_JMinv.m_jacobianM1.m_angular,
							row1->m_JMinv.m_jacobianM1.m_angular,
							row2->m_JMinv.m_jacobianM1.m_angular,
							row3->m_JMinv.m_jacobianM1.m_angular);
						ndVector::Transpose4x4(tmp[4], tmp[5], tmp[6], tmp[7],
							row4->m_JMinv.m_jacobianM1.m_angular,
							row5->m_JMinv.m_jacobianM1.m_angular,
							row6->m_JMinv.m_jacobianM1.m_angular,
							row7->m_JMinv.m_jacobianM1.m_angular);
						row.m_JMinv.m_jacobianM1.m_angular.m_x = ndAvxFloat(tmp[0], tmp[4]);
						row.m_JMinv.m_jacobianM1.m_angular.m_y = ndAvxFloat(tmp[1], tmp[5]);
						row.m_JMinv.m_jacobianM1.m_angular.m_z = ndAvxFloat(tmp[2], tmp[6]);
					
						#ifdef D_NEWTON_USE_DOUBLE
							dInt64* const normalIndex = (dInt64*)&row.m_normalForceIndex[0];
						#else
							ndInt32* const normalIndex = (ndInt32*)&row.m_normalForceIndex[0];
						#endif
						for (ndInt32 k = 0; k < D_AVX_WORK_GROUP; k++)
						{
							const ndConstraint* const soaJoint = jointArray[index + k];
							const ndRightHandSide* const rhs = &rightHandSide[soaJoint->m_rowStart + j];
							row.m_force[k] = rhs->m_force;
							row.m_diagDamp[k] = rhs->m_diagDamp;
							row.m_invJinvMJt[k] = rhs->m_invJinvMJt;
							row.m_coordenateAccel[k] = rhs->m_coordenateAccel;
							normalIndex[k] = (rhs->m_normalForceIndex + 1) * D_AVX_WORK_GROUP + k;
							row.m_lowerBoundFrictionCoefficent[k] = rhs->m_lowerBoundFrictionCoefficent;
							row.m_upperBoundFrictionCoefficent[k] = rhs->m_upperBoundFrictionCoefficent;
						}
					}
				}
				else
				{
					const ndConstraint* const firstJoint = jointArray[index];
					for (ndInt32 j = 0; j < firstJoint->m_rowCount; j++)
					{
						ndSoaMatrixElement& row = massMatrix[soaRowBase + j];
						row.m_Jt.m_jacobianM0.m_linear.m_x = zero;
						row.m_Jt.m_jacobianM0.m_linear.m_y = zero;
						row.m_Jt.m_jacobianM0.m_linear.m_z = zero;
						row.m_Jt.m_jacobianM0.m_angular.m_x = zero;
						row.m_Jt.m_jacobianM0.m_angular.m_y = zero;
						row.m_Jt.m_jacobianM0.m_angular.m_z = zero;
						row.m_Jt.m_jacobianM1.m_linear.m_x = zero;
						row.m_Jt.m_jacobianM1.m_linear.m_y = zero;
						row.m_Jt.m_jacobianM1.m_linear.m_z = zero;
						row.m_Jt.m_jacobianM1.m_angular.m_x = zero;
						row.m_Jt.m_jacobianM1.m_angular.m_y = zero;
						row.m_Jt.m_jacobianM1.m_angular.m_z = zero;
					
						row.m_JMinv.m_jacobianM0.m_linear.m_x = zero;
						row.m_JMinv.m_jacobianM0.m_linear.m_y = zero;
						row.m_JMinv.m_jacobianM0.m_linear.m_z = zero;
						row.m_JMinv.m_jacobianM0.m_angular.m_x = zero;
						row.m_JMinv.m_jacobianM0.m_angular.m_y = zero;
						row.m_JMinv.m_jacobianM0.m_angular.m_z = zero;
						row.m_JMinv.m_jacobianM1.m_linear.m_x = zero;
						row.m_JMinv.m_jacobianM1.m_linear.m_y = zero;
						row.m_JMinv.m_jacobianM1.m_linear.m_z = zero;
						row.m_JMinv.m_jacobianM1.m_angular.m_x = zero;
						row.m_JMinv.m_jacobianM1.m_angular.m_y = zero;
						row.m_JMinv.m_jacobianM1.m_angular.m_z = zero;
					
						row.m_force = zero;
						row.m_diagDamp = zero;
						row.m_invJinvMJt = zero;
						row.m_coordenateAccel = zero;
						row.m_normalForceIndex = ordinals;
						row.m_lowerBoundFrictionCoefficent = zero;
						row.m_upperBoundFrictionCoefficent = zero;
					}
					
					for (ndInt32 j = 0; j < D_AVX_WORK_GROUP; j++)
					{
						const ndConstraint* const joint = jointArray[index + j];
						if (joint)
						{
							for (ndInt32 k = 0; k < joint->m_rowCount; k++)
							{
								ndSoaMatrixElement& row = massMatrix[soaRowBase + k];
								const ndLeftHandSide* const lhs = &leftHandSide[joint->m_rowStart + k];
					
								row.m_Jt.m_jacobianM0.m_linear.m_x[j] = lhs->m_Jt.m_jacobianM0.m_linear.m_x;
								row.m_Jt.m_jacobianM0.m_linear.m_y[j] = lhs->m_Jt.m_jacobianM0.m_linear.m_y;
								row.m_Jt.m_jacobianM0.m_linear.m_z[j] = lhs->m_Jt.m_jacobianM0.m_linear.m_z;
								row.m_Jt.m_jacobianM0.m_angular.m_x[j] = lhs->m_Jt.m_jacobianM0.m_angular.m_x;
								row.m_Jt.m_jacobianM0.m_angular.m_y[j] = lhs->m_Jt.m_jacobianM0.m_angular.m_y;
								row.m_Jt.m_jacobianM0.m_angular.m_z[j] = lhs->m_Jt.m_jacobianM0.m_angular.m_z;
								row.m_Jt.m_jacobianM1.m_linear.m_x[j] = lhs->m_Jt.m_jacobianM1.m_linear.m_x;
								row.m_Jt.m_jacobianM1.m_linear.m_y[j] = lhs->m_Jt.m_jacobianM1.m_linear.m_y;
								row.m_Jt.m_jacobianM1.m_linear.m_z[j] = lhs->m_Jt.m_jacobianM1.m_linear.m_z;
								row.m_Jt.m_jacobianM1.m_angular.m_x[j] = lhs->m_Jt.m_jacobianM1.m_angular.m_x;
								row.m_Jt.m_jacobianM1.m_angular.m_y[j] = lhs->m_Jt.m_jacobianM1.m_angular.m_y;
								row.m_Jt.m_jacobianM1.m_angular.m_z[j] = lhs->m_Jt.m_jacobianM1.m_angular.m_z;
					
								row.m_JMinv.m_jacobianM0.m_linear.m_x[j] = lhs->m_JMinv.m_jacobianM0.m_linear.m_x;
								row.m_JMinv.m_jacobianM0.m_linear.m_y[j] = lhs->m_JMinv.m_jacobianM0.m_linear.m_y;
								row.m_JMinv.m_jacobianM0.m_linear.m_z[j] = lhs->m_JMinv.m_jacobianM0.m_linear.m_z;
								row.m_JMinv.m_jacobianM0.m_angular.m_x[j] = lhs->m_JMinv.m_jacobianM0.m_angular.m_x;
								row.m_JMinv.m_jacobianM0.m_angular.m_y[j] = lhs->m_JMinv.m_jacobianM0.m_angular.m_y;
								row.m_JMinv.m_jacobianM0.m_angular.m_z[j] = lhs->m_JMinv.m_jacobianM0.m_angular.m_z;
								row.m_JMinv.m_jacobianM1.m_linear.m_x[j] = lhs->m_JMinv.m_jacobianM1.m_linear.m_x;
								row.m_JMinv.m_jacobianM1.m_linear.m_y[j] = lhs->m_JMinv.m_jacobianM1.m_linear.m_y;
								row.m_JMinv.m_jacobianM1.m_linear.m_z[j] = lhs->m_JMinv.m_jacobianM1.m_linear.m_z;
								row.m_JMinv.m_jacobianM1.m_angular.m_x[j] = lhs->m_JMinv.m_jacobianM1.m_angular.m_x;
								row.m_JMinv.m_jacobianM1.m_angular.m_y[j] = lhs->m_JMinv.m_jacobianM1.m_angular.m_y;
								row.m_JMinv.m_jacobianM1.m_angular.m_z[j] = lhs->m_JMinv.m_jacobianM1.m_angular.m_z;
					
								const ndRightHandSide* const rhs = &rightHandSide[joint->m_rowStart + k];
								row.m_force[j] = rhs->m_force;
								row.m_diagDamp[j] = rhs->m_diagDamp;
								row.m_invJinvMJt[j] = rhs->m_invJinvMJt;
								row.m_coordenateAccel[j] = rhs->m_coordenateAccel;
			
								#ifdef D_NEWTON_USE_DOUBLE
									dInt64* const normalIndex = (dInt64*)&row.m_normalForceIndex[0];
								#else
									ndInt32* const normalIndex = (ndInt32*)&row.m_normalForceIndex[0];
								#endif
								normalIndex[j] = (rhs->m_normalForceIndex + 1) * D_AVX_WORK_GROUP + j;
								row.m_lowerBoundFrictionCoefficent[j] = rhs->m_lowerBoundFrictionCoefficent;
								row.m_upperBoundFrictionCoefficent[j] = rhs->m_upperBoundFrictionCoefficent;
							}
						}
					}
				}
			}
		}
	};

	class ndInitJacobianAccumulatePartialForces : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			const ndAvxFloat zero(ndAvxFloat::m_zero);
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdate* const me = (ndDynamicsUpdate*)world->m_solver;

			const ndArray<ndInt32>& bodyIndex = me->GetJointForceIndexBuffer();
			ndBodyKinematic** const bodyArray = &m_owner->GetActiveBodyArray()[0];
			ndAvxFloat* const internalForces = (ndAvxFloat*)&me->GetInternalForces()[0];

			const ndAvxFloat* const jointInternalForces = (ndAvxFloat*)&me->GetTempInternalForces()[0];
			const ndJointBodyPairIndex* const jointBodyPairIndexBuffer = &me->GetJointBodyPairIndexBuffer()[0];

			const ndStartEnd startEnd(bodyIndex.GetCount() - 2, GetThreadId(), m_owner->GetThreadCount());
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				ndAvxFloat force(ndAvxFloat::m_zero);

				const ndInt32 index = bodyIndex[i];
				const ndJointBodyPairIndex& scan = jointBodyPairIndexBuffer[index];
				ndBodyKinematic* const body = bodyArray[scan.m_body];

				dAssert(body->m_isStatic <= 1);
				dAssert(body->m_index == scan.m_body);
				const ndInt32 mask = ndInt32(body->m_isStatic) - 1;
				const ndInt32 count = mask & (bodyIndex[i + 1] - index);

				for (ndInt32 j = 0; j < count; ++j)
				{
					const ndInt32 jointIndex = jointBodyPairIndexBuffer[index + j].m_joint;
					force = force + jointInternalForces[jointIndex];
				}
				internalForces[i] = force;
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	if (scene->GetActiveContactArray().GetCount())
	{
		D_TRACKTIME();
		m_rightHandSide[0].m_force = ndFloat32(1.0f);
		scene->SubmitJobs<ndInitJacobianMatrix>();
		scene->SubmitJobs<ndInitJacobianAccumulatePartialForces>();
		scene->SubmitJobs<ndTransposeMassMatrix>();
	}
}

void ndDynamicsUpdateAvx2::UpdateForceFeedback()
{
	D_TRACKTIME();
	class ndUpdateForceFeedback : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateAvx2* const me = (ndDynamicsUpdateAvx2*)world->m_solver;
			const ndConstraintArray& jointArray = m_owner->GetActiveContactArray();
			ndArray<ndRightHandSide>& rightHandSide = me->m_rightHandSide;

			const ndInt32 threadIndex = GetThreadId();
			const ndInt32 threadCount = m_owner->GetThreadCount();
			const ndInt32 jointCount = jointArray.GetCount();

			const ndFloat32 timestepRK = me->m_timestepRK;
			for (ndInt32 i = threadIndex; i < jointCount; i += threadCount)
			{
				ndConstraint* const joint = jointArray[i];
				const ndInt32 rows = joint->m_rowCount;
				const ndInt32 first = joint->m_rowStart;

				for (ndInt32 j = 0; j < rows; j++)
				{
					const ndRightHandSide* const rhs = &rightHandSide[j + first];
					dAssert(dCheckFloat(rhs->m_force));
					rhs->m_jointFeebackForce->Push(rhs->m_force);
					rhs->m_jointFeebackForce->m_force = rhs->m_force;
					rhs->m_jointFeebackForce->m_impact = rhs->m_maxImpact * timestepRK;
				}

				if (joint->GetAsBilateral())
				{
					const ndArray<ndLeftHandSide>& leftHandSide = me->m_leftHandSide;
					ndVector force0(ndVector::m_zero);
					ndVector force1(ndVector::m_zero);
					ndVector torque0(ndVector::m_zero);
					ndVector torque1(ndVector::m_zero);
					for (ndInt32 j = 0; j < rows; j++)
					{
						const ndRightHandSide* const rhs = &rightHandSide[j + first];
						const ndLeftHandSide* const lhs = &leftHandSide[j + first];
						const ndVector f(rhs->m_force);
						force0 += lhs->m_Jt.m_jacobianM0.m_linear * f;
						torque0 += lhs->m_Jt.m_jacobianM0.m_angular * f;
						force1 += lhs->m_Jt.m_jacobianM1.m_linear * f;
						torque1 += lhs->m_Jt.m_jacobianM1.m_angular * f;
					}
					ndJointBilateralConstraint* const bilateral = joint->GetAsBilateral();
					bilateral->m_forceBody0 = force0;
					bilateral->m_torqueBody0 = torque0;
					bilateral->m_forceBody1 = force1;
					bilateral->m_torqueBody1 = torque1;
				}
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndUpdateForceFeedback>();
}

void ndDynamicsUpdateAvx2::InitSkeletons()
{
	D_TRACKTIME();

	class ndInitSkeletons : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			const ndInt32 threadIndex = GetThreadId();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateAvx2* const me = (ndDynamicsUpdateAvx2*)world->m_solver;
			ndSkeletonList::ndNode* node = world->GetSkeletonList().GetFirst();
			for (ndInt32 i = 0; i < threadIndex; i++)
			{
				node = node ? node->GetNext() : nullptr;
			}

			ndArray<ndRightHandSide>& rightHandSide = me->m_rightHandSide;
			const ndArray<ndLeftHandSide>& leftHandSide = me->m_leftHandSide;

			const ndInt32 threadCount = m_owner->GetThreadCount();
			while (node)
			{
				ndSkeletonContainer* const skeleton = &node->GetInfo();
				skeleton->InitMassMatrix(&leftHandSide[0], &rightHandSide[0]);

				for (ndInt32 i = 0; i < threadCount; i++)
				{
					node = node ? node->GetNext() : nullptr;
				}
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndInitSkeletons>();
}

void ndDynamicsUpdateAvx2::UpdateSkeletons()
{
	D_TRACKTIME();
	class ndUpdateSkeletons : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			const ndInt32 threadIndex = GetThreadId();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateAvx2* const me = (ndDynamicsUpdateAvx2*)world->m_solver;
			ndSkeletonList::ndNode* node = world->GetSkeletonList().GetFirst();
			for (ndInt32 i = 0; i < threadIndex; i++)
			{
				node = node ? node->GetNext() : nullptr;
			}

			ndJacobian* const internalForces = &me->GetInternalForces()[0];
			const ndArray<ndBodyKinematic*>& activeBodies = m_owner->ndScene::GetActiveBodyArray();
			const ndBodyKinematic** const bodyArray = (const ndBodyKinematic**)&activeBodies[0];

			const ndInt32 threadCount = m_owner->GetThreadCount();
			while (node)
			{
				ndSkeletonContainer* const skeleton = &node->GetInfo();
				skeleton->CalculateJointForce(bodyArray, internalForces);

				for (ndInt32 i = 0; i < threadCount; i++)
				{
					node = node ? node->GetNext() : nullptr;
				}
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndUpdateSkeletons>();
}

void ndDynamicsUpdateAvx2::CalculateJointsAcceleration()
{
	D_TRACKTIME();
	class ndCalculateJointsAcceleration : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateAvx2* const me = (ndDynamicsUpdateAvx2*)world->m_solver;
			const ndConstraintArray& jointArray = m_owner->GetActiveContactArray();

			ndJointAccelerationDecriptor joindDesc;
			joindDesc.m_timestep = me->m_timestepRK;
			joindDesc.m_invTimestep = me->m_invTimestepRK;
			joindDesc.m_firstPassCoefFlag = me->m_firstPassCoef;
			ndArray<ndLeftHandSide>& leftHandSide = me->m_leftHandSide;
			ndArray<ndRightHandSide>& rightHandSide = me->m_rightHandSide;

			const ndInt32 threadIndex = GetThreadId();
			const ndInt32 threadCount = m_owner->GetThreadCount();
			const ndInt32 jointCount = jointArray.GetCount();

			for (ndInt32 i = threadIndex; i < jointCount; i += threadCount)
			{
				ndConstraint* const joint = jointArray[i];
				const ndInt32 pairStart = joint->m_rowStart;
				joindDesc.m_rowsCount = joint->m_rowCount;
				joindDesc.m_leftHandSide = &leftHandSide[pairStart];
				joindDesc.m_rightHandSide = &rightHandSide[pairStart];
				joint->JointAccelerations(&joindDesc);
			}
		}
	};

	class ndUpdateAcceleration : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateAvx2* const me = (ndDynamicsUpdateAvx2*)world->m_solver;
			const ndConstraintArray& jointArray = m_owner->GetActiveContactArray();
			const ndArray<ndRightHandSide>& rightHandSide = me->m_rightHandSide;

			const ndInt32 threadIndex = GetThreadId();
			const ndInt32 threadCount = m_owner->GetThreadCount();
			const ndInt32 jointCount = jointArray.GetCount();
			const ndInt32 mask = -ndInt32(D_AVX_WORK_GROUP);
			const ndInt32* const soaJointRows = &me->m_avxJointRows[0];
			const ndInt32 soaJointCountBatches = ((jointCount + D_AVX_WORK_GROUP - 1) & mask) / D_AVX_WORK_GROUP;

			const ndConstraint* const * jointArrayPtr = &jointArray[0];
			dAvxMatrixArray& massMatrix = *me->m_avxMassMatrixArray;

			for (ndInt32 i = threadIndex; i < soaJointCountBatches; i += threadCount)
			{
				const ndConstraint* const * jointGroup = &jointArrayPtr[i * D_AVX_WORK_GROUP];
				const ndConstraint* const firstJoint = jointGroup[0];
				const ndConstraint* const lastJoint = jointGroup[D_AVX_WORK_GROUP - 1];
				const ndInt32 soaRowStartBase = soaJointRows[i];
				if (lastJoint && (firstJoint->m_rowCount == lastJoint->m_rowCount))
				{
					const ndInt32 rowCount = firstJoint->m_rowCount;
					for (ndInt32 j = 0; j < D_AVX_WORK_GROUP; j++)
					{
						const ndConstraint* const Joint = jointGroup[j];
						const ndInt32 base = Joint->m_rowStart;
						for (ndInt32 k = 0; k < rowCount; k++)
						{
							ndSoaMatrixElement* const row = &massMatrix[soaRowStartBase + k];
							row->m_coordenateAccel[j] = rightHandSide[base + k].m_coordenateAccel;
						}
					}
				}
				else
				{
					for (ndInt32 j = 0; j < D_AVX_WORK_GROUP; j++)
					{
						const ndConstraint* const Joint = jointGroup[j];
						if (Joint)
						{
							const ndInt32 rowCount = Joint->m_rowCount;
							const ndInt32 base = Joint->m_rowStart;
							for (ndInt32 k = 0; k < rowCount; k++)
							{
								ndSoaMatrixElement* const row = &massMatrix[soaRowStartBase + k];
								row->m_coordenateAccel[j] = rightHandSide[base + k].m_coordenateAccel;
							}
						}
					}
				}
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndCalculateJointsAcceleration>();
	m_firstPassCoef = ndFloat32(1.0f);

	scene->SubmitJobs<ndUpdateAcceleration>();
}

void ndDynamicsUpdateAvx2::IntegrateBodiesVelocity()
{
	D_TRACKTIME();
	class ndIntegrateBodiesVelocity : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdate* const me = world->m_solver;
			ndArray<ndBodyKinematic*>& bodyArray = me->GetBodyIslandOrder();
			const ndArray<ndJacobian>& internalForces = me->GetInternalForces();

			const ndVector timestep4(me->GetTimestepRK());
			const ndVector speedFreeze2(world->m_freezeSpeed2 * ndFloat32(0.1f));

			const ndStartEnd startEnd(bodyArray.GetCount() - me->GetUnconstrainedBodyCount(), GetThreadId(), m_owner->GetThreadCount());
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				ndBodyKinematic* const body = bodyArray[i];

				dAssert(body);
				dAssert(body->GetAsBodyDynamic());
				dAssert(body->m_bodyIsConstrained);
				const ndInt32 index = body->m_index;
				const ndJacobian& forceAndTorque = internalForces[index];
				const ndVector force(body->GetForce() + forceAndTorque.m_linear);
				const ndVector torque(body->GetTorque() + forceAndTorque.m_angular - body->GetGyroTorque());
				const ndJacobian velocStep(body->IntegrateForceAndToque(force, torque, timestep4));

				if (!body->m_equilibrium0)
				{
					body->m_veloc += velocStep.m_linear;
					body->m_omega += velocStep.m_angular;
					body->IntegrateGyroSubstep(timestep4);
				}
				else
				{
					const ndVector velocStep2(velocStep.m_linear.DotProduct(velocStep.m_linear));
					const ndVector omegaStep2(velocStep.m_angular.DotProduct(velocStep.m_angular));
					const ndVector test(((velocStep2 > speedFreeze2) | (omegaStep2 > speedFreeze2)) & ndVector::m_negOne);
					const ndInt8 equilibrium = test.GetSignMask() ? 0 : 1;
					body->m_equilibrium0 = equilibrium;
				}
				dAssert(body->m_veloc.m_w == ndFloat32(0.0f));
				dAssert(body->m_omega.m_w == ndFloat32(0.0f));
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndIntegrateBodiesVelocity>();
}

void ndDynamicsUpdateAvx2::CalculateJointsForce()
{
	D_TRACKTIME();
	class ndCalculateJointsForce : public ndScene::ndBaseJob
	{
		public:
		ndCalculateJointsForce()
			:m_one(ndAvxFloat::m_one)
			,m_zero(ndAvxFloat::m_zero)
		{
		}

		ndFloat32 JointForce(ndInt32 block, ndSoaMatrixElement* const massMatrix)
		{
			ndAvxFloat weight0;
			ndAvxFloat weight1;
			ndAvxVector6 forceM0;
			ndAvxVector6 forceM1;
			ndAvxFloat preconditioner0;
			ndAvxFloat preconditioner1;
			ndAvxFloat normalForce[D_CONSTRAINT_MAX_ROWS + 1];

			ndConstraint** const jointGroup = &m_jointArray[block];

			if (jointGroup[D_AVX_WORK_GROUP - 1])
			{
				for (ndInt32 i = 0; i < D_AVX_WORK_GROUP; i++)
				{
					const ndConstraint* const joint = jointGroup[i];
					const ndBodyKinematic* const body0 = joint->GetBody0();
					const ndBodyKinematic* const body1 = joint->GetBody1();

					const ndInt32 m0 = body0->m_index;
					const ndInt32 m1 = body1->m_index;

					weight0[i] = body0->m_weigh;
					weight1[i] = body1->m_weigh;
					preconditioner0[i] = joint->m_preconditioner0;
					preconditioner1[i] = joint->m_preconditioner1;

					forceM0.m_linear.m_x[i] = m_internalForces[m0].m_linear.m_x;
					forceM0.m_linear.m_y[i] = m_internalForces[m0].m_linear.m_y;
					forceM0.m_linear.m_z[i] = m_internalForces[m0].m_linear.m_z;
					forceM0.m_angular.m_x[i] = m_internalForces[m0].m_angular.m_x;
					forceM0.m_angular.m_y[i] = m_internalForces[m0].m_angular.m_y;
					forceM0.m_angular.m_z[i] = m_internalForces[m0].m_angular.m_z;

					forceM1.m_linear.m_x[i] = m_internalForces[m1].m_linear.m_x;
					forceM1.m_linear.m_y[i] = m_internalForces[m1].m_linear.m_y;
					forceM1.m_linear.m_z[i] = m_internalForces[m1].m_linear.m_z;
					forceM1.m_angular.m_x[i] = m_internalForces[m1].m_angular.m_x;
					forceM1.m_angular.m_y[i] = m_internalForces[m1].m_angular.m_y;
					forceM1.m_angular.m_z[i] = m_internalForces[m1].m_angular.m_z;
				}
			}
			else
			{
				weight0 = m_zero;
				weight1 = m_zero;
				preconditioner0 = m_zero;
				preconditioner1 = m_zero;
				
				forceM0.m_linear.m_x = m_zero;
				forceM0.m_linear.m_y = m_zero;
				forceM0.m_linear.m_z = m_zero;
				forceM0.m_angular.m_x = m_zero;
				forceM0.m_angular.m_y = m_zero;
				forceM0.m_angular.m_z = m_zero;
				
				forceM1.m_linear.m_x = m_zero;
				forceM1.m_linear.m_y = m_zero;
				forceM1.m_linear.m_z = m_zero;
				forceM1.m_angular.m_x = m_zero;
				forceM1.m_angular.m_y = m_zero;
				forceM1.m_angular.m_z = m_zero;
				for (ndInt32 i = 0; i < D_AVX_WORK_GROUP; i++)
				{
					const ndConstraint* const joint = jointGroup[i];
					if (joint)
					{
						const ndBodyKinematic* const body0 = joint->GetBody0();
						const ndBodyKinematic* const body1 = joint->GetBody1();
				
						const ndInt32 m0 = body0->m_index;
						const ndInt32 m1 = body1->m_index;

						preconditioner0[i] = joint->m_preconditioner0;
						preconditioner1[i] = joint->m_preconditioner1;
				
						forceM0.m_linear.m_x[i] = m_internalForces[m0].m_linear.m_x;
						forceM0.m_linear.m_y[i] = m_internalForces[m0].m_linear.m_y;
						forceM0.m_linear.m_z[i] = m_internalForces[m0].m_linear.m_z;
						forceM0.m_angular.m_x[i] = m_internalForces[m0].m_angular.m_x;
						forceM0.m_angular.m_y[i] = m_internalForces[m0].m_angular.m_y;
						forceM0.m_angular.m_z[i] = m_internalForces[m0].m_angular.m_z;
				
						forceM1.m_linear.m_x[i] = m_internalForces[m1].m_linear.m_x;
						forceM1.m_linear.m_y[i] = m_internalForces[m1].m_linear.m_y;
						forceM1.m_linear.m_z[i] = m_internalForces[m1].m_linear.m_z;
						forceM1.m_angular.m_x[i] = m_internalForces[m1].m_angular.m_x;
						forceM1.m_angular.m_y[i] = m_internalForces[m1].m_angular.m_y;
						forceM1.m_angular.m_z[i] = m_internalForces[m1].m_angular.m_z;
				
						weight0[i] = body0->m_weigh;
						weight1[i] = body1->m_weigh;
					}
				}
			}

			forceM0.m_linear.m_x = forceM0.m_linear.m_x * preconditioner0;
			forceM0.m_linear.m_y = forceM0.m_linear.m_y * preconditioner0;
			forceM0.m_linear.m_z = forceM0.m_linear.m_z * preconditioner0;
			forceM0.m_angular.m_x = forceM0.m_angular.m_x * preconditioner0;
			forceM0.m_angular.m_y = forceM0.m_angular.m_y * preconditioner0;
			forceM0.m_angular.m_z = forceM0.m_angular.m_z * preconditioner0;

			forceM1.m_linear.m_x = forceM1.m_linear.m_x * preconditioner1;
			forceM1.m_linear.m_y = forceM1.m_linear.m_y * preconditioner1;
			forceM1.m_linear.m_z = forceM1.m_linear.m_z * preconditioner1;
			forceM1.m_angular.m_x = forceM1.m_angular.m_x * preconditioner1;
			forceM1.m_angular.m_y = forceM1.m_angular.m_y * preconditioner1;
			forceM1.m_angular.m_z = forceM1.m_angular.m_z * preconditioner1;

			preconditioner0 = preconditioner0 * weight0;
			preconditioner1 = preconditioner1 * weight1;

			normalForce[0] = m_one;
			ndAvxFloat accNorm(m_zero);
			const ndInt32 rowsCount = jointGroup[0]->m_rowCount;
		
			for (ndInt32 j = 0; j < rowsCount; j++)
			{
				ndSoaMatrixElement* const row = &massMatrix[j];

				ndAvxFloat a0(row->m_JMinv.m_jacobianM0.m_linear.m_x * forceM0.m_linear.m_x);
				ndAvxFloat a1(row->m_JMinv.m_jacobianM1.m_linear.m_x * forceM1.m_linear.m_x);
				a0 = a0.MulAdd(row->m_JMinv.m_jacobianM0.m_angular.m_x, forceM0.m_angular.m_x);
				a1 = a1.MulAdd(row->m_JMinv.m_jacobianM1.m_angular.m_x, forceM1.m_angular.m_x);

				a0 = a0.MulAdd(row->m_JMinv.m_jacobianM0.m_linear.m_y, forceM0.m_linear.m_y);
				a1 = a1.MulAdd(row->m_JMinv.m_jacobianM1.m_linear.m_y, forceM1.m_linear.m_y);
				a0 = a0.MulAdd(row->m_JMinv.m_jacobianM0.m_angular.m_y, forceM0.m_angular.m_y);
				a1 = a1.MulAdd(row->m_JMinv.m_jacobianM1.m_angular.m_y, forceM1.m_angular.m_y);

				a0 = a0.MulAdd(row->m_JMinv.m_jacobianM0.m_linear.m_z, forceM0.m_linear.m_z);
				a1 = a1.MulAdd(row->m_JMinv.m_jacobianM1.m_linear.m_z, forceM1.m_linear.m_z);
				a0 = a0.MulAdd(row->m_JMinv.m_jacobianM0.m_angular.m_z, forceM0.m_angular.m_z);
				a1 = a1.MulAdd(row->m_JMinv.m_jacobianM1.m_angular.m_z, forceM1.m_angular.m_z);

				ndAvxFloat a (a0 + a1);
				a = row->m_coordenateAccel.MulSub(row->m_force, row->m_diagDamp) - a;
				ndAvxFloat f(row->m_force.MulAdd(row->m_invJinvMJt, a));

				const ndAvxFloat frictionNormal(normalForce, row->m_normalForceIndex);
				const ndAvxFloat lowerFrictionForce(frictionNormal * row->m_lowerBoundFrictionCoefficent);
				const ndAvxFloat upperFrictionForce(frictionNormal * row->m_upperBoundFrictionCoefficent);

				a = a & (f < upperFrictionForce) & (f > lowerFrictionForce);
				f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);

				accNorm = accNorm.MulAdd(a, a);
				normalForce[j + 1] = f;

				const ndAvxFloat deltaForce(f - row->m_force);
				const ndAvxFloat deltaForce0(deltaForce * preconditioner0);
				const ndAvxFloat deltaForce1(deltaForce * preconditioner1);

				forceM0.m_linear.m_x = forceM0.m_linear.m_x.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_x, deltaForce0);
				forceM0.m_linear.m_y = forceM0.m_linear.m_y.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_y, deltaForce0);
				forceM0.m_linear.m_z = forceM0.m_linear.m_z.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_z, deltaForce0);
				forceM0.m_angular.m_x = forceM0.m_angular.m_x.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_x, deltaForce0);
				forceM0.m_angular.m_y = forceM0.m_angular.m_y.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_y, deltaForce0);
				forceM0.m_angular.m_z = forceM0.m_angular.m_z.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_z, deltaForce0);

				forceM1.m_linear.m_x = forceM1.m_linear.m_x.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_x, deltaForce1);
				forceM1.m_linear.m_y = forceM1.m_linear.m_y.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_y, deltaForce1);
				forceM1.m_linear.m_z = forceM1.m_linear.m_z.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_z, deltaForce1);
				forceM1.m_angular.m_x = forceM1.m_angular.m_x.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_x, deltaForce1);
				forceM1.m_angular.m_y = forceM1.m_angular.m_y.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_y, deltaForce1);
				forceM1.m_angular.m_z = forceM1.m_angular.m_z.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_z, deltaForce1);
			}

			const ndFloat32 tol = ndFloat32(0.5f);
			const ndFloat32 tol2 = tol * tol;

			ndAvxFloat maxAccel(accNorm);
			for (ndInt32 k = 0; (k < 4) && (maxAccel.AddHorizontal() > tol2); k++)
			{
				maxAccel = m_zero;
				for (ndInt32 j = 0; j < rowsCount; j++)
				{
					ndSoaMatrixElement* const row = &massMatrix[j];

					ndAvxFloat a0(row->m_JMinv.m_jacobianM0.m_linear.m_x * forceM0.m_linear.m_x);
					ndAvxFloat a1(row->m_JMinv.m_jacobianM1.m_linear.m_x * forceM1.m_linear.m_x);
					a0 = a0.MulAdd(row->m_JMinv.m_jacobianM0.m_angular.m_x, forceM0.m_angular.m_x);
					a1 = a1.MulAdd(row->m_JMinv.m_jacobianM1.m_angular.m_x, forceM1.m_angular.m_x);

					a0 = a0.MulAdd(row->m_JMinv.m_jacobianM0.m_linear.m_y, forceM0.m_linear.m_y);
					a1 = a1.MulAdd(row->m_JMinv.m_jacobianM1.m_linear.m_y, forceM1.m_linear.m_y);
					a0 = a0.MulAdd(row->m_JMinv.m_jacobianM0.m_angular.m_y, forceM0.m_angular.m_y);
					a1 = a1.MulAdd(row->m_JMinv.m_jacobianM1.m_angular.m_y, forceM1.m_angular.m_y);

					a0 = a0.MulAdd(row->m_JMinv.m_jacobianM0.m_linear.m_z, forceM0.m_linear.m_z);
					a1 = a1.MulAdd(row->m_JMinv.m_jacobianM1.m_linear.m_z, forceM1.m_linear.m_z);
					a0 = a0.MulAdd(row->m_JMinv.m_jacobianM0.m_angular.m_z, forceM0.m_angular.m_z);
					a1 = a1.MulAdd(row->m_JMinv.m_jacobianM1.m_angular.m_z, forceM1.m_angular.m_z);

					ndAvxFloat a(a0 + a1);
					const ndAvxFloat force(normalForce[j + 1]);
					a = row->m_coordenateAccel.MulSub(force, row->m_diagDamp) - a;
					ndAvxFloat f(force.MulAdd(row->m_invJinvMJt, a));

					const ndAvxFloat frictionNormal(normalForce, row->m_normalForceIndex);
					const ndAvxFloat lowerFrictionForce(frictionNormal * row->m_lowerBoundFrictionCoefficent);
					const ndAvxFloat upperFrictionForce(frictionNormal * row->m_upperBoundFrictionCoefficent);

					a = a & (f < upperFrictionForce) & (f > lowerFrictionForce);
					f = f.GetMax(lowerFrictionForce).GetMin(upperFrictionForce);

					maxAccel = maxAccel.MulAdd(a, a);
					normalForce[j + 1] = f;

					const ndAvxFloat deltaForce(f - force);
					const ndAvxFloat deltaForce0(deltaForce * preconditioner0);
					const ndAvxFloat deltaForce1(deltaForce * preconditioner1);

					forceM0.m_linear.m_x = forceM0.m_linear.m_x.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_x, deltaForce0);
					forceM0.m_linear.m_y = forceM0.m_linear.m_y.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_y, deltaForce0);
					forceM0.m_linear.m_z = forceM0.m_linear.m_z.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_z, deltaForce0);
					forceM0.m_angular.m_x = forceM0.m_angular.m_x.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_x, deltaForce0);
					forceM0.m_angular.m_y = forceM0.m_angular.m_y.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_y, deltaForce0);
					forceM0.m_angular.m_z = forceM0.m_angular.m_z.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_z, deltaForce0);

					forceM1.m_linear.m_x = forceM1.m_linear.m_x.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_x, deltaForce1);
					forceM1.m_linear.m_y = forceM1.m_linear.m_y.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_y, deltaForce1);
					forceM1.m_linear.m_z = forceM1.m_linear.m_z.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_z, deltaForce1);
					forceM1.m_angular.m_x = forceM1.m_angular.m_x.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_x, deltaForce1);
					forceM1.m_angular.m_y = forceM1.m_angular.m_y.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_y, deltaForce1);
					forceM1.m_angular.m_z = forceM1.m_angular.m_z.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_z, deltaForce1);
				}
			}

			ndAvxFloat mask(ndAvxFloat::m_mask);
			if ((block + D_AVX_WORK_GROUP) > m_activeCount)
			{
				for (ndInt32 i = 0; i < D_AVX_WORK_GROUP; i++)
				{
					const ndConstraint* const joint = jointGroup[i];
					if (joint)
					{
						const ndBodyKinematic* const body0 = joint->GetBody0();
						const ndBodyKinematic* const body1 = joint->GetBody1();
						dAssert(body0);
						dAssert(body1);

						const ndInt32 resting = body0->m_equilibrium0 & body1->m_equilibrium0;
						if (resting)
						{
							mask[i] = ndFloat32(0.0f);
						}
					}
					else
					{
						mask[i] = ndFloat32(0.0f);
					}
				}
			}

			forceM0.m_linear.m_x = m_zero;
			forceM0.m_linear.m_y = m_zero;
			forceM0.m_linear.m_z = m_zero;
			forceM0.m_angular.m_x = m_zero;
			forceM0.m_angular.m_y = m_zero;
			forceM0.m_angular.m_z = m_zero;

			forceM1.m_linear.m_x = m_zero;
			forceM1.m_linear.m_y = m_zero;
			forceM1.m_linear.m_z = m_zero;
			forceM1.m_angular.m_x = m_zero;
			forceM1.m_angular.m_y = m_zero;
			forceM1.m_angular.m_z = m_zero;
			for (ndInt32 i = 0; i < rowsCount; i++)
			{
				ndSoaMatrixElement* const row = &massMatrix[i];
				const ndAvxFloat force(row->m_force.Select(normalForce[i + 1], mask));
				row->m_force = force;

				forceM0.m_linear.m_x = forceM0.m_linear.m_x.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_x, force);
				forceM0.m_linear.m_y = forceM0.m_linear.m_y.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_y, force);
				forceM0.m_linear.m_z = forceM0.m_linear.m_z.MulAdd(row->m_Jt.m_jacobianM0.m_linear.m_z, force);
				forceM0.m_angular.m_x = forceM0.m_angular.m_x.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_x, force);
				forceM0.m_angular.m_y = forceM0.m_angular.m_y.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_y, force);
				forceM0.m_angular.m_z = forceM0.m_angular.m_z.MulAdd(row->m_Jt.m_jacobianM0.m_angular.m_z, force);

				forceM1.m_linear.m_x = forceM1.m_linear.m_x.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_x, force);
				forceM1.m_linear.m_y = forceM1.m_linear.m_y.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_y, force);
				forceM1.m_linear.m_z = forceM1.m_linear.m_z.MulAdd(row->m_Jt.m_jacobianM1.m_linear.m_z, force);
				forceM1.m_angular.m_x = forceM1.m_angular.m_x.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_x, force);
				forceM1.m_angular.m_y = forceM1.m_angular.m_y.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_y, force);
				forceM1.m_angular.m_z = forceM1.m_angular.m_z.MulAdd(row->m_Jt.m_jacobianM1.m_angular.m_z, force);
			}

			ndAvxFloat force0[8];
			ndAvxFloat force1[8];
			ndVector::Transpose4x4(
				force0[0].m_vector8.m_linear, 
				force0[1].m_vector8.m_linear, 
				force0[2].m_vector8.m_linear, 
				force0[3].m_vector8.m_linear,
				forceM0.m_linear.m_x.m_vector8.m_linear, 
				forceM0.m_linear.m_y.m_vector8.m_linear, 
				forceM0.m_linear.m_z.m_vector8.m_linear, ndVector::m_zero);
			ndVector::Transpose4x4(
				force0[4].m_vector8.m_linear,
				force0[5].m_vector8.m_linear,
				force0[6].m_vector8.m_linear,
				force0[7].m_vector8.m_linear,
				forceM0.m_linear.m_x.m_vector8.m_angular,
				forceM0.m_linear.m_y.m_vector8.m_angular,
				forceM0.m_linear.m_z.m_vector8.m_angular, ndVector::m_zero);
			ndVector::Transpose4x4(
				force0[0].m_vector8.m_angular,
				force0[1].m_vector8.m_angular,
				force0[2].m_vector8.m_angular,
				force0[3].m_vector8.m_angular,
				forceM0.m_angular.m_x.m_vector8.m_linear,
				forceM0.m_angular.m_y.m_vector8.m_linear,
				forceM0.m_angular.m_z.m_vector8.m_linear, ndVector::m_zero);
			ndVector::Transpose4x4(
				force0[4].m_vector8.m_angular,
				force0[5].m_vector8.m_angular,
				force0[6].m_vector8.m_angular,
				force0[7].m_vector8.m_angular,
				forceM0.m_angular.m_x.m_vector8.m_angular,
				forceM0.m_angular.m_y.m_vector8.m_angular,
				forceM0.m_angular.m_z.m_vector8.m_angular, ndVector::m_zero);

			ndVector::Transpose4x4(
				force1[0].m_vector8.m_linear,
				force1[1].m_vector8.m_linear,
				force1[2].m_vector8.m_linear,
				force1[3].m_vector8.m_linear,
				forceM1.m_linear.m_x.m_vector8.m_linear,
				forceM1.m_linear.m_y.m_vector8.m_linear,
				forceM1.m_linear.m_z.m_vector8.m_linear, ndVector::m_zero);
			ndVector::Transpose4x4(
				force1[4].m_vector8.m_linear,
				force1[5].m_vector8.m_linear,
				force1[6].m_vector8.m_linear,
				force1[7].m_vector8.m_linear,
				forceM1.m_linear.m_x.m_vector8.m_angular,
				forceM1.m_linear.m_y.m_vector8.m_angular,
				forceM1.m_linear.m_z.m_vector8.m_angular, ndVector::m_zero);
			ndVector::Transpose4x4(
				force1[0].m_vector8.m_angular,
				force1[1].m_vector8.m_angular,
				force1[2].m_vector8.m_angular,
				force1[3].m_vector8.m_angular,
				forceM1.m_angular.m_x.m_vector8.m_linear,
				forceM1.m_angular.m_y.m_vector8.m_linear,
				forceM1.m_angular.m_z.m_vector8.m_linear, ndVector::m_zero);
			ndVector::Transpose4x4(
				force1[4].m_vector8.m_angular,
				force1[5].m_vector8.m_angular,
				force1[6].m_vector8.m_angular,
				force1[7].m_vector8.m_angular,
				forceM1.m_angular.m_x.m_vector8.m_angular,
				forceM1.m_angular.m_y.m_vector8.m_angular,
				forceM1.m_angular.m_z.m_vector8.m_angular, ndVector::m_zero);

			ndRightHandSide* const rightHandSide = &m_rightHandSide[0];
			for (ndInt32 i = 0; i < D_AVX_WORK_GROUP; i++)
			{
				const ndConstraint* const joint = jointGroup[i];
				if (joint)
				{
					//const ndBodyKinematic* const body0 = joint->GetBody0();
					//const ndBodyKinematic* const body1 = joint->GetBody1();
					const ndInt32 rowCount = joint->m_rowCount;
					const ndInt32 rowStartBase = joint->m_rowStart;
					for (ndInt32 j = 0; j < rowCount; j++)
					{
						const ndSoaMatrixElement* const row = &massMatrix[j];
						rightHandSide[j + rowStartBase].m_force = row->m_force[i];
						rightHandSide[j + rowStartBase].m_maxImpact = dMax(dAbs(row->m_force[i]), rightHandSide[j + rowStartBase].m_maxImpact);
					}

					const ndInt32 index0 = (block + i) * 2 + 0;
					ndAvxFloat& outBody0 = (ndAvxFloat&)m_jointPartialForces[index0];
					outBody0 = force0[i];

					const ndInt32 index1 = (block + i) * 2 + 1;
					ndAvxFloat& outBody1 = (ndAvxFloat&)m_jointPartialForces[index1];
					outBody1 = force1[i];
				}
			}

			accNorm = m_zero.Select(accNorm, mask);
			return accNorm.AddHorizontal();
		}

		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateAvx2* const me = (ndDynamicsUpdateAvx2*)world->m_solver;

			m_leftHandSide = &me->GetLeftHandSide()[0];
			m_rightHandSide = &me->GetRightHandSide()[0];
			m_internalForces = &me->GetInternalForces()[0];
			m_jointPartialForces = &me->GetTempInternalForces()[0];
			m_jointBodyPairIndexBuffer = &me->GetJointBodyPairIndexBuffer()[0];
			ndConstraintArray& jointArray = m_owner->GetActiveContactArray();
			const ndInt32* const soaJointRows = &me->m_avxJointRows[0];
			dAvxMatrixArray& soaMassMatrixArray = *me->m_avxMassMatrixArray;
			ndSoaMatrixElement* const soaMassMatrix = &soaMassMatrixArray[0];
			m_jointArray = &jointArray[0];

			ndFloat32 accNorm = ndFloat32(0.0f);
			m_activeCount = me->m_activeJointCount;
			const ndInt32 jointCount = jointArray.GetCount();
			const ndInt32 threadIndex = GetThreadId();
			const ndInt32 threadCount = m_owner->GetThreadCount();

			const ndInt32 mask = -ndInt32(D_AVX_WORK_GROUP);
			const ndInt32 soaJointCount = ((jointCount + D_AVX_WORK_GROUP - 1) & mask) / D_AVX_WORK_GROUP;
			for (ndInt32 i = threadIndex; i < soaJointCount; i += threadCount)
			{
				accNorm += JointForce(i * D_AVX_WORK_GROUP, &soaMassMatrix[soaJointRows[i]]);
			}

			ndFloat32* const accelNorm = (ndFloat32*)m_context;
			accelNorm[threadIndex] = accNorm;
		}

		ndAvxFloat m_one;
		ndAvxFloat m_zero;
		ndConstraint** m_jointArray;
		ndJacobian* m_jointPartialForces;
		ndRightHandSide* m_rightHandSide;
		const ndJacobian* m_internalForces;
		const ndLeftHandSide* m_leftHandSide;
		const ndJointBodyPairIndex* m_jointBodyPairIndexBuffer;
		ndInt32 m_activeCount;
	};

	class ndApplyJacobianAccumulatePartialForces : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			const ndAvxFloat zero(ndAvxFloat::m_zero);
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdate* const me = (ndDynamicsUpdate*)world->m_solver;

			const ndInt32* const bodyIndex = &me->GetJointForceIndexBuffer()[0];
			const ndArray<ndBodyKinematic*>& bodyArray = m_owner->GetActiveBodyArray();
			ndAvxFloat* const internalForces = (ndAvxFloat*)&me->GetInternalForces()[0];
			const ndAvxFloat* const jointInternalForces = (ndAvxFloat*)&me->GetTempInternalForces()[0];
			const ndJointBodyPairIndex* const jointBodyPairIndexBuffer = &me->GetJointBodyPairIndexBuffer()[0];

			const ndStartEnd startEnd(bodyArray.GetCount(), GetThreadId(), m_owner->GetThreadCount());
			for (ndInt32 i = startEnd.m_start; i < startEnd.m_end; ++i)
			{
				ndAvxFloat force(zero);
				ndAvxFloat torque(zero);
				const ndBodyKinematic* const body = bodyArray[i];

				const ndInt32 startIndex = bodyIndex[i];
				const ndInt32 mask = body->m_isStatic - 1;
				const ndInt32 count = mask & (bodyIndex[i + 1] - startIndex);
				for (ndInt32 j = 0; j < count; ++j)
				{
					const ndInt32 index = jointBodyPairIndexBuffer[startIndex + j].m_joint;
					force = force + jointInternalForces[index];
				}
				internalForces[i] = force;
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	const ndInt32 passes = m_solverPasses;
	const ndInt32 threadsCount = scene->GetThreadCount();

	ndFloat32 m_accelNorm[D_MAX_THREADS_COUNT];
	ndFloat32 accNorm = D_SOLVER_MAX_ERROR * ndFloat32(2.0f);

	for (ndInt32 i = 0; (i < passes) && (accNorm > D_SOLVER_MAX_ERROR); i++)
	{
		scene->SubmitJobs<ndCalculateJointsForce>(m_accelNorm);
		scene->SubmitJobs<ndApplyJacobianAccumulatePartialForces>();

		accNorm = ndFloat32(0.0f);
		for (ndInt32 j = 0; j < threadsCount; j++)
		{
			accNorm = dMax(accNorm, m_accelNorm[j]);
		}
	}
}

void ndDynamicsUpdateAvx2::CalculateForces()
{
	D_TRACKTIME();
	if (m_world->GetScene()->GetActiveContactArray().GetCount())
	{
		m_firstPassCoef = ndFloat32(0.0f);
		if (m_world->m_skeletonList.GetCount())
		{
			InitSkeletons();
		}
		
		for (ndInt32 step = 0; step < 4; step++)
		{
			CalculateJointsAcceleration();
			CalculateJointsForce();
			if (m_world->m_skeletonList.GetCount())
			{
				UpdateSkeletons();
			}
			IntegrateBodiesVelocity();
		}
		
		UpdateForceFeedback();
	}
}

void ndDynamicsUpdateAvx2::Update()
{
	D_TRACKTIME();
	m_timestep = m_world->GetScene()->GetTimestep();

	BuildIsland();
	if (GetIslands().GetCount())
	{
		IntegrateUnconstrainedBodies();
		InitWeights();
		InitBodyArray();
		InitJacobianMatrix();
		CalculateForces();
		IntegrateBodies();
		DetermineSleepStates();
	}
}
