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

		inline ndAvxFloat(const dFloat32 val)
			:m_low(_mm256_set1_pd(val))
			,m_high(_mm256_set1_pd(val))
		{
		}

		inline ndAvxFloat(const dInt32 val)
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
			inline ndAvxFloat(const dVector& low, const dVector& high)
				:m_low(low.m_type)
				,m_high(high.m_type)
			{
			}
		#else
			inline ndAvxFloat(const dVector& low, const dVector& high)
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

		inline dFloat32& operator[] (dInt32 i)
		{
			dAssert(i >= 0);
			dAssert(i < D_AVX_WORK_GROUP);
			dFloat32* const ptr = (dFloat32*)&m_low;
			return ptr[i];
		}

		inline const dFloat32& operator[] (dInt32 i) const
		{
			dAssert(i >= 0);
			dAssert(i < D_AVX_WORK_GROUP);
			const dFloat32* const ptr = (dFloat32*)&m_low;
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


		inline dFloat32 AddHorizontal() const
		{
			//__m256 tmp0(_mm256_add_ps(m_type, _mm256_permute2f128_ps(m_type, m_type, 1)));
			//__m256 tmp1(_mm256_hadd_ps(tmp0, tmp0));
			//__m256 tmp2(_mm256_hadd_ps(tmp1, tmp1));
			//return *((dFloat32*)&tmp2);
			__m256d tmp0(_mm256_add_pd(m_low, m_high));
			__m256d tmp1(_mm256_hadd_pd(tmp0, tmp0));
			__m256d tmp2(_mm256_add_pd(tmp1, _mm256_permute2f128_pd(tmp1, tmp1, 1)));
			return *((dFloat32*)&tmp2);
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

		inline ndAvxFloat(const dFloat32 val)
			:m_type(_mm256_set1_ps(val))
		{
		}

		inline ndAvxFloat(const dInt32 val)
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

		inline ndAvxFloat(const dVector& low, const dVector& high)
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

		inline dFloat32& operator[] (dInt32 i)
		{
			dAssert(i >= 0);
			dAssert(i < D_AVX_WORK_GROUP);
			dFloat32* const ptr = (dFloat32*)&m_type;
			return ptr[i];
		}

		inline const dFloat32& operator[] (dInt32 i) const
		{
			dAssert(i >= 0);
			dAssert(i < D_AVX_WORK_GROUP);
			const dFloat32* const ptr = (dFloat32*)&m_type;
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


		inline dFloat32 AddHorizontal() const
		{
			__m256 tmp0(_mm256_add_ps(m_type, _mm256_permute2f128_ps(m_type, m_type, 1)));
			__m256 tmp1(_mm256_hadd_ps(tmp0, tmp0));
			__m256 tmp2(_mm256_hadd_ps(tmp1, tmp1));
			return *((dFloat32*)&tmp2);
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
			dInt32 m_ints[D_AVX_WORK_GROUP];
		};
	} D_GCC_NEWTON_ALIGN_32;
#endif

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

class dAvxMatrixArray : public dArray<ndSoaMatrixElement>
{
};

ndDynamicsUpdateAvx2::ndDynamicsUpdateAvx2(ndWorld* const world)
	:ndDynamicsUpdate(world)
	,m_zero(dFloat32 (0.0f))
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
		ndDetermineSleepStates()
			:m_velocTol(dFloat32(1.0e-8f))
		{
		}

		void UpdateIslandState(dInt32 entry)
		{
			ndWorld* const world = m_owner->GetWorld();
			ndScene* const scene = world->GetScene();
			ndDynamicsUpdate* const me = world->m_solver;
			const ndIsland& island = me->GetIsland____()[entry];
			const dArray<dInt32>& bodyIslandOrder = me->GetBodyIslandOrder____();
			ndBodyKinematic** const bodyIslands = &scene->GetActiveBodyArray()[0];

			dFloat32 velocityDragCoeff = D_FREEZZING_VELOCITY_DRAG;

			const dInt32 count = island.m_count;
			if (count <= D_SMALL_ISLAND_COUNT)
			{
				velocityDragCoeff = dFloat32(0.9999f);
			}


			dFloat32 maxAccel = dFloat32(0.0f);
			dFloat32 maxAlpha = dFloat32(0.0f);
			dFloat32 maxSpeed = dFloat32(0.0f);
			dFloat32 maxOmega = dFloat32(0.0f);

			const dFloat32 speedFreeze = world->m_freezeSpeed2;
			const dFloat32 accelFreeze = world->m_freezeAccel2 * ((count <= D_SMALL_ISLAND_COUNT) ? dFloat32(0.01f) : dFloat32(1.0f));
			const dFloat32 acc2 = D_SOLVER_MAX_ERROR * D_SOLVER_MAX_ERROR;

			const dVector maxAccNorm2((count > 4) ? acc2 : acc2 * dFloat32(0.0625f));
			const dVector velocDragVect(velocityDragCoeff, velocityDragCoeff, velocityDragCoeff, dFloat32(0.0f));

			dInt32 stackSleeping = 1;
			dInt32 sleepCounter = 10000;
			const dInt32 start = island.m_start;

			for (dInt32 i = 0; i < count; i++)
			{
				dInt32 index = bodyIslandOrder[start + i];
				ndBodyDynamic* const dynBody = bodyIslands[index]->GetAsBodyDynamic();
				if (dynBody)
				{
					dAssert(dynBody->m_accel.m_w == dFloat32(0.0f));
					dAssert(dynBody->m_alpha.m_w == dFloat32(0.0f));
					dAssert(dynBody->m_veloc.m_w == dFloat32(0.0f));
					dAssert(dynBody->m_omega.m_w == dFloat32(0.0f));

					dVector accelTest((dynBody->m_accel.DotProduct(dynBody->m_accel) > maxAccNorm2) | (dynBody->m_alpha.DotProduct(dynBody->m_alpha) > maxAccNorm2));
					dynBody->m_accel = dynBody->m_accel & accelTest;
					dynBody->m_alpha = dynBody->m_alpha & accelTest;

					dUnsigned32 equilibrium = (dynBody->m_invMass.m_w == dFloat32(0.0f)) ? 1 : dynBody->m_autoSleep;
					const dVector isMovingMask(dynBody->m_veloc + dynBody->m_omega + dynBody->m_accel + dynBody->m_alpha);
					const dVector mask(isMovingMask.TestZero());
					const dInt32 test = mask.GetSignMask() & 7;
					if (test != 7)
					{
						const dFloat32 accel2 = dynBody->m_accel.DotProduct(dynBody->m_accel).GetScalar();
						const dFloat32 alpha2 = dynBody->m_alpha.DotProduct(dynBody->m_alpha).GetScalar();
						const dFloat32 speed2 = dynBody->m_veloc.DotProduct(dynBody->m_veloc).GetScalar();
						const dFloat32 omega2 = dynBody->m_omega.DotProduct(dynBody->m_omega).GetScalar();

						maxAccel = dMax(maxAccel, accel2);
						maxAlpha = dMax(maxAlpha, alpha2);
						maxSpeed = dMax(maxSpeed, speed2);
						maxOmega = dMax(maxOmega, omega2);
						dUnsigned32 equilibriumTest = (accel2 < accelFreeze) && (alpha2 < accelFreeze) && (speed2 < speedFreeze) && (omega2 < speedFreeze);

						if (equilibriumTest)
						{
							const dVector veloc(dynBody->m_veloc * velocDragVect);
							const dVector omega(dynBody->m_omega * velocDragVect);
							const dVector velocMask(veloc.DotProduct(veloc) > m_velocTol);
							const dVector omegaMask(omega.DotProduct(omega) > m_velocTol);
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
					ndBodyKinematic* const kinBody = dynBody->GetAsBodyKinematic();
					dAssert(kinBody);
					dUnsigned32 equilibrium = (kinBody->m_invMass.m_w == dFloat32(0.0f)) ? 1 : (kinBody->m_autoSleep & ~kinBody->m_equilibriumOverride);
					const dVector isMovingMask(kinBody->m_veloc + kinBody->m_omega);
					const dVector mask(isMovingMask.TestZero());
					const dInt32 test = mask.GetSignMask() & 7;
					if (test != 7)
					{
						const dFloat32 speed2 = kinBody->m_veloc.DotProduct(kinBody->m_veloc).GetScalar();
						const dFloat32 omega2 = kinBody->m_omega.DotProduct(kinBody->m_omega).GetScalar();

						maxSpeed = dMax(maxSpeed, speed2);
						maxOmega = dMax(maxOmega, omega2);
						dUnsigned32 equilibriumTest = (speed2 < speedFreeze) && (omega2 < speedFreeze);

						if (equilibriumTest)
						{
							const dVector veloc(kinBody->m_veloc * velocDragVect);
							const dVector omega(kinBody->m_omega * velocDragVect);
							const dVector velocMask(veloc.DotProduct(veloc) > m_velocTol);
							const dVector omegaMask(omega.DotProduct(omega) > m_velocTol);
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
				for (dInt32 i = 0; i < count; i++)
				{
					// force entire island to equilibriumTest
					dInt32 index = bodyIslandOrder[start + i];
					ndBodyDynamic* const body = bodyIslands[index]->GetAsBodyDynamic();

					if (body)
					{
						body->m_accel = dVector::m_zero;
						body->m_alpha = dVector::m_zero;
						body->m_veloc = dVector::m_zero;
						body->m_omega = dVector::m_zero;
						body->m_equilibrium = (body->m_invMass.m_w == dFloat32(0.0f)) ? 1 : body->m_autoSleep;
					}
					else
					{
						ndBodyKinematic* const kinBody = body->GetAsBodyKinematic();
						dAssert(kinBody);
						kinBody->m_veloc = dVector::m_zero;
						kinBody->m_omega = dVector::m_zero;
						kinBody->m_equilibrium = (kinBody->m_invMass.m_w == dFloat32(0.0f)) ? 1 : kinBody->m_autoSleep;
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
					for (dInt32 i = 0; i < count; i++)
					{
						dInt32 index = bodyIslandOrder[start + i];
						ndBodyDynamic* const body = bodyIslands[index]->GetAsBodyDynamic();
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
						for (dInt32 i = 0; i < count; i++)
						{
							dInt32 index = bodyIslandOrder[start + i];
							ndBodyKinematic* const body = bodyIslands[index];
							body->m_equilibrium = 0;
						}
					}
					dInt32 timeScaleSleepCount = dInt32(dFloat32(60.0f) * sleepCounter * m_timestep);

					dInt32 index = D_SLEEP_ENTRIES;
					for (dInt32 i = 1; i < D_SLEEP_ENTRIES; i++)
					{
						if (world->m_sleepTable[i].m_steps > timeScaleSleepCount)
						{
							index = i;
							break;
						}
					}
					index--;

					bool state1 =
						(maxAccel < world->m_sleepTable[index].m_maxAccel) &&
						(maxAlpha < world->m_sleepTable[index].m_maxAlpha) &&
						(maxSpeed < world->m_sleepTable[index].m_maxVeloc) &&
						(maxOmega < world->m_sleepTable[index].m_maxOmega);
					if (state1)
					{
						for (dInt32 i = 0; i < count; i++)
						{
							dInt32 index1 = bodyIslandOrder[start + i];
							ndBodyKinematic* const body = bodyIslands[index1];
							body->m_veloc = dVector::m_zero;
							body->m_omega = dVector::m_zero;
							body->m_equilibrium = body->m_autoSleep;
							ndBodyDynamic* const dynBody = body->GetAsBodyDynamic();
							if (dynBody)
							{
								dynBody->m_accel = dVector::m_zero;
								dynBody->m_alpha = dVector::m_zero;
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
			ndDynamicsUpdateAvx2* const me = (ndDynamicsUpdateAvx2*)world->m_solver;
			const dArray<ndIsland>& islandArray = me->GetIsland____();

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 islandCount = islandArray.GetCount();

			for (dInt32 i = threadIndex; i < islandCount; i += threadCount)
			{
				UpdateIslandState(i);
			}
		}

		dVector m_velocTol;
	};

	ndScene* const scene = m_world->GetScene();
	scene->SubmitJobs<ndDetermineSleepStates>();
}

void ndDynamicsUpdateAvx2::SortJoints()
{
	D_TRACKTIME();

	class ndSetRowStarts : public ndScene::ndBaseJob
	{
		public:
		class ndRowsCount
		{
			public:
			dInt32 m_rowsCount;
			dInt32 m_soaJointRowCount;
		};

		void SetRowsCount()
		{
			ndWorld* const world = m_owner->GetWorld();
			ndScene* const scene = world->GetScene();
			ndConstraintArray& jointArray = scene->GetActiveContactArray();
			const dInt32 count = jointArray.GetCount();

			dInt32 rowCount = 1;
			for (dInt32 i = 0; i < count; i++)
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
			ndScene* const scene = world->GetScene();
			ndDynamicsUpdateAvx2* const me = (ndDynamicsUpdateAvx2*)world->m_solver;
			ndConstraintArray& jointArray = scene->GetActiveContactArray();
			const dInt32 count = jointArray.GetCount();

			dInt32 soaJointRowCount = 0;
			dArray<dInt32>& soaJointRows = me->m_avxJointRows;
			dInt32 soaJointCountBatches = soaJointRows.GetCount();
			for (dInt32 i = 0; i < soaJointCountBatches; i++)
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
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();

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
		//	jointArray.SetCount(0);
		return;
	}

	ndScene* const scene = m_world->GetScene();
	ndConstraintArray& jointArray = scene->GetActiveContactArray();

	#ifdef _DEBUG
		for (dInt32 i = 1; i < m_activeJointCount; i++)
		{
			ndConstraint* const joint0 = jointArray[i - 1];
			ndConstraint* const joint1 = jointArray[i - 0];
			dAssert(!joint0->m_resting);
			dAssert(!joint1->m_resting);
			dAssert(joint0->m_rowCount >= joint1->m_rowCount);
			dAssert(!(joint0->GetBody0()->m_resting & joint0->GetBody1()->m_resting));
			dAssert(!(joint1->GetBody0()->m_resting & joint1->GetBody1()->m_resting));
		}

		for (dInt32 i = m_activeJointCount + 1; i < jointArray.GetCount(); i++)
		{
			ndConstraint* const joint0 = jointArray[i - 1];
			ndConstraint* const joint1 = jointArray[i - 0];
			dAssert(joint0->m_resting);
			dAssert(joint1->m_resting);
			dAssert(joint0->m_rowCount >= joint1->m_rowCount);
			dAssert(joint0->GetBody0()->m_resting & joint0->GetBody1()->m_resting);
			dAssert(joint1->GetBody0()->m_resting & joint1->GetBody1()->m_resting);
		}
	#endif

	const dInt32 mask = -dInt32(D_AVX_WORK_GROUP);
	const dInt32 jointCount = jointArray.GetCount();
	const dInt32 soaJointCount = (jointCount + D_AVX_WORK_GROUP - 1) & mask;
	dAssert(jointArray.GetCapacity() > soaJointCount);
	ndConstraint** const jointArrayPtr = &jointArray[0];
	for (dInt32 i = jointCount; i < soaJointCount; i++)
	{
		jointArrayPtr[i] = nullptr;
	}

	if (m_activeJointCount - jointArray.GetCount())
	{
		const dInt32 base = m_activeJointCount & mask;
		const dInt32 count = jointArrayPtr[base + D_AVX_WORK_GROUP - 1] ? D_AVX_WORK_GROUP : jointArray.GetCount() - base;
		dAssert(count <= D_AVX_WORK_GROUP);
		ndConstraint** const array = &jointArrayPtr[base];
		for (dInt32 j = 1; j < count; j++)
		{
			dInt32 slot = j;
			ndConstraint* const joint = array[slot];
			for (; (slot > 0) && (array[slot - 1]->m_rowCount < joint->m_rowCount); slot--)
			{
				array[slot] = array[slot - 1];
			}
			array[slot] = joint;
		}
	}

	ndSetRowStarts::ndRowsCount rowsCount;
	const dInt32 soaJointCountBatches = soaJointCount / D_AVX_WORK_GROUP;
	m_avxJointRows.SetCount(soaJointCountBatches);
	scene->SubmitJobs<ndSetRowStarts>(&rowsCount);
	
	dInt32 rowCount = 1;
	for (dInt32 i = 0; i < jointArray.GetCount(); i++)
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
		const dInt32 maxRowCount = m_leftHandSide.GetCount();
		for (dInt32 i = 0; i < jointArray.GetCount(); i++)
		{
			ndConstraint* const joint = jointArray[i];
			dAssert(joint->m_rowStart < m_leftHandSide.GetCount());
			dAssert((joint->m_rowStart + joint->m_rowCount) <= maxRowCount);
		}

		for (dInt32 i = 0; i < jointCount; i += D_AVX_WORK_GROUP)
		{
			const dInt32 count = jointArrayPtr[i + D_AVX_WORK_GROUP - 1] ? D_AVX_WORK_GROUP : jointCount - i;
			for (dInt32 j = 1; j < count; j++)
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
	ndScene* const scene = m_world->GetScene();
	const dArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	GetInternalForces().SetCount(bodyArray.GetCount());

	dInt32 bodyCount = 0;
	const dInt32 totalBodyCount = bodyArray.GetCount() - 1;
	ndBodyIndexPair* const buffer0 = (ndBodyIndexPair*)&GetInternalForces()[0];
	for (dInt32 i = 0; i < totalBodyCount; i++)
	{
		ndBodyKinematic* const body = bodyArray[i];
		if (!(body->m_resting & body->m_islandSleep) || body->GetAsBodyPlayerCapsule())
		{
			buffer0[bodyCount].m_body = body;
			if (body->m_invMass.m_w > dFloat32(0.0f))
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

	dArray<ndIsland>& islands = GetIsland____();
	dArray<dInt32>& islandOrder = GetBodyIslandOrder____();

	islands.SetCount(0);
	islandOrder.SetCount(bodyCount);
	
	m_unConstrainedBodyCount____ = 0;
	if (bodyCount)
	{
		// sort using counting sort o(n)
		dInt32 scans[2];
		scans[0] = 0;
		scans[1] = 0;
		for (dInt32 i = 0; i < bodyCount; i++)
		{
			dInt32 j = 1 - buffer0[i].m_root->m_bodyIsConstrained;
			scans[j] ++;
		}
		scans[1] = scans[0];
		scans[0] = 0;
		ndBodyIndexPair* const buffer2 = buffer0 + bodyCount;
		for (dInt32 i = 0; i < bodyCount; i++)
		{
			const dInt32 key = 1 - buffer0[i].m_root->m_bodyIsConstrained;
			const dInt32 j = scans[key];
			buffer2[j] = buffer0[i];
			scans[key] = j + 1;
		}

		const ndBodyIndexPair* const buffer1 = buffer0 + bodyCount;
		for (dInt32 i = 0; i < bodyCount; i++)
		{
			dAssert(bodyArray[buffer1[i].m_body->m_index] == buffer1[i].m_body);
			dAssert((i == bodyCount - 1) || (buffer1[i].m_root->m_bodyIsConstrained >= buffer1[i + 1].m_root->m_bodyIsConstrained));

			islandOrder[i] = buffer1[i].m_body->m_index;
			if (buffer1[i].m_root->m_rank == -1)
			{
				buffer1[i].m_root->m_rank = 0;
				ndIsland island(buffer1[i].m_root);
				islands.PushBack(island);
			}
			buffer1[i].m_root->m_rank += 1;
		}

		dInt32 start = 0;
		dInt32 islandMaxKeySize = 0;
		dInt32 unConstrainedCount = 0;
		for (dInt32 i = 0; i < islands.GetCount(); i++)
		{
			ndIsland& island = islands[i];
			island.m_start = start;
			island.m_count = island.m_root->m_rank;
			islandMaxKeySize = dMax(islandMaxKeySize, island.m_count);
			start += island.m_count;
			unConstrainedCount += island.m_root->m_bodyIsConstrained ? 0 : 1;
		}

		m_unConstrainedBodyCount____ = unConstrainedCount;

		class EvaluateKey
		{
			public:
			dUnsigned32 GetKey(const ndIsland& island) const
			{
				dUnsigned32 key = island.m_count * 2 + island.m_root->m_bodyIsConstrained;
				const dUnsigned32 maxVal = 1 << (D_MAX_BODY_RADIX_BIT * 2);
				dAssert(key < maxVal);
				return maxVal - key;
			}
		};

		scene->CountingSort<ndIsland, D_MAX_BODY_RADIX_BIT, EvaluateKey>(&islands[0], (ndIsland*)GetTempBuffer(), islands.GetCount(), 0);
		if (islandMaxKeySize >= 256)
		{
			scene->CountingSort<ndIsland, D_MAX_BODY_RADIX_BIT, EvaluateKey>(&islands[0], (ndIsland*)GetTempBuffer(), islands.GetCount(), 1);
		}
	}
}

void ndDynamicsUpdateAvx2::BuildIsland()
{
	ndScene* const scene = m_world->GetScene();
	const dArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
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
			ndScene* const scene = world->GetScene();
			ndDynamicsUpdateAvx2* const me = (ndDynamicsUpdateAvx2*)world->m_solver;
			dArray<dInt32>& bodyIslandOrder = me->GetBodyIslandOrder____();
			ndBodyKinematic** const bodyArray = &scene->GetActiveBodyArray()[0];

			const dFloat32 timestep = m_timestep;
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = me->GetUnconstrainedBodyCount();

			const dInt32 stride = bodyCount / threadCount;
			const dInt32 start0 = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : bodyCount - start0;
			const dInt32 start = bodyIslandOrder.GetCount() - bodyCount + start0;

			for (dInt32 i = 0; i < blockSize; i++)
			{
				dInt32 index = bodyIslandOrder[start + i];
				ndBodyKinematic* const body = bodyArray[index]->GetAsBodyKinematic();
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
			ndScene* const scene = world->GetScene();
			const dArray<dInt32>& bodyIslandOrder = me->GetBodyIslandOrder____();
			ndBodyKinematic** const bodyArray = &scene->GetActiveBodyArray()[0];

			const dFloat32 timestep = m_timestep;
			const dVector invTime(me->m_invTimestep);

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = bodyIslandOrder.GetCount();
			const dInt32 stride = bodyCount / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : bodyCount - start;

			for (dInt32 i = 0; i < blockSize; i++)
			{
				dInt32 index = bodyIslandOrder[start + i];
				ndBodyDynamic* const dynBody = bodyArray[index]->GetAsBodyDynamic();

				// the initial velocity and angular velocity were stored in m_accel and dynBody->m_alpha for memory saving
				if (dynBody)
				{
					if (!dynBody->m_equilibrium)
					{
						dynBody->m_accel = invTime * (dynBody->m_veloc - dynBody->m_accel);
						dynBody->m_alpha = invTime * (dynBody->m_omega - dynBody->m_alpha);
						dynBody->IntegrateVelocity(timestep);
					}
				}
				else
				{
					ndBodyKinematic* const kinBody = dynBody->GetAsBodyKinematic();
					dAssert(kinBody);
					if (!kinBody->m_equilibrium)
					{
						kinBody->IntegrateVelocity(timestep);
					}
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
			const ndConstraintArray& jointArray = m_owner->GetActiveContactArray();

			dFloat32 maxExtraPasses = dFloat32(1.0f);
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 jointCount = jointArray.GetCount();

			const dInt32 stride = jointCount / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : jointCount - start;

			for (dInt32 i = 0; i < blockSize; i++)
			{
				ndConstraint* const constraint = jointArray[i + start];
				ndBodyKinematic* const body0 = constraint->GetBody0();
				ndBodyKinematic* const body1 = constraint->GetBody1();

				if (body1->m_invMass.m_w > dFloat32(0.0f))
				{
					dScopeSpinLock lock(body1->m_lock);
					body1->m_weigh += dFloat32(1.0f);
					maxExtraPasses = dMax(body1->m_weigh, maxExtraPasses);
				}
				else if (body1->m_weigh != dFloat32(1.0f))
				{
					body1->m_weigh = dFloat32(1.0f);
				}
				dScopeSpinLock lock(body0->m_lock);
				body0->m_weigh += dFloat32(1.0f);
				dAssert(body0->m_invMass.m_w != dFloat32(0.0f));
				maxExtraPasses = dMax(body0->m_weigh, maxExtraPasses);
			}

			dFloat32* const extraPasses = (dFloat32*)m_context;
			extraPasses[threadIndex] = maxExtraPasses;
		}
	};

	ndScene* const scene = m_world->GetScene();
	m_invTimestep = dFloat32(1.0f) / m_timestep;
	m_invStepRK = dFloat32(0.25f);
	m_timestepRK = m_timestep * m_invStepRK;
	m_invTimestepRK = m_invTimestep * dFloat32(4.0f);

	const dArray<ndBodyKinematic*>& bodyArray = scene->GetActiveBodyArray();
	const dInt32 bodyCount = bodyArray.GetCount();
	GetInternalForces().SetCount(bodyCount);

	dFloat32 extraPassesArray[D_MAX_THREADS_COUNT];
	memset(extraPassesArray, 0, sizeof(extraPassesArray));
	scene->SubmitJobs<ndInitWeights>(extraPassesArray);

	dFloat32 extraPasses = dFloat32(0.0f);
	const dInt32 threadCount = scene->GetThreadCount();
	for (dInt32 i = 0; i < threadCount; i++)
	{
		extraPasses = dMax(extraPasses, extraPassesArray[i]);
	}

	const dInt32 conectivity = 7;
	m_solverPasses = m_world->GetSolverIterations() + 2 * dInt32(extraPasses) / conectivity + 1;
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
			ndScene* const scene = world->GetScene();
			ndDynamicsUpdateAvx2* const me = (ndDynamicsUpdateAvx2*)world->m_solver;
			const dArray<dInt32>& bodyIslandOrder = me->GetBodyIslandOrder____();
			ndBodyKinematic** const bodyArray = &scene->GetActiveBodyArray()[0];

			const dFloat32 timestep = m_timestep;
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = bodyIslandOrder.GetCount() - me->GetUnconstrainedBodyCount();

			const dInt32 stride = bodyCount / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : bodyCount - start;

			for (dInt32 i = 0; i < blockSize; i++)
			{
				dInt32 index = bodyIslandOrder[start + i];
				ndBodyDynamic* const body = bodyArray[index]->GetAsBodyDynamic();
				if (body)
				{
					dAssert(body->m_bodyIsConstrained);
					body->UpdateInvInertiaMatrix();
					body->AddDampingAcceleration(timestep);

					const dVector localOmega(body->m_matrix.UnrotateVector(body->m_omega));
					const dVector localAngularMomentum(body->m_mass * localOmega);
					const dVector angularMomentum(body->m_matrix.RotateVector(localAngularMomentum));
					body->m_gyroTorque = body->m_omega.CrossProduct(angularMomentum);
					body->m_gyroAlpha = body->m_invWorldInertiaMatrix.RotateVector(body->m_gyroTorque);

					body->m_accel = body->m_veloc;
					body->m_alpha = body->m_omega;
					body->m_gyroRotation = body->m_rotation;
				}
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
	for (dInt32 i = joint->GetRowsCount() - 1; i >= 0; i--)
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
	const dInt32 dof = constraintParam.m_rowsCount;
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
	const dInt32 baseIndex = joint->m_rowStart;
	for (dInt32 i = 0; i < dof; i++)
	{
		dAssert(constraintParam.m_forceBounds[i].m_jointForce);

		ndLeftHandSide* const row = &m_leftHandSide[baseIndex + i];
		ndRightHandSide* const rhs = &m_rightHandSide[baseIndex + i];

		row->m_Jt = constraintParam.m_jacobian[i];
		rhs->m_diagDamp = dFloat32(0.0f);
		rhs->m_diagonalRegularizer = dMax(constraintParam.m_diagonalRegularizer[i], dFloat32(1.0e-5f));

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
		ndInitJacobianMatrix()
			:m_zero(dFloat32 (0.0f))
		{
		}

		void BuildJacobianMatrix(ndConstraint* const joint, dInt32 jointIndex)
		{
			dAssert(joint->GetBody0());
			dAssert(joint->GetBody1());
			ndBodyKinematic* const body0 = joint->GetBody0();
			ndBodyKinematic* const body1 = joint->GetBody1();
			const ndBodyDynamic* const dynBody0 = body0->GetAsBodyDynamic();
			const ndBodyDynamic* const dynBody1 = body1->GetAsBodyDynamic();

			const dInt32 m0 = body0->m_index;
			const dInt32 m1 = body1->m_index;
			const dInt32 index = joint->m_rowStart;
			const dInt32 count = joint->m_rowCount;

			const bool isBilateral = joint->IsBilateral();

			const dMatrix& invInertia0 = body0->m_invWorldInertiaMatrix;
			const dMatrix& invInertia1 = body1->m_invWorldInertiaMatrix;
			const dVector invMass0(body0->m_invMass[3]);
			const dVector invMass1(body1->m_invMass[3]);

			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateAvx2* const me = (ndDynamicsUpdateAvx2*)world->m_solver;
			dVector force0(me->m_zero);
			dVector torque0(me->m_zero);
			if (dynBody0)
			{
				force0 = dynBody0->m_externalForce;
				torque0 = dynBody0->m_externalTorque;
			}

			dVector force1(me->m_zero);
			dVector torque1(me->m_zero);
			if (dynBody1)
			{
				force1 = dynBody1->m_externalForce;
				torque1 = dynBody1->m_externalTorque;
			}

			joint->m_preconditioner0 = dFloat32(1.0f);
			joint->m_preconditioner1 = dFloat32(1.0f);
			if ((invMass0.GetScalar() > dFloat32(0.0f)) && (invMass1.GetScalar() > dFloat32(0.0f)) && !(body0->GetSkeleton() && body1->GetSkeleton()))
			{
				const dFloat32 mass0 = body0->GetMassMatrix().m_w;
				const dFloat32 mass1 = body1->GetMassMatrix().m_w;
				if (mass0 > (D_DIAGONAL_PRECONDITIONER * mass1))
				{
					joint->m_preconditioner0 = mass0 / (mass1 * D_DIAGONAL_PRECONDITIONER);
				}
				else if (mass1 > (D_DIAGONAL_PRECONDITIONER * mass0))
				{
					joint->m_preconditioner1 = mass1 / (mass0 * D_DIAGONAL_PRECONDITIONER);
				}
			}

			ndAvxFloat forceAcc0(m_zero);
			ndAvxFloat forceAcc1(m_zero);

			const dVector weigh0(body0->m_weigh * joint->m_preconditioner0);
			const dVector weigh1(body1->m_weigh * joint->m_preconditioner0);
			const dFloat32 preconditioner0 = joint->m_preconditioner0;
			const dFloat32 preconditioner1 = joint->m_preconditioner1;

			for (dInt32 i = 0; i < count; i++)
			{
				ndLeftHandSide* const row = &m_leftHandSide[index + i];
				ndRightHandSide* const rhs = &m_rightHandSide[index + i];

				row->m_JMinv.m_jacobianM0.m_linear = row->m_Jt.m_jacobianM0.m_linear * invMass0;
				row->m_JMinv.m_jacobianM0.m_angular = invInertia0.RotateVector(row->m_Jt.m_jacobianM0.m_angular);
				row->m_JMinv.m_jacobianM1.m_linear = row->m_Jt.m_jacobianM1.m_linear * invMass1;
				row->m_JMinv.m_jacobianM1.m_angular = invInertia1.RotateVector(row->m_Jt.m_jacobianM1.m_angular);

				const ndJacobian& JMinvM0 = row->m_JMinv.m_jacobianM0;
				const ndJacobian& JMinvM1 = row->m_JMinv.m_jacobianM1;
				const dVector tmpAccel(
					JMinvM0.m_linear * force0 + JMinvM0.m_angular * torque0 +
					JMinvM1.m_linear * force1 + JMinvM1.m_angular * torque1);

				dFloat32 extenalAcceleration = -tmpAccel.AddHorizontal().GetScalar();
				rhs->m_deltaAccel = extenalAcceleration;
				rhs->m_coordenateAccel += extenalAcceleration;
				dAssert(rhs->m_jointFeebackForce);
				const dFloat32 force = rhs->m_jointFeebackForce->GetInitialGuess();

				rhs->m_force = isBilateral ? dClamp(force, rhs->m_lowerBoundFrictionCoefficent, rhs->m_upperBoundFrictionCoefficent) : force;
				rhs->m_maxImpact = dFloat32(0.0f);

				const ndJacobian& JtM0 = row->m_Jt.m_jacobianM0;
				const ndJacobian& JtM1 = row->m_Jt.m_jacobianM1;
				const dVector tmpDiag(
					weigh0 * (JMinvM0.m_linear * JtM0.m_linear + JMinvM0.m_angular * JtM0.m_angular) +
					weigh1 * (JMinvM1.m_linear * JtM1.m_linear + JMinvM1.m_angular * JtM1.m_angular));

				dFloat32 diag = tmpDiag.AddHorizontal().GetScalar();
				dAssert(diag > dFloat32(0.0f));
				rhs->m_diagDamp = diag * rhs->m_diagonalRegularizer;

				diag *= (dFloat32(1.0f) + rhs->m_diagonalRegularizer);
				rhs->m_invJinvMJt = dFloat32(1.0f) / diag;

				forceAcc0 = forceAcc0.MulAdd((ndAvxFloat&)JtM0.m_linear, ndAvxFloat (rhs->m_force * preconditioner0));
				forceAcc1 = forceAcc1.MulAdd((ndAvxFloat&)JtM1.m_linear, ndAvxFloat (rhs->m_force * preconditioner1));
			}

			const dInt32 index0 = jointIndex * 2 + 0;
			ndAvxFloat& outBody0 = (ndAvxFloat&)m_internalForces[index0];
			outBody0 = forceAcc0;

			const dInt32 index1 = jointIndex * 2 + 1;
			ndAvxFloat& outBody1 = (ndAvxFloat&)m_internalForces[index1];
			outBody1 = forceAcc1;
		}

		virtual void Execute()
		{
			D_TRACKTIME();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateAvx2* const me = (ndDynamicsUpdateAvx2*)world->m_solver;
			m_leftHandSide = &me->GetLeftHandSide()[0];
			m_rightHandSide = &me->GetRightHandSide()[0];
			m_internalForces = &me->GetTempInternalForces()[0];
			m_jointBodyPairIndexBuffer = &me->GetJointBodyPairIndexBuffer()[0];
			ndConstraint** const jointArray = &m_owner->GetActiveContactArray()[0];

			const dInt32 jointCount = m_owner->GetActiveContactArray().GetCount();
			const dInt32 bodyCount = m_owner->GetActiveBodyArray().GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			
			for (dInt32 i = threadIndex; i < jointCount; i += threadCount)
			{
				ndConstraint* const joint = jointArray[i];
				me->GetJacobianDerivatives(joint);
				BuildJacobianMatrix(joint, i);
			}
		}

		ndAvxFloat m_zero;
		ndJacobian* m_internalForces;
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
			const dInt32 jointCount = m_owner->GetActiveContactArray().GetCount();

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const ndLeftHandSide* const leftHandSide = &me->GetLeftHandSide()[0];
			const ndRightHandSide* const rightHandSide = &me->GetRightHandSide()[0];
			dAvxMatrixArray& massMatrix = *me->m_avxMassMatrixArray;

			const dInt32 mask = -dInt32(D_AVX_WORK_GROUP);
			const dInt32 soaJointCount = ((jointCount + D_AVX_WORK_GROUP - 1) & mask) / D_AVX_WORK_GROUP;
			const dInt32* const soaJointRows = &me->m_avxJointRows[0];
			
			ndAvxFloat tmpOrdinals;
			for (dInt32 i = 0; i < D_AVX_WORK_GROUP; i++)
			{
				tmpOrdinals.m_ints[i] = i;
			}
			
			const ndAvxFloat zero(dFloat32(0.0f));
			const ndAvxFloat ordinals(tmpOrdinals);
			for (dInt32 i = threadIndex; i < soaJointCount; i += threadCount)
			{
				const dInt32 index = i * D_AVX_WORK_GROUP;
				for (dInt32 j = 1; j < D_AVX_WORK_GROUP; j++)
				{
					ndConstraint* const joint = jointArray[index + j];
					if (joint)
					{
						dInt32 slot = j;
						for (; (slot > 0) && (jointArray[index + slot - 1]->m_rowCount < joint->m_rowCount); slot--)
						{
							jointArray[index + slot] = jointArray[index + slot - 1];
						}
						jointArray[index + slot] = joint;
					}
				}

				const dInt32 soaRowBase = soaJointRows[i];
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
					const dInt32 rowCount = joint->m_rowCount;
					
					for (dInt32 j = 0; j < rowCount; j++)
					{
						dVector tmp[D_AVX_WORK_GROUP];
						const ndLeftHandSide* const row0 = &leftHandSide[joint0->m_rowStart + j];
						const ndLeftHandSide* const row1 = &leftHandSide[joint1->m_rowStart + j];
						const ndLeftHandSide* const row2 = &leftHandSide[joint2->m_rowStart + j];
						const ndLeftHandSide* const row3 = &leftHandSide[joint3->m_rowStart + j];
						const ndLeftHandSide* const row4 = &leftHandSide[joint4->m_rowStart + j];
						const ndLeftHandSide* const row5 = &leftHandSide[joint5->m_rowStart + j];
						const ndLeftHandSide* const row6 = &leftHandSide[joint6->m_rowStart + j];
						const ndLeftHandSide* const row7 = &leftHandSide[joint7->m_rowStart + j];
						ndSoaMatrixElement& row = massMatrix[soaRowBase + j];
					
						dVector::Transpose4x4(tmp[0], tmp[1], tmp[2], tmp[3],
							row0->m_Jt.m_jacobianM0.m_linear,
							row1->m_Jt.m_jacobianM0.m_linear,
							row2->m_Jt.m_jacobianM0.m_linear,
							row3->m_Jt.m_jacobianM0.m_linear);
						dVector::Transpose4x4(tmp[4], tmp[5], tmp[6], tmp[7],
							row4->m_Jt.m_jacobianM0.m_linear,
							row5->m_Jt.m_jacobianM0.m_linear,
							row6->m_Jt.m_jacobianM0.m_linear,
							row7->m_Jt.m_jacobianM0.m_linear);
						row.m_Jt.m_jacobianM0.m_linear.m_x = ndAvxFloat(tmp[0], tmp[4]);
						row.m_Jt.m_jacobianM0.m_linear.m_y = ndAvxFloat(tmp[1], tmp[5]);
						row.m_Jt.m_jacobianM0.m_linear.m_z = ndAvxFloat(tmp[2], tmp[6]);
						dVector::Transpose4x4(tmp[0], tmp[1], tmp[2], tmp[3],
							row0->m_Jt.m_jacobianM0.m_angular,
							row1->m_Jt.m_jacobianM0.m_angular,
							row2->m_Jt.m_jacobianM0.m_angular,
							row3->m_Jt.m_jacobianM0.m_angular);
						dVector::Transpose4x4(tmp[4], tmp[5], tmp[6], tmp[7],
							row4->m_Jt.m_jacobianM0.m_angular,
							row5->m_Jt.m_jacobianM0.m_angular,
							row6->m_Jt.m_jacobianM0.m_angular,
							row7->m_Jt.m_jacobianM0.m_angular);
						row.m_Jt.m_jacobianM0.m_angular.m_x = ndAvxFloat(tmp[0], tmp[4]);
						row.m_Jt.m_jacobianM0.m_angular.m_y = ndAvxFloat(tmp[1], tmp[5]);
						row.m_Jt.m_jacobianM0.m_angular.m_z = ndAvxFloat(tmp[2], tmp[6]);
					
						dVector::Transpose4x4(tmp[0], tmp[1], tmp[2], tmp[3],
							row0->m_Jt.m_jacobianM1.m_linear,
							row1->m_Jt.m_jacobianM1.m_linear,
							row2->m_Jt.m_jacobianM1.m_linear,
							row3->m_Jt.m_jacobianM1.m_linear);
						dVector::Transpose4x4(tmp[4], tmp[5], tmp[6], tmp[7],
							row4->m_Jt.m_jacobianM1.m_linear,
							row5->m_Jt.m_jacobianM1.m_linear,
							row6->m_Jt.m_jacobianM1.m_linear,
							row7->m_Jt.m_jacobianM1.m_linear);
						row.m_Jt.m_jacobianM1.m_linear.m_x = ndAvxFloat(tmp[0], tmp[4]);
						row.m_Jt.m_jacobianM1.m_linear.m_y = ndAvxFloat(tmp[1], tmp[5]);
						row.m_Jt.m_jacobianM1.m_linear.m_z = ndAvxFloat(tmp[2], tmp[6]);
						dVector::Transpose4x4(tmp[0], tmp[1], tmp[2], tmp[3],
							row0->m_Jt.m_jacobianM1.m_angular,
							row1->m_Jt.m_jacobianM1.m_angular,
							row2->m_Jt.m_jacobianM1.m_angular,
							row3->m_Jt.m_jacobianM1.m_angular);
						dVector::Transpose4x4(tmp[4], tmp[5], tmp[6], tmp[7],
							row4->m_Jt.m_jacobianM1.m_angular,
							row5->m_Jt.m_jacobianM1.m_angular,
							row6->m_Jt.m_jacobianM1.m_angular,
							row7->m_Jt.m_jacobianM1.m_angular);
						row.m_Jt.m_jacobianM1.m_angular.m_x = ndAvxFloat(tmp[0], tmp[4]);
						row.m_Jt.m_jacobianM1.m_angular.m_y = ndAvxFloat(tmp[1], tmp[5]);
						row.m_Jt.m_jacobianM1.m_angular.m_z = ndAvxFloat(tmp[2], tmp[6]);
					
						dVector::Transpose4x4(tmp[0], tmp[1], tmp[2], tmp[3],
							row0->m_JMinv.m_jacobianM0.m_linear,
							row1->m_JMinv.m_jacobianM0.m_linear,
							row2->m_JMinv.m_jacobianM0.m_linear,
							row3->m_JMinv.m_jacobianM0.m_linear);
						dVector::Transpose4x4(tmp[4], tmp[5], tmp[6], tmp[7],
							row4->m_JMinv.m_jacobianM0.m_linear,
							row5->m_JMinv.m_jacobianM0.m_linear,
							row6->m_JMinv.m_jacobianM0.m_linear,
							row7->m_JMinv.m_jacobianM0.m_linear);
						row.m_JMinv.m_jacobianM0.m_linear.m_x = ndAvxFloat(tmp[0], tmp[4]);
						row.m_JMinv.m_jacobianM0.m_linear.m_y = ndAvxFloat(tmp[1], tmp[5]);
						row.m_JMinv.m_jacobianM0.m_linear.m_z = ndAvxFloat(tmp[2], tmp[6]);
						dVector::Transpose4x4(tmp[0], tmp[1], tmp[2], tmp[3],
							row0->m_JMinv.m_jacobianM0.m_angular,
							row1->m_JMinv.m_jacobianM0.m_angular,
							row2->m_JMinv.m_jacobianM0.m_angular,
							row3->m_JMinv.m_jacobianM0.m_angular);
						dVector::Transpose4x4(tmp[4], tmp[5], tmp[6], tmp[7],
							row4->m_JMinv.m_jacobianM0.m_angular,
							row5->m_JMinv.m_jacobianM0.m_angular,
							row6->m_JMinv.m_jacobianM0.m_angular,
							row7->m_JMinv.m_jacobianM0.m_angular);
						row.m_JMinv.m_jacobianM0.m_angular.m_x = ndAvxFloat(tmp[0], tmp[4]);
						row.m_JMinv.m_jacobianM0.m_angular.m_y = ndAvxFloat(tmp[1], tmp[5]);
						row.m_JMinv.m_jacobianM0.m_angular.m_z = ndAvxFloat(tmp[2], tmp[6]);
					
						dVector::Transpose4x4(tmp[0], tmp[1], tmp[2], tmp[3],
							row0->m_JMinv.m_jacobianM1.m_linear,
							row1->m_JMinv.m_jacobianM1.m_linear,
							row2->m_JMinv.m_jacobianM1.m_linear,
							row3->m_JMinv.m_jacobianM1.m_linear);
						dVector::Transpose4x4(tmp[4], tmp[5], tmp[6], tmp[7],
							row4->m_JMinv.m_jacobianM1.m_linear,
							row5->m_JMinv.m_jacobianM1.m_linear,
							row6->m_JMinv.m_jacobianM1.m_linear,
							row7->m_JMinv.m_jacobianM1.m_linear);
						row.m_JMinv.m_jacobianM1.m_linear.m_x = ndAvxFloat(tmp[0], tmp[4]);
						row.m_JMinv.m_jacobianM1.m_linear.m_y = ndAvxFloat(tmp[1], tmp[5]);
						row.m_JMinv.m_jacobianM1.m_linear.m_z = ndAvxFloat(tmp[2], tmp[6]);
						dVector::Transpose4x4(tmp[0], tmp[1], tmp[2], tmp[3],
							row0->m_JMinv.m_jacobianM1.m_angular,
							row1->m_JMinv.m_jacobianM1.m_angular,
							row2->m_JMinv.m_jacobianM1.m_angular,
							row3->m_JMinv.m_jacobianM1.m_angular);
						dVector::Transpose4x4(tmp[4], tmp[5], tmp[6], tmp[7],
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
							dInt32* const normalIndex = (dInt32*)&row.m_normalForceIndex[0];
						#endif
						for (dInt32 k = 0; k < D_AVX_WORK_GROUP; k++)
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
					for (dInt32 j = 0; j < firstJoint->m_rowCount; j++)
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
					
					for (dInt32 j = 0; j < D_AVX_WORK_GROUP; j++)
					{
						const ndConstraint* const joint = jointArray[index + j];
						if (joint)
						{
							for (dInt32 k = 0; k < joint->m_rowCount; k++)
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
									dInt32* const normalIndex = (dInt32*)&row.m_normalForceIndex[0];
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
			const dVector zero(dVector::m_zero);
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateSoa* const me = (ndDynamicsUpdateSoa*)world->m_solver;

			ndJacobian* const internalForces = &me->GetInternalForces()[0];
			const dInt32* const bodyIndex = &me->GetJointForceIndexBuffer()[0];
			const dArray<ndBodyKinematic*>& bodyArray = m_owner->GetActiveBodyArray();
			const ndJacobian* const jointInternalForces = &me->GetTempInternalForces()[0];
			const ndJointBodyPairIndex* const jointBodyPairIndexBuffer = &me->GetJointBodyPairIndexBuffer()[0];

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = bodyArray.GetCount();

			const dInt32 stride = bodyCount / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : bodyCount - start;

			for (dInt32 i = 0; i < blockSize; i++)
			{
				dInt32 startIndex = bodyIndex[start + i];
				dInt32 count = bodyIndex[start + i + 1] - startIndex;
				if (count)
				{
					dVector force(zero);
					dVector torque(zero);
					const ndBodyKinematic* const body = bodyArray[i + start];
					if (body->m_invMass.m_w > dFloat32(0.0f))
					{
						for (dInt32 j = 0; j < count; j++)
						{
							dInt32 index = jointBodyPairIndexBuffer[startIndex + j].m_joint;
							force += jointInternalForces[index].m_linear;
							torque += jointInternalForces[index].m_angular;
						}
					}
					internalForces[i + start].m_linear = force;
					internalForces[i + start].m_angular = torque;
				}
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	if (scene->GetActiveContactArray().GetCount())
	{
		D_TRACKTIME();
		m_rightHandSide[0].m_force = dFloat32(1.0f);
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
			dArray<ndRightHandSide>& rightHandSide = me->m_rightHandSide;

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 jointCount = jointArray.GetCount();

			const dFloat32 timestepRK = me->m_timestepRK;
			for (dInt32 i = threadIndex; i < jointCount; i += threadCount)
			{
				ndConstraint* const joint = jointArray[i];
				const dInt32 rows = joint->m_rowCount;
				const dInt32 first = joint->m_rowStart;

				for (dInt32 j = 0; j < rows; j++)
				{
					const ndRightHandSide* const rhs = &rightHandSide[j + first];
					dAssert(dCheckFloat(rhs->m_force));
					rhs->m_jointFeebackForce->Push(rhs->m_force);
					rhs->m_jointFeebackForce->m_force = rhs->m_force;
					rhs->m_jointFeebackForce->m_impact = rhs->m_maxImpact * timestepRK;
				}

				if (joint->GetAsBilateral())
				{
					const dArray<ndLeftHandSide>& leftHandSide = me->m_leftHandSide;
					dVector force0(me->m_zero);
					dVector force1(me->m_zero);
					dVector torque0(me->m_zero);
					dVector torque1(me->m_zero);
					for (dInt32 j = 0; j < rows; j++)
					{
						const ndRightHandSide* const rhs = &rightHandSide[j + first];
						const ndLeftHandSide* const lhs = &leftHandSide[j + first];
						const dVector f(rhs->m_force);
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
			const dInt32 threadIndex = GetThreadId();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateAvx2* const me = (ndDynamicsUpdateAvx2*)world->m_solver;
			ndSkeletonList::dNode* node = world->GetSkeletonList().GetFirst();
			for (dInt32 i = 0; i < threadIndex; i++)
			{
				node = node ? node->GetNext() : nullptr;
			}

			dArray<ndRightHandSide>& rightHandSide = me->m_rightHandSide;
			const dArray<ndLeftHandSide>& leftHandSide = me->m_leftHandSide;

			const dInt32 threadCount = m_owner->GetThreadCount();
			while (node)
			{
				ndSkeletonContainer* const skeleton = &node->GetInfo();
				skeleton->InitMassMatrix(&leftHandSide[0], &rightHandSide[0]);

				for (dInt32 i = 0; i < threadCount; i++)
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
			const dInt32 threadIndex = GetThreadId();
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateAvx2* const me = (ndDynamicsUpdateAvx2*)world->m_solver;
			ndSkeletonList::dNode* node = world->GetSkeletonList().GetFirst();
			for (dInt32 i = 0; i < threadIndex; i++)
			{
				node = node ? node->GetNext() : nullptr;
			}

			ndJacobian* const internalForces = &me->GetInternalForces()[0];
			const dArray<ndBodyKinematic*>& activeBodies = m_owner->ndScene::GetActiveBodyArray();
			const ndBodyKinematic** const bodyArray = (const ndBodyKinematic**)&activeBodies[0];

			const dInt32 threadCount = m_owner->GetThreadCount();
			while (node)
			{
				ndSkeletonContainer* const skeleton = &node->GetInfo();
				skeleton->CalculateJointForce(bodyArray, internalForces);

				for (dInt32 i = 0; i < threadCount; i++)
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
			dArray<ndLeftHandSide>& leftHandSide = me->m_leftHandSide;
			dArray<ndRightHandSide>& rightHandSide = me->m_rightHandSide;

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 jointCount = jointArray.GetCount();

			for (dInt32 i = threadIndex; i < jointCount; i += threadCount)
			{
				ndConstraint* const joint = jointArray[i];
				const dInt32 pairStart = joint->m_rowStart;
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
			const dArray<ndRightHandSide>& rightHandSide = me->m_rightHandSide;

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 jointCount = jointArray.GetCount();
			const dInt32 mask = -dInt32(D_AVX_WORK_GROUP);
			const dInt32* const soaJointRows = &me->m_avxJointRows[0];
			const dInt32 soaJointCountBatches = ((jointCount + D_AVX_WORK_GROUP - 1) & mask) / D_AVX_WORK_GROUP;

			const ndConstraint* const * jointArrayPtr = &jointArray[0];
			dAvxMatrixArray& massMatrix = *me->m_avxMassMatrixArray;

			for (dInt32 i = threadIndex; i < soaJointCountBatches; i += threadCount)
			{
				const ndConstraint* const * jointGroup = &jointArrayPtr[i * D_AVX_WORK_GROUP];
				const ndConstraint* const firstJoint = jointGroup[0];
				const ndConstraint* const lastJoint = jointGroup[D_AVX_WORK_GROUP - 1];
				const dInt32 soaRowStartBase = soaJointRows[i];
				if (lastJoint && (firstJoint->m_rowCount == lastJoint->m_rowCount))
				{
					const dInt32 rowCount = firstJoint->m_rowCount;
					for (dInt32 j = 0; j < D_AVX_WORK_GROUP; j++)
					{
						const ndConstraint* const Joint = jointGroup[j];
						const dInt32 base = Joint->m_rowStart;
						for (dInt32 k = 0; k < rowCount; k++)
						{
							ndSoaMatrixElement* const row = &massMatrix[soaRowStartBase + k];
							row->m_coordenateAccel[j] = rightHandSide[base + k].m_coordenateAccel;
						}
					}
				}
				else
				{
					for (dInt32 j = 0; j < D_AVX_WORK_GROUP; j++)
					{
						const ndConstraint* const Joint = jointGroup[j];
						if (Joint)
						{
							const dInt32 rowCount = Joint->m_rowCount;
							const dInt32 base = Joint->m_rowStart;
							for (dInt32 k = 0; k < rowCount; k++)
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
	m_firstPassCoef = dFloat32(1.0f);

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
			ndScene* const scene = world->GetScene();
			ndDynamicsUpdateAvx2* const me = (ndDynamicsUpdateAvx2*)world->m_solver;
			const dArray<dInt32>& bodyIslandOrder = me->GetBodyIslandOrder____();
			ndBodyKinematic** const bodyArray = &scene->GetActiveBodyArray()[0];
			const dArray<ndJacobian>& internalForces = me->GetInternalForces();

			const dVector timestep4(me->m_timestepRK);
			const dVector speedFreeze2(world->m_freezeSpeed2 * dFloat32(0.1f));

			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();
			const dInt32 bodyCount = bodyIslandOrder.GetCount() - me->GetUnconstrainedBodyCount();

			const dInt32 stride = bodyCount / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : bodyCount - start;
			
			for (dInt32 i = 0; i < blockSize; i++)
			{
				dInt32 index = bodyIslandOrder[start + i];
				ndBodyDynamic* const body = bodyArray[index]->GetAsBodyDynamic();
				if (body)
				{
					dAssert(body->m_index == index);
					dAssert(body->m_bodyIsConstrained);
					const ndJacobian& forceAndTorque = internalForces[index];
					const dVector force(body->GetForce() + forceAndTorque.m_linear);
					const dVector torque(body->GetTorque() + forceAndTorque.m_angular - body->GetGyroTorque());
					const ndJacobian velocStep(body->IntegrateForceAndToque(force, torque, timestep4));

					if (!body->m_resting)
					{
						body->m_veloc += velocStep.m_linear;
						body->m_omega += velocStep.m_angular;
						body->IntegrateGyroSubstep(timestep4);
					}
					else
					{
						const dVector velocStep2(velocStep.m_linear.DotProduct(velocStep.m_linear));
						const dVector omegaStep2(velocStep.m_angular.DotProduct(velocStep.m_angular));
						const dVector test(((velocStep2 > speedFreeze2) | (omegaStep2 > speedFreeze2)) & dVector::m_negOne);
						const dInt32 equilibrium = test.GetSignMask() ? 0 : 1;
						body->m_resting &= equilibrium;
					}

					dAssert(body->m_veloc.m_w == dFloat32(0.0f));
					dAssert(body->m_omega.m_w == dFloat32(0.0f));
				}
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
			:m_one(dFloat32(1.0f))
			,m_zero(dFloat32 (0.0f))
		{
		}

		dFloat32 JointForce(dInt32 block, ndSoaMatrixElement* const massMatrix)
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
				for (dInt32 i = 0; i < D_AVX_WORK_GROUP; i++)
				{
					const ndConstraint* const joint = jointGroup[i];
					const ndBodyKinematic* const body0 = joint->GetBody0();
					const ndBodyKinematic* const body1 = joint->GetBody1();

					const dInt32 m0 = body0->m_index;
					const dInt32 m1 = body1->m_index;

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
				for (dInt32 i = 0; i < D_AVX_WORK_GROUP; i++)
				{
					const ndConstraint* const joint = jointGroup[i];
					if (joint)
					{
						const ndBodyKinematic* const body0 = joint->GetBody0();
						const ndBodyKinematic* const body1 = joint->GetBody1();
				
						const dInt32 m0 = body0->m_index;
						const dInt32 m1 = body1->m_index;

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
			const dInt32 rowsCount = jointGroup[0]->m_rowCount;
		
			for (dInt32 j = 0; j < rowsCount; j++)
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

			const dFloat32 tol = dFloat32(0.5f);
			const dFloat32 tol2 = tol * tol;

			ndAvxFloat maxAccel(accNorm);
			for (dInt32 k = 0; (k < 4) && (maxAccel.AddHorizontal() > tol2); k++)
			{
				maxAccel = m_zero;
				for (dInt32 j = 0; j < rowsCount; j++)
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

			ndAvxFloat mask(m_mask);
			if ((block + D_AVX_WORK_GROUP) > m_activeCount)
			{
				for (dInt32 i = 0; i < D_AVX_WORK_GROUP; i++)
				{
					const ndConstraint* const joint = jointGroup[i];
					if (joint)
					{
						const ndBodyKinematic* const body0 = joint->GetBody0();
						const ndBodyKinematic* const body1 = joint->GetBody1();
						dAssert(body0);
						dAssert(body1);
						//const dInt32 isSleeping = body0->m_resting & body1->m_resting;
						const dInt32 resting = body0->m_resting & body1->m_resting;
						if (resting)
						{
							mask[i] = dFloat32(0.0f);
						}
					}
					else
					{
						mask[i] = dFloat32(0.0f);
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
			for (dInt32 i = 0; i < rowsCount; i++)
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
			dVector::Transpose4x4(
				force0[0].m_vector8.m_linear, 
				force0[1].m_vector8.m_linear, 
				force0[2].m_vector8.m_linear, 
				force0[3].m_vector8.m_linear,
				forceM0.m_linear.m_x.m_vector8.m_linear, 
				forceM0.m_linear.m_y.m_vector8.m_linear, 
				forceM0.m_linear.m_z.m_vector8.m_linear, dVector::m_zero);
			dVector::Transpose4x4(
				force0[4].m_vector8.m_linear,
				force0[5].m_vector8.m_linear,
				force0[6].m_vector8.m_linear,
				force0[7].m_vector8.m_linear,
				forceM0.m_linear.m_x.m_vector8.m_angular,
				forceM0.m_linear.m_y.m_vector8.m_angular,
				forceM0.m_linear.m_z.m_vector8.m_angular, dVector::m_zero);
			dVector::Transpose4x4(
				force0[0].m_vector8.m_angular,
				force0[1].m_vector8.m_angular,
				force0[2].m_vector8.m_angular,
				force0[3].m_vector8.m_angular,
				forceM0.m_angular.m_x.m_vector8.m_linear,
				forceM0.m_angular.m_y.m_vector8.m_linear,
				forceM0.m_angular.m_z.m_vector8.m_linear, dVector::m_zero);
			dVector::Transpose4x4(
				force0[4].m_vector8.m_angular,
				force0[5].m_vector8.m_angular,
				force0[6].m_vector8.m_angular,
				force0[7].m_vector8.m_angular,
				forceM0.m_angular.m_x.m_vector8.m_angular,
				forceM0.m_angular.m_y.m_vector8.m_angular,
				forceM0.m_angular.m_z.m_vector8.m_angular, dVector::m_zero);

			dVector::Transpose4x4(
				force1[0].m_vector8.m_linear,
				force1[1].m_vector8.m_linear,
				force1[2].m_vector8.m_linear,
				force1[3].m_vector8.m_linear,
				forceM1.m_linear.m_x.m_vector8.m_linear,
				forceM1.m_linear.m_y.m_vector8.m_linear,
				forceM1.m_linear.m_z.m_vector8.m_linear, dVector::m_zero);
			dVector::Transpose4x4(
				force1[4].m_vector8.m_linear,
				force1[5].m_vector8.m_linear,
				force1[6].m_vector8.m_linear,
				force1[7].m_vector8.m_linear,
				forceM1.m_linear.m_x.m_vector8.m_angular,
				forceM1.m_linear.m_y.m_vector8.m_angular,
				forceM1.m_linear.m_z.m_vector8.m_angular, dVector::m_zero);
			dVector::Transpose4x4(
				force1[0].m_vector8.m_angular,
				force1[1].m_vector8.m_angular,
				force1[2].m_vector8.m_angular,
				force1[3].m_vector8.m_angular,
				forceM1.m_angular.m_x.m_vector8.m_linear,
				forceM1.m_angular.m_y.m_vector8.m_linear,
				forceM1.m_angular.m_z.m_vector8.m_linear, dVector::m_zero);
			dVector::Transpose4x4(
				force1[4].m_vector8.m_angular,
				force1[5].m_vector8.m_angular,
				force1[6].m_vector8.m_angular,
				force1[7].m_vector8.m_angular,
				forceM1.m_angular.m_x.m_vector8.m_angular,
				forceM1.m_angular.m_y.m_vector8.m_angular,
				forceM1.m_angular.m_z.m_vector8.m_angular, dVector::m_zero);

			ndRightHandSide* const rightHandSide = &m_rightHandSide[0];
			for (dInt32 i = 0; i < D_AVX_WORK_GROUP; i++)
			{
				const ndConstraint* const joint = jointGroup[i];
				if (joint)
				{
					const ndBodyKinematic* const body0 = joint->GetBody0();
					const ndBodyKinematic* const body1 = joint->GetBody1();

					const dInt32 m0 = body0->m_index;
					const dInt32 m1 = body1->m_index;

					dInt32 const rowCount = joint->m_rowCount;
					dInt32 const rowStartBase = joint->m_rowStart;
					for (dInt32 j = 0; j < rowCount; j++)
					{
						const ndSoaMatrixElement* const row = &massMatrix[j];
						rightHandSide[j + rowStartBase].m_force = row->m_force[i];
						rightHandSide[j + rowStartBase].m_maxImpact = dMax(dAbs(row->m_force[i]), rightHandSide[j + rowStartBase].m_maxImpact);
					}

					const dInt32 index0 = (block + i) * 2 + 0;
					ndAvxFloat& outBody0 = (ndAvxFloat&)m_jointPartialForces[index0];
					outBody0 = force0[i];

					const dInt32 index1 = (block + i) * 2 + 1;
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
			const dInt32* const soaJointRows = &me->m_avxJointRows[0];
			dAvxMatrixArray& soaMassMatrixArray = *me->m_avxMassMatrixArray;
			ndSoaMatrixElement* const soaMassMatrix = &soaMassMatrixArray[0];
			m_jointArray = &jointArray[0];

			dFloat32 accNorm = dFloat32(0.0f);
			m_activeCount = me->m_activeJointCount;
			const dInt32 jointCount = jointArray.GetCount();
			const dInt32 bodyCount = m_owner->GetActiveBodyArray().GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();

			m_mask = ndAvxFloat(dInt32(-1));
			const dInt32 mask = -dInt32(D_AVX_WORK_GROUP);
			const dInt32 soaJointCount = ((jointCount + D_AVX_WORK_GROUP - 1) & mask) / D_AVX_WORK_GROUP;
			for (dInt32 i = threadIndex; i < soaJointCount; i += threadCount)
			{
				accNorm += JointForce(i * D_AVX_WORK_GROUP, &soaMassMatrix[soaJointRows[i]]);
			}

			dFloat32* const accelNorm = (dFloat32*)m_context;
			accelNorm[threadIndex] = accNorm;
		}

		ndAvxFloat m_one;
		ndAvxFloat m_zero;
		ndAvxFloat m_mask;
		ndConstraint** m_jointArray;
		ndJacobian* m_jointPartialForces;
		ndRightHandSide* m_rightHandSide;
		const ndJacobian* m_internalForces;
		const ndLeftHandSide* m_leftHandSide;
		const ndJointBodyPairIndex* m_jointBodyPairIndexBuffer;
		dInt32 m_activeCount;
	};

	class ndApplyJacobianAccumulatePartialForces : public ndScene::ndBaseJob
	{
		public:
		virtual void Execute()
		{
			D_TRACKTIME();
			const dVector zero(dVector::m_zero);
			ndWorld* const world = m_owner->GetWorld();
			ndDynamicsUpdateSoa* const me = (ndDynamicsUpdateSoa*)world->m_solver;

			ndJacobian* const internalForces = &me->GetInternalForces()[0];
			const dInt32* const bodyIndex = &me->GetJointForceIndexBuffer()[0];
			const dArray<ndBodyKinematic*>& bodyArray = m_owner->GetActiveBodyArray();
			const ndJacobian* const jointInternalForces = &me->GetTempInternalForces()[0];
			const ndJointBodyPairIndex* const jointBodyPairIndexBuffer = &me->GetJointBodyPairIndexBuffer()[0];

			const dInt32 bodyCount = bodyArray.GetCount();
			const dInt32 threadIndex = GetThreadId();
			const dInt32 threadCount = m_owner->GetThreadCount();

			const dInt32 stride = bodyCount / threadCount;
			const dInt32 start = threadIndex * stride;
			const dInt32 blockSize = (threadIndex != (threadCount - 1)) ? stride : bodyCount - start;

			for (dInt32 i = 0; i < blockSize; i++)
			{
				dInt32 startIndex = bodyIndex[start + i];
				dInt32 count = bodyIndex[start + i + 1] - startIndex;
				if (count)
				{
					const ndBodyKinematic* const body = bodyArray[i + start];
					if (body->m_invMass.m_w > dFloat32(0.0f))
					{
						dVector force(zero);
						dVector torque(zero);
						for (dInt32 j = 0; j < count; j++)
						{
							dInt32 index = jointBodyPairIndexBuffer[startIndex + j].m_joint;
							force += jointInternalForces[index].m_linear;
							torque += jointInternalForces[index].m_angular;
						}
						internalForces[i + start].m_linear = force;
						internalForces[i + start].m_angular = torque;
					}
				}
			}
		}
	};

	ndScene* const scene = m_world->GetScene();
	const dInt32 passes = m_solverPasses;
	const dInt32 threadsCount = scene->GetThreadCount();

	dFloat32 m_accelNorm[D_MAX_THREADS_COUNT];
	dFloat32 accNorm = D_SOLVER_MAX_ERROR * dFloat32(2.0f);

	for (dInt32 i = 0; (i < passes) && (accNorm > D_SOLVER_MAX_ERROR); i++)
	{
		scene->SubmitJobs<ndCalculateJointsForce>(m_accelNorm);
		scene->SubmitJobs<ndApplyJacobianAccumulatePartialForces>();

		accNorm = dFloat32(0.0f);
		for (dInt32 j = 0; j < threadsCount; j++)
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
		m_firstPassCoef = dFloat32(0.0f);
		if (m_world->m_skeletonList.GetCount())
		{
			InitSkeletons();
		}
		
		for (dInt32 step = 0; step < 4; step++)
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
	dInt32 count = GetIsland____().GetCount();
	if (count)
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
