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

#ifndef __ND_CONVEXHULL_3D_UTILS__
#define __ND_CONVEXHULL_3D_UTILS__

#include <mutex>
#include <vector>
#include <atomic>
#include <thread>
#include <condition_variable>
#include "vhacdVector.h"

#ifndef _ASSERT
	#define _ASSERT(x)
#endif

namespace nd
{
	namespace VHACD 
	{
		#define VHACD_GOOGOL_SIZE		4
		#define VHACD_WORKERS_THREADS	4

		class Googol;
		Googol Determinant2x2(const Googol matrix[2][2]);
		Googol Determinant3x3(const Googol matrix[3][3]);
		double Determinant2x2(const double matrix[2][2], double* const error);
		double Determinant3x3(const double matrix[3][3], double* const error);

		inline int dExp2(int x)
		{
			int exp;
			for (exp = -1; x; x >>= 1)
			{
				exp++;
			}
			return exp;
		}

		inline int dBitReversal(int v, int base)
		{
			int x = 0;
			int power = dExp2(base) - 1;
			do
			{
				x += (v & 1) << power;
				v >>= 1;
				power--;
			} while (v);
			return x;
		}

		template<class T>
		class List
		{
			public:
			class ndNode
			{
				ndNode(ndNode* const prev, ndNode* const next)
					:m_info()
					, m_next(next)
					, m_prev(prev)
				{
					if (m_prev)
					{
						m_prev->m_next = this;
					}
					if (m_next)
					{
						m_next->m_prev = this;
					}
				}

				ndNode(const T &info, ndNode* const prev, ndNode* const next)
					:m_info(info)
					,m_next(next)
					,m_prev(prev)
				{
					if (m_prev)
					{
						m_prev->m_next = this;
					}
					if (m_next)
					{
						m_next->m_prev = this;
					}
				}

				~ndNode()
				{
				}

				void Unlink()
				{
					if (m_prev)
					{
						m_prev->m_next = m_next;
					}
					if (m_next)
					{
						m_next->m_prev = m_prev;
					}
					m_prev = nullptr;
					m_next = nullptr;
				}

				void AddLast(ndNode* const node)
				{
					m_next = node;
					node->m_prev = this;
				}

				void AddFirst(ndNode* const node)
				{
					m_prev = node;
					node->m_next = this;
				}

				public:
				T& GetInfo()
				{
					return m_info;
				}

				ndNode *GetNext() const
				{
					return m_next;
				}

				ndNode *GetPrev() const
				{
					return m_prev;
				}

				private:
				T m_info;
				ndNode *m_next;
				ndNode *m_prev;
				friend class List<T>;
			};

			public:
			List()
				:m_first(nullptr)
				, m_last(nullptr)
				, m_count(0)
			{
			}

			~List()
			{
				RemoveAll();
			}

			void RemoveAll()
			{
				for (ndNode *node = m_first; node; node = m_first)
				{
					m_count--;
					m_first = node->GetNext();
					node->Unlink();
					delete node;
				}
				m_last = nullptr;
				m_first = nullptr;
			}

			ndNode* Append()
			{
				m_count++;
				if (m_first == nullptr)
				{
					m_first = new ndNode(nullptr, nullptr);
					m_last = m_first;
				}
				else
				{
					m_last = new ndNode(m_last, nullptr);
				}
				return m_last;
			}

			ndNode* Append(const T &element)
			{
				m_count++;
				if (m_first == nullptr)
				{
					m_first = new ndNode(element, nullptr, nullptr);
					m_last = m_first;
				}
				else
				{
					m_last = new ndNode(element, m_last, nullptr);
				}
				return m_last;
			}

			ndNode* Addtop(const T &element)
			{
				m_count++;
				if (m_last == nullptr)
				{
					m_last = new ndNode(element, nullptr, nullptr);
					m_first = m_last;
				}
				else
				{
					m_first = new ndNode(element, nullptr, m_first);
				}
				return m_first;
			}

			int GetCount() const
			{
				return m_count;
			}

			//operator int() const;

			ndNode* GetLast() const
			{
				return m_last;
			}

			ndNode* GetFirst() const
			{
				return m_first;
			}

			void Remove(ndNode* const node)
			{
				Unlink(node);
				delete node;
			}

			void Unlink(ndNode* const node)
			{
				m_count--;
				if (node == m_first)
				{
					m_first = m_first->GetNext();
				}
				if (node == m_last)
				{
					m_last = m_last->GetPrev();
				}
				node->Unlink();
			}

			void Remove(const T &element)
			{
				ndNode *const node = Find(element);
				if (node)
				{
					Remove(node);
				}
			}

			ndNode* Find(const T &element) const
			{
				ndNode *node;
				for (node = m_first; node; node = node->GetNext())
				{
					if (element == node->m_info)
					{
						break;
					}
				}
				return node;
			}

			private:
			ndNode* m_first;
			ndNode* m_last;
			int m_count;
			friend class ndNode;
		};

		class hullVector : public VHACD::Vec3<double>
		{
			public:
			hullVector()
				:Vec3<double>(0, 0, 0)
			{
			}

			hullVector(double x)
				:Vec3<double>(x, x, x)
			{
			}

			hullVector(const hullVector& x)
				:Vec3<double>(x.X(), x.Y(), x.Z())
			{
			}

			hullVector(double x, double y, double z, double)
				:Vec3<double>(x, y, z)
			{
			}

			hullVector GetMin(const hullVector& p) const
			{
				return hullVector(
					X() < p.X() ? X() : p.X(),
					Y() < p.Y() ? Y() : p.Y(),
					Z() < p.Z() ? Z() : p.Z(), 0.0);
			}

			hullVector GetMax(const hullVector& p) const
			{
				return hullVector(
					X() > p.X() ? X() : p.X(),
					Y() > p.Y() ? Y() : p.Y(),
					Z() > p.Z() ? Z() : p.Z(), 0.0);
			}

			hullVector Scale(double s) const
			{
				return hullVector(X() * s, Y() * s, Z() * s, 0.0);
			}

			inline hullVector operator+(const hullVector & rhs) const
			{
				return hullVector(X() + rhs.X(), Y() + rhs.Y(), Z() + rhs.Z(), 0.0f);
			}

			inline hullVector operator-(const hullVector & rhs) const
			{
				return hullVector(X() - rhs.X(), Y() - rhs.Y(), Z() - rhs.Z(), 0.0f);
			}

			inline hullVector operator*(const hullVector & rhs) const
			{
				return hullVector(X() * rhs.X(), Y() * rhs.Y(), Z() * rhs.Z(), 0.0f);
			}

			inline double DotProduct(const hullVector & rhs) const
			{
				return X() * rhs.X() + Y() * rhs.Y() + Z() * rhs.Z();
			}

			inline hullVector CrossProduct(const hullVector & rhs) const
			{
				return hullVector(Y() * rhs.Z() - Z() * rhs.Y(), Z() * rhs.X() - X() * rhs.Z(), X() * rhs.Y() - Y() * rhs.X(), 0.0);
			}

			inline hullVector operator= (const Vec3 & rhs)
			{
				X() = rhs.X();
				Y() = rhs.Y();
				Z() = rhs.Z();
				return *this;
			}
		};

		class hullPlane : public hullVector
		{
			public:
			hullPlane(double x, double y, double z, double w)
				:hullVector(x, y, z, 0.0)
				, m_w(w)
			{
			}

			hullPlane(const hullVector &P0, const hullVector &P1, const hullVector &P2)
				:hullVector((P1 - P0).CrossProduct(P2 - P0))
			{
				m_w = -DotProduct(P0);
			}

			hullPlane Scale(double s) const
			{
				return hullPlane(X() * s, Y() * s, Z() * s, m_w * s);
			}

			inline hullPlane operator= (const hullPlane &rhs)
			{
				X() = rhs.X();
				Y() = rhs.Y();
				Z() = rhs.Z();
				m_w = rhs.m_w;
				return *this;
			}

			inline hullVector operator*(const hullVector & rhs) const
			{
				return hullVector(X() * rhs.X(), Y() * rhs.Y(), Z() * rhs.Z(), 0.0f);
			}

			double Evalue(const hullVector &point) const
			{
				return DotProduct(point) + m_w;
			}

			double m_w;
		};

		class Googol
		{
			public:
			Googol(void);
			Googol(double value);

			operator double() const;
			Googol operator+ (const Googol &A) const;
			Googol operator- (const Googol &A) const;
			Googol operator* (const Googol &A) const;
			Googol operator/ (const Googol &A) const;

			Googol operator+= (const Googol &A);
			Googol operator-= (const Googol &A);

			bool operator> (const Googol &A) const;
			bool operator>= (const Googol &A) const;
			bool operator< (const Googol &A) const;
			bool operator<= (const Googol &A) const;
			bool operator== (const Googol &A) const;
			bool operator!= (const Googol &A) const;

			Googol Abs() const;
			Googol Sqrt() const;
			Googol InvSqrt() const;
			Googol Floor() const;

			void Trace() const;
			void ToString(char* const string) const;

			private:
			void InitFloatFloat(double value);
			void NegateMantissa(uint64_t* const mantissa) const;
			void CopySignedMantissa(uint64_t* const mantissa) const;
			int NormalizeMantissa(uint64_t* const mantissa) const;
			uint64_t CheckCarrier(uint64_t a, uint64_t b) const;
			void ShiftRightMantissa(uint64_t* const mantissa, int bits) const;

			int LeadingZeros(uint64_t a) const;
			void ExtendeMultiply(uint64_t a, uint64_t b, uint64_t& high, uint64_t& low) const;
			void ScaleMantissa(uint64_t* const out, uint64_t scale) const;

			int m_sign;
			int m_exponent;
			uint64_t m_mantissa[VHACD_GOOGOL_SIZE];

			public:
			static Googol m_zero;
			static Googol m_one;
			static Googol m_two;
			static Googol m_three;
			static Googol m_half;
		};

		template <class T>
		inline T Max(T A, T B)
		{
			return (A > B) ? A : B;
		}

		template <class T>
		inline void Swap(T& A, T& B)
		{
			T tmp(A);
			A = B;
			B = tmp;
		}

		template <class T, class dCompareKey>
		void Sort(T* const array, int elements)
		{
			const int batchSize = 8;
			int stack[1024][2];

			stack[0][0] = 0;
			stack[0][1] = elements - 1;
			int stackIndex = 1;
			const dCompareKey comparator;
			while (stackIndex)
			{
				stackIndex--;
				int lo = stack[stackIndex][0];
				int hi = stack[stackIndex][1];
				if ((hi - lo) > batchSize)
				{
					int mid = (lo + hi) >> 1;
					if (comparator.Compare(array[lo], array[mid]) > 0)
					{
						Swap(array[lo], array[mid]);
					}
					if (comparator.Compare(array[mid], array[hi]) > 0)
					{
						Swap(array[mid], array[hi]);
					}
					if (comparator.Compare(array[lo], array[mid]) > 0)
					{
						Swap(array[lo], array[mid]);
					}
					int i = lo + 1;
					int j = hi - 1;
					const T pivot(array[mid]);
					do
					{
						while (comparator.Compare(array[i], pivot) < 0)
						{
							i++;
						}
						while (comparator.Compare(array[j], pivot) > 0)
						{
							j--;
						}

						if (i <= j)
						{
							Swap(array[i], array[j]);
							i++;
							j--;
						}
					} while (i <= j);

					if (i < hi)
					{
						stack[stackIndex][0] = i;
						stack[stackIndex][1] = hi;
						stackIndex++;
					}
					if (lo < j)
					{
						stack[stackIndex][0] = lo;
						stack[stackIndex][1] = j;
						stackIndex++;
					}
					_ASSERT(stackIndex < int(sizeof(stack) / (2 * sizeof(stack[0][0]))));
				}
			}

			int stride = batchSize + 1;
			if (elements < stride)
			{
				stride = elements;
			}
			for (int i = 1; i < stride; ++i)
			{
				if (comparator.Compare(array[0], array[i]) > 0)
				{
					Swap(array[0], array[i]);
				}
			}

			for (int i = 1; i < elements; ++i)
			{
				int j = i;
				const T tmp(array[i]);
				for (; comparator.Compare(array[j - 1], tmp) > 0; --j)
				{
					_ASSERT(j > 0);
					array[j] = array[j - 1];
				}
				array[j] = tmp;
			}

			#if 0
			for (int i = 0; i < (elements - 1); ++i)
			{
				_ASSERT(comparator.Compare(array[i], array[i + 1], context) <= 0);
			}
			#endif
		}

		class Job
		{
			public:
			Job()
			{
			}

			virtual ~Job()
			{
			}

			virtual void Execute(int threadId) = 0;
		};

		class Semaphore
		{
		public:
			Semaphore();
			~Semaphore();
			bool Wait();
			void Signal();
			void Terminate();

			int m_count;
			std::mutex m_mutex;
			std::condition_variable m_condition;
			std::atomic<bool> m_terminate;
		};

		class Queue;
		class Thread : public Semaphore, public std::thread
		{
			public:
			Thread();
			~Thread();
			void ThreadFunctionCallback();

			int m_threadID;
			Queue* m_queue;
		};

		class Queue : public List<Job*>
		{
			public:
			Queue();
			~Queue();

			void Sync();
			Job* PopTask();
			void PushTask(Job* const job);

			std::mutex m_mutex;
			std::atomic<int> m_jobs;
			Thread m_threads[VHACD_WORKERS_THREADS];
		};
	}
}
#endif
