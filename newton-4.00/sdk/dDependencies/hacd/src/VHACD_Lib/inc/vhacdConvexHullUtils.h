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

#ifndef __ND_CONVEXHULL_3D_UTILS__
#define __ND_CONVEXHULL_3D_UTILS__

#include "vhacdDefines.h"
#include "vhacdVector.h"

#ifndef _ASSERT
#define _ASSERT(x)
#endif

namespace nd
{
	namespace VHACD 
	{
		#define VHACD_WORKERS_THREADS	4
		//#define VHACD_WORKERS_THREADS	1

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

		template <class T>
		inline T Min(T A, T B)
		{
			return (A < B) ? A : B;
		}

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

		template <class dItem, class dKey>
		class ndHeap
		{
			public:
			ndHeap(int maxElements);
			ndHeap(const void* const buffer, int sizeInBytes);
			~ndHeap();

			void Flush();
			dKey MaxValue() const;
			dKey Value(int i = 0) const;
			int GetCount() const;
			int GetMaxCount() const;
			const dItem& operator[] (int i) const;
			int Find(dItem& obj);
			int Find(dKey key);

			void Pop();
			void Sort();
			void Remove(int Index);
			void Push(dItem& obj, dKey key);

			protected:
			struct dRecord
			{
				dRecord()
				{
				}

				dRecord(dKey key, const dItem& obj)
					:m_key(key)
					,m_obj(obj)
				{
				}

				dKey m_key;
				dItem m_obj;
			};

			dRecord* m_pool;
			int m_curCount;
			int m_maxCount;
			bool m_bufferIsOwnned;
		};


		// *************************
		//
		// implementation
		//
		// *************************

		template <class dItem, class dKey>
		ndHeap<dItem, dKey>::ndHeap(int maxElements)
			:m_pool(new dRecord[size_t(maxElements)])
			,m_curCount(0)
			,m_maxCount(maxElements)
			,m_bufferIsOwnned(true)
		{
			Flush();
		}

		template <class dItem, class dKey>
		ndHeap<dItem, dKey>::ndHeap(const void* const buffer, int sizeInBytes)
			:m_pool((dRecord*)buffer)
			,m_curCount(0)
			,m_maxCount(int(sizeInBytes / sizeof(dRecord)))
			,m_bufferIsOwnned(false)
		{
			Flush();
		}

		template <class dItem, class dKey>
		ndHeap<dItem, dKey>::~ndHeap()
		{
			if (m_bufferIsOwnned)
			{
				//ndMemory::Free(m_pool);
				delete[] m_pool;
			}
		}

		template <class dItem, class dKey>
		dKey ndHeap<dItem, dKey>::Value(int i) const
		{
			return m_pool[i].m_key;
		}

		template <class dItem, class dKey>
		int ndHeap<dItem, dKey>::GetCount() const
		{
			return m_curCount;
		}

		template <class dItem, class dKey>
		void ndHeap<dItem, dKey>::Flush()
		{
			m_curCount = 0;
		}

		template <class dItem, class dKey>
		dKey ndHeap<dItem, dKey>::MaxValue() const
		{
			return m_pool[0].m_key;
		}

		template <class dItem, class dKey>
		int ndHeap<dItem, dKey>::GetMaxCount() const
		{
			return m_maxCount;
		}

		template <class dItem, class dKey>
		int ndHeap<dItem, dKey>::Find(dItem& obj)
		{
			// For now let perform a linear search
			// this is efficient if the size of the heap is small
			// ex: m_curCount < 32
			// this will be change to a binary search in the heap should the 
			// the size of the heap get larger than 32
			for (int i = 0; i < m_curCount; i++)
			{
				if (m_pool[i].obj == obj)
				{
					return i;
				}
			}
			return -1;
		}

		template <class dItem, class dKey>
		int ndHeap<dItem, dKey>::Find(dKey key)
		{
			// ex: m_curCount < 32
			// this will be change to a binary search in the heap should the 
			// the size of the heap get larger than 32
			for (int i = 0; i < m_curCount; i++)
			{
				if (m_pool[i].m_key == key)
				{
					return i;
				}
			}
			return -1;
		}

		template <class dItem, class dKey>
		const dItem& ndHeap<dItem, dKey>::operator[] (int i) const
		{
			return m_pool[i].m_obj;
		}

		template <class dItem, class dKey>
		void ndHeap<dItem, dKey>::Push(dItem& obj, dKey key)
		{
			ndHeap<dItem, dKey>::m_curCount++;

			int i = ndHeap<dItem, dKey>::m_curCount;
			for (int j = 0; i; i = j)
			{
				j = i >> 1;
				if (!j || (ndHeap<dItem, dKey>::m_pool[j - 1].m_key > key))
				{
					break;
				}
				ndHeap<dItem, dKey>::m_pool[i - 1] = ndHeap<dItem, dKey>::m_pool[j - 1];
			}
			ndHeap<dItem, dKey>::m_pool[i - 1].m_key = key;
			ndHeap<dItem, dKey>::m_pool[i - 1].m_obj = obj;
		}

		template <class dItem, class dKey>
		void ndHeap<dItem, dKey>::Pop()
		{
			Remove(0);
		}

		template <class dItem, class dKey>
		void ndHeap<dItem, dKey>::Remove(int index)
		{
			ndHeap<dItem, dKey>::m_curCount--;
			ndHeap<dItem, dKey>::m_pool[index] = ndHeap<dItem, dKey>::m_pool[ndHeap<dItem, dKey>::m_curCount];
			while (index && ndHeap<dItem, dKey>::m_pool[(index - 1) >> 1].m_key < ndHeap<dItem, dKey>::m_pool[index].m_key)
			{
				Swap(ndHeap<dItem, dKey>::m_pool[(index - 1) >> 1], ndHeap<dItem, dKey>::m_pool[index]);
				index = (index - 1) >> 1;
			}

			while ((2 * index + 1) < ndHeap<dItem, dKey>::m_curCount)
			{
				int i0 = 2 * index + 1;
				int i1 = 2 * index + 2;
				if (i1 < ndHeap<dItem, dKey>::m_curCount)
				{
					i0 = (ndHeap<dItem, dKey>::m_pool[i0].m_key > ndHeap<dItem, dKey>::m_pool[i1].m_key) ? i0 : i1;
					if (ndHeap<dItem, dKey>::m_pool[i0].m_key <= ndHeap<dItem, dKey>::m_pool[index].m_key)
					{
						break;
					}
					Swap(ndHeap<dItem, dKey>::m_pool[i0], ndHeap<dItem, dKey>::m_pool[index]);
					index = i0;
				}
				else
				{
					if (ndHeap<dItem, dKey>::m_pool[i0].m_key > ndHeap<dItem, dKey>::m_pool[index].m_key)
					{
						Swap(ndHeap<dItem, dKey>::m_pool[i0], ndHeap<dItem, dKey>::m_pool[index]);
					}
					index = i0;
				}
			}
		}

		template <class dItem, class dKey>
		void ndHeap<dItem, dKey>::Sort()
		{
			int count = ndHeap<dItem, dKey>::m_curCount;
			for (int i = 1; i < count; i++)
			{
				dKey key(ndHeap<dItem, dKey>::m_pool[0].m_key);
				dItem obj(ndHeap<dItem, dKey>::m_pool[0].m_obj);

				Pop();

				ndHeap<dItem, dKey>::m_pool[ndHeap<dItem, dKey>::m_curCount].m_key = key;
				ndHeap<dItem, dKey>::m_pool[ndHeap<dItem, dKey>::m_curCount].m_obj = obj;
			}

			ndHeap<dItem, dKey>::m_curCount = count;
			for (int i = 0; i < count / 2; i++)
			{
				dKey key(ndHeap<dItem, dKey>::m_pool[i].m_key);
				dItem obj(ndHeap<dItem, dKey>::m_pool[i].m_obj);

				ndHeap<dItem, dKey>::m_pool[i].m_key = ndHeap<dItem, dKey>::m_pool[count - i - 1].m_key;
				ndHeap<dItem, dKey>::m_pool[i].m_obj = ndHeap<dItem, dKey>::m_pool[count - i - 1].m_obj;

				ndHeap<dItem, dKey>::m_pool[count - i - 1].m_key = key;
				ndHeap<dItem, dKey>::m_pool[count - i - 1].m_obj = obj;
			}
		}

		// *****************************************
		//
		//  two typical instances of heaps, up and down.
		//
		// *****************************************
		template <class dKey>
		class ndDownHeapCompare
		{
			public:
			ndDownHeapCompare()
			{
			}

			ndDownHeapCompare(dKey key)
				:m_key(key)
			{
			}

			bool operator> (const ndDownHeapCompare<dKey>& key) const
			{
				return m_key > key.m_key;
			}

			bool operator< (const ndDownHeapCompare<dKey>& key) const
			{
				return m_key < key.m_key;
			}

			bool operator<= (const ndDownHeapCompare<dKey>& key) const
			{
				return m_key <= key.m_key;
			}

			dKey m_key;
		};

		template <class dItem, class dKey>
		class ndDownHeap : public ndHeap<dItem, ndDownHeapCompare<dKey> >
		{
			public:
			ndDownHeap(int maxElements)
				:ndHeap<dItem, ndDownHeapCompare<dKey>>(maxElements)
			{
			}

			ndDownHeap(const void* const buffer, int sizeInBytes)
				:ndHeap<dItem, ndDownHeapCompare<dKey>>(buffer, sizeInBytes)
			{
			}

			dKey Value(int i = 0) const
			{
				const ndDownHeapCompare<dKey> key(ndHeap<dItem, ndDownHeapCompare<dKey>>::Value(i));
				return key.m_key;
			}
		};


		template <class dKey>
		class ndUpHeapCompare
		{
			public:
			ndUpHeapCompare()
			{
			}

			ndUpHeapCompare(dKey key)
				:m_key(key)
			{
			}

			bool operator> (const ndUpHeapCompare<dKey>& key) const
			{
				return m_key < key.m_key;
			}

			bool operator< (const ndUpHeapCompare<dKey>& key) const
			{
				return m_key > key.m_key;
			}

			bool operator<= (const ndUpHeapCompare<dKey>& key) const
			{
				return m_key >= key.m_key;
			}

			dKey m_key;
		};

		template <class dItem, class dKey>
		class ndUpHeap : public ndHeap<dItem, ndUpHeapCompare<dKey> >
		{
			public:
			ndUpHeap(int maxElements)
				:ndHeap<dItem, ndUpHeapCompare<dKey>>(maxElements)
			{
			}

			ndUpHeap(const void* const buffer, int sizeInBytes)
				:ndHeap<dItem, ndUpHeapCompare<dKey>>(buffer, sizeInBytes)
			{
			}

			dKey Value(int i = 0) const
			{
				const ndUpHeapCompare<dKey> key(ndHeap<dItem, ndUpHeapCompare<dKey>>::Value(i));
				return key.m_key;
			}
		};


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

		//class hullVector : public VHACD::Vec3<double>
		//{
		//	public:
		//	hullVector()
		//		:Vec3<double>(0, 0, 0)
		//	{
		//	}
		//
		//	hullVector(double x)
		//		:Vec3<double>(x, x, x)
		//	{
		//	}
		//
		//	hullVector(const hullVector& x)
		//		:Vec3<double>(x.X(), x.Y(), x.Z())
		//	{
		//	}
		//
		//	hullVector(double x, double y, double z, double)
		//		:Vec3<double>(x, y, z)
		//	{
		//	}
		//
		//	hullVector GetMin(const hullVector& p) const
		//	{
		//		return hullVector(
		//			X() < p.X() ? X() : p.X(),
		//			Y() < p.Y() ? Y() : p.Y(),
		//			Z() < p.Z() ? Z() : p.Z(), 0.0);
		//	}
		//
		//	hullVector GetMax(const hullVector& p) const
		//	{
		//		return hullVector(
		//			X() > p.X() ? X() : p.X(),
		//			Y() > p.Y() ? Y() : p.Y(),
		//			Z() > p.Z() ? Z() : p.Z(), 0.0);
		//	}
		//
		//	hullVector Scale(double s) const
		//	{
		//		return hullVector(X() * s, Y() * s, Z() * s, 0.0);
		//	}
		//
		//	inline hullVector operator+(const hullVector & rhs) const
		//	{
		//		return hullVector(X() + rhs.X(), Y() + rhs.Y(), Z() + rhs.Z(), 0.0f);
		//	}
		//
		//	inline hullVector operator-(const hullVector & rhs) const
		//	{
		//		return hullVector(X() - rhs.X(), Y() - rhs.Y(), Z() - rhs.Z(), 0.0f);
		//	}
		//
		//	inline hullVector operator*(const hullVector & rhs) const
		//	{
		//		return hullVector(X() * rhs.X(), Y() * rhs.Y(), Z() * rhs.Z(), 0.0f);
		//	}
		//
		//	inline double DotProduct(const hullVector & rhs) const
		//	{
		//		return X() * rhs.X() + Y() * rhs.Y() + Z() * rhs.Z();
		//	}
		//
		//	inline hullVector CrossProduct(const hullVector & rhs) const
		//	{
		//		return hullVector(Y() * rhs.Z() - Z() * rhs.Y(), Z() * rhs.X() - X() * rhs.Z(), X() * rhs.Y() - Y() * rhs.X(), 0.0);
		//	}
		//
		//	inline hullVector operator= (const Vec3 & rhs)
		//	{
		//		X() = rhs.X();
		//		Y() = rhs.Y();
		//		Z() = rhs.Z();
		//		return *this;
		//	}
		//
		//	inline hullVector& operator= (const hullVector& rhs)
		//	{
		//		X() = rhs.X();
		//		Y() = rhs.Y();
		//		Z() = rhs.Z();
		//		return *this;
		//	}
		//};
		//
		//class hullPlane : public hullVector
		//{
		//	public:
		//	hullPlane(const hullPlane& src)
		//		:hullVector(src)
		//		,m_w(src.m_w)
		//	{
		//	}
		//
		//	hullPlane(double x, double y, double z, double w)
		//		:hullVector(x, y, z, 0.0)
		//		, m_w(w)
		//	{
		//	}
		//
		//	hullPlane(const hullVector &P0, const hullVector &P1, const hullVector &P2)
		//		:hullVector((P1 - P0).CrossProduct(P2 - P0))
		//	{
		//		m_w = -DotProduct(P0);
		//	}
		//
		//	hullPlane Scale(double s) const
		//	{
		//		return hullPlane(X() * s, Y() * s, Z() * s, m_w * s);
		//	}
		//
		//	inline hullPlane operator= (const hullPlane &rhs)
		//	{
		//		X() = rhs.X();
		//		Y() = rhs.Y();
		//		Z() = rhs.Z();
		//		m_w = rhs.m_w;
		//		return *this;
		//	}
		//
		//	inline hullVector operator*(const hullVector & rhs) const
		//	{
		//		return hullVector(X() * rhs.X(), Y() * rhs.Y(), Z() * rhs.Z(), 0.0f);
		//	}
		//
		//	double Evalue(const hullVector &point) const
		//	{
		//		return DotProduct(point) + m_w;
		//	}
		//
		//	double m_w;
		//};

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

			Job(const Job&)
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
