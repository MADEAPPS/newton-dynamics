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

#ifndef __ND_TRHEAD_SYNC_UTILS_H__
#define __ND_TRHEAD_SYNC_UTILS_H__

#include "ndCoreStdafx.h"
#include "ndTypes.h"

/// tell the operating system the library is done with this job, 
/// the OS is free to switch to another task if it needs to.
D_CORE_API void ndThreadYield();

/// in some systems, thread yield could be too expensive 
/// since it could task switch to another thread. 
D_CORE_API void ndThreadPause();

/// Set cpu floating point exceptions, the original exception state is restored when the destructor is called.
class ndFloatExceptions
{
	public:
	//#if defined (WIN32) || defined(_WIN32)
	#if defined (_MSC_VER)
		#define D_FLOAT_EXCEPTIONS_MASK	(_EM_ZERODIVIDE | _EM_INVALID | _EM_DENORMAL)
	#else
		#define D_FLOAT_EXCEPTIONS_MASK	(FE_DIVBYZERO | FE_INVALID | FE_INEXACT)
	#endif

	D_CORE_API ndFloatExceptions(ndUnsigned32 mask = D_FLOAT_EXCEPTIONS_MASK);
	D_CORE_API ~ndFloatExceptions();

	private:
	ndUnsigned32 m_floatMask;
	ndUnsigned32 m_simdMask;
};

#ifdef D_USE_THREAD_EMULATION
	/// wrapper over standard atomic operations
	template<class T>
	class ndAtomic
	{
		public:
		ndAtomic<T>()
			:m_val(T(0))
		{
		}

		ndAtomic<T>(T val)
			:m_val(val)
		{
		}

		operator T() const
		{
			return m_val;
		}

		T operator++(int)
		{
			return fetch_add(1);
		}

		T load() const
		{
			return m_val;
		}

		void store(T val)
		{
			m_val = val;
		}

		T exchange(T val)
		{
			ndSwap(val, m_val);
			return val;
		}

		T fetch_add(T val)
		{
			T ret = m_val;
			m_val += val;
			return ret;
		}

		T fetch_sub(T val)
		{
			T ret = m_val;
			m_val -= val;
			return ret;
		}

		bool compare_exchange_weak(T oldValue, T newValue)
		{
			if (m_val == oldValue)
			{
				m_val = newValue;
				return true;
			}
			return false;
		}

		private:
		T m_val;
	};
#else
	/// wrapper over standard atomic operations
	template<class T>
	class ndAtomic : public std::atomic<T>
	{
		public:
		using std::atomic<T>::atomic;
		using std::atomic<T>::operator=;
	};
#endif

/// Simple spin lock for synchronizing threads for very short period of time.
class ndSpinLock
{
	public:
	ndSpinLock()
		#ifndef D_USE_THREAD_EMULATION	
		:m_lock(0)
		#endif
	{
	}

	#ifndef D_USE_THREAD_EMULATION
	ndSpinLock(ndSpinLock const& other)
		:m_lock(other.m_lock.load())
	#else
		ndSpinLock(ndSpinLock const&)
	#endif
	{
	}

	void Lock()
	{
		#ifndef D_USE_THREAD_EMULATION	
		ndInt32 exp = 1;
		for (ndUnsigned32 test = 0; !m_lock.compare_exchange_weak(test, 1); test = 0)
		{
			Delay(exp);
		}
		#endif
	}

	void Unlock()
	{
		#ifndef D_USE_THREAD_EMULATION	
			m_lock.store(0);
		#endif
	}

	#ifndef D_USE_THREAD_EMULATION	
	private:
	D_CORE_API void Delay(ndInt32& exp);
	ndAtomic<ndUnsigned32> m_lock;
	#endif
};

/// Simple scope based spin lock.
class ndScopeSpinLock
{
	public:
	ndScopeSpinLock(ndSpinLock& spinLock)
		:m_spinLock(spinLock)
	{
		m_spinLock.Lock();
	}

	~ndScopeSpinLock()
	{
		m_spinLock.Unlock();
	}
	private:
	ndSpinLock& m_spinLock;
};

class ndReadWriteSpinLock
{
	public:
	ndReadWriteSpinLock()
		:m_readLock()
		,m_writeLock()
		,m_writeActive(0)
	{
	}

	ndReadWriteSpinLock(const ndReadWriteSpinLock& other)
		:m_readLock(other.m_readLock)
		,m_writeLock(other.m_writeLock)
		,m_writeActive(other.m_writeActive)
	{
	}

	void ReadLock()
	{
		m_readLock.Lock();
		ndAssert(m_writeActive >= 0);
		m_writeActive++;
		if (m_writeActive == 1)
		{
			m_writeLock.Lock();
		}
		m_readLock.Unlock();
	}

	void ReadUnlock()
	{
		m_readLock.Lock();
		m_writeActive--;
		ndAssert(m_writeActive >= 0);
		if (m_writeActive == 0)
		{
			m_writeLock.Unlock();
		}
		m_readLock.Unlock();
	}

	void WriteLock()
	{
		m_writeLock.Lock();
	}

	void WriteUnlock()
	{
		m_writeLock.Unlock();
	}

	private:
	ndSpinLock m_readLock;
	ndSpinLock m_writeLock;
	ndInt32 m_writeActive;
};

/// Simple scope based spin lock.
class ndScopeReadSpinLock
{
	public:
	ndScopeReadSpinLock(ndReadWriteSpinLock& spinLock)
		:m_spinLock(spinLock)
	{
		m_spinLock.ReadLock();
	}

	~ndScopeReadSpinLock()
	{
		m_spinLock.ReadUnlock();
	}
	private:
	ndReadWriteSpinLock& m_spinLock;
};

/// Simple scope based spin lock.
class ndScopeWriteSpinLock
{
	public:
	ndScopeWriteSpinLock(ndReadWriteSpinLock& spinLock)
		:m_spinLock(spinLock)
	{
		m_spinLock.WriteLock();
	}

	~ndScopeWriteSpinLock()
	{
		m_spinLock.WriteUnlock();
	}
	private:
	ndReadWriteSpinLock& m_spinLock;
};

#endif

