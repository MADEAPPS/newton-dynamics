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

#ifndef __ND_STL_ALLOCATOR_H__
#define __ND_STL_ALLOCATOR_H__

void* ndSyclMalloc(size_t size);
void ndSyclFree(void* const ptr);

template <class T>
class StlAllocator
{
	public:
	// type definitions
	typedef T        value_type;

	// rebind allocator to type U
	template <class U>
	struct rebind 
	{
		typedef StlAllocator<U> other;
	};

	// return address of values
	T* address(T& value) const
	{
		return &value;
	}

	const T* address(const T& value) const
	{
		return &value;
	}

	StlAllocator() throw()
	{
	}

	StlAllocator(const StlAllocator&) throw()
	{
	}

	template <class U>
	StlAllocator(const StlAllocator<U>&) throw()
	{
	}

	~StlAllocator() throw()
	{
	}

	// return maximum number of elements that can be allocated
	std::size_t max_size() const throw()
	{
		std::size_t size = 1024 * 1024 * 64;
		return size;
	}

	// initialize elements of allocated storage p with value value
	void construct(T* p, const T& value)
	{
		// initialize memory with placement new
		new((void*)p)T(value);
	}

	// destroy elements of initialized storage p
	void destroy(T* p)
	{
		// destroy objects by calling their destructor
		p->~T();
	}

	// allocate but don't initialize num elements of type T
	T* allocate(std::size_t num, const void* = 0)
	{
		T* const ptr = (T*)ndSyclMalloc(num * sizeof (T));
		return ptr;
	}

	// deallocate storage p of deleted elements
	void deallocate(T* ptr, std::size_t num)
	{
		ndSyclFree(ptr);
	}
};

template <class T>
class StlVector : public std::vector<T, StlAllocator<T> >
{
};

#endif