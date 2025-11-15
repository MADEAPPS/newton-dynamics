/* Copyright (c) 2011 Khaled Mamou (kmamou at gmail dot com)
 All rights reserved.
 
 
 Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 
 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 
 3. The names of the contributors may not be used to endorse or promote products derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ND_VHACD_SARRAY_H
#define ND_VHACD_SARRAY_H

namespace nd
{
	namespace VHACD 
	{
		//!    SArray.
#if 0 
		template <typename T>
		class SArray : public ndArray<T>
		{
			public:

			T& operator[](ndInt32 i)
			{
				return ndArray<T>::operator[](i);
			}
			const T& operator[](ndInt32 i) const
			{
				return ndArray<T>::operator[](ndInt32(i));
			}

			T& operator[](size_t i)
			{
				return ndArray<T>::operator[](ndInt32(i));
			}
			const T& operator[](size_t i) const
			{
				return ndArray<T>::operator[](ndInt32(i));
			}

			size_t Size() const
			{
				return (size_t)GetCount();
			}

			T* Data()
			{
				return &ndArray<T>::operator[](0);
			}
			const T* Data() const
			{
				return &ndArray<T>::operator[](0);
			}

			void Clear()
			{
				SetCount(0);
			}

			void Resize(size_t size)
			{
				ndArray<T>::Resize(ndInt64(size));
			}

			void Allocate(size_t size)
			{
				SetCount(ndInt32 (size));
			}
		};
	}

#else

	#define SARRAY_DEFAULT_MIN_SIZE 16
		template <typename T, size_t N = 64>
		class SArray 
		{
			public:
			T& operator[](ndInt32 i)
			{
				T* const data = Data();
				return data[i];
			}
			const T& operator[](ndInt32 i) const
			{
				const T* const data = Data();
				return data[i];
			}

			T& operator[](size_t i)
			{
				T* const data = Data();
				return data[i];
			}
			const T& operator[](size_t i) const
			{
				const T* const data = Data();
				return data[i];
			}
			size_t Size() const
			{
				return m_size;
			}
			T* Data()
			{
				return (m_maxSize == N) ? m_data0 : m_data;
			}
			const T* Data() const
			{
				return (m_maxSize == N) ? m_data0 : m_data;
			}
			void Clear()
			{
				m_size = 0;
				delete[] m_data;
				m_data = 0;
				m_maxSize = N;
			}
			void PopBack()
			{
				--m_size;
			}
			void Allocate(size_t size)
			{
				if (size > m_maxSize) 
				{
					T* temp = new T[size];
					memcpy(temp, Data(), m_size * sizeof(T));
					delete[] m_data;
					m_data = temp;
					m_maxSize = size;
				}
			}
			void Resize(size_t size)
			{
				Allocate(size);
				m_size = size;
			}
			void SetCount(size_t size)
			{
				Resize(size);
			}

			void PushBack(const T& value)
			{
				if (m_size == m_maxSize) 
				{
					size_t maxSize = (m_maxSize << 1);
					T* temp = new T[maxSize];
					memcpy(temp, Data(), m_maxSize * sizeof(T));
					delete[] m_data;
					m_data = temp;
					m_maxSize = maxSize;
				}
				T* const data = Data();
				data[m_size++] = value;
			}
			bool Find(const T& value, size_t& pos)
			{
				T* const data = Data();
				for (pos = 0; pos < m_size; ++pos)
					if (value == data[pos])
						return true;
				return false;
			}
			bool Insert(const T& value)
			{
				size_t pos;
				if (Find(value, pos))
					return false;
				PushBack(value);
				return true;
			}
			bool Erase(const T& value)
			{
				size_t pos;
				T* const data = Data();
				if (Find(value, pos)) 
				{
					for (size_t j = pos + 1; j < m_size; ++j)
						data[j - 1] = data[j];
					--m_size;
					return true;
				}
				return false;
			}
			void operator=(const SArray& rhs)
			{
				if (m_maxSize < rhs.m_size) 
				{
					delete[] m_data;
					m_maxSize = rhs.m_maxSize;
					m_data = new T[m_maxSize];
				}
				m_size = rhs.m_size;
				memcpy(Data(), rhs.Data(), m_size * sizeof(T));
			}
			void Initialize()
			{
				m_data = 0;
				m_size = 0;
				m_maxSize = N;
			}
			SArray(const SArray& rhs)
			{
				m_data = 0;
				m_size = 0;
				m_maxSize = N;
				*this = rhs;
			}
			SArray()
			{
				Initialize();
			}
			~SArray()
			{
				delete[] m_data;
			}

			private:
			T m_data0[N];
			T* m_data;
			size_t m_size;
			size_t m_maxSize;
		};
#endif
	}
}
#endif