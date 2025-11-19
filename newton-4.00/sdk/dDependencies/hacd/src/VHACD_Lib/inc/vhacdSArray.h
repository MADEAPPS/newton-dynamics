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
		template <typename T>
		class SArray : public ndArray<T>
		{
			public:

			void operator=(const SArray& rhs)
			{
				ndArray<T>::SetCount(0);
				for (ndInt32 i = 0; i < ndInt32 (rhs.GetCount()); ++i)
				{
					ndArray<T>::PushBack(rhs[i]);
				}
			}

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
				return (size_t)ndArray<T>::GetCount();
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
				ndArray<T>::SetCount(0);
			}

			void Resize(size_t size)
			{
				ndArray<T>::Resize(ndInt64(size));
			}

			void Allocate(size_t size)
			{
				Resize(size);
				ndArray<T>::SetCount(0);
			}
		};
	}
}
#endif