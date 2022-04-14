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

#ifndef __CU_SORT_H__
#define __CU_SORT_H__

#include <cuda.h>
#include <vector_types.h>
#include <cuda_runtime.h>
#include <ndNewtonStdafx.h>

class cuSceneInfo;

class cuAabbGridHash
{
	public:
	union
	{
		int4 m_key;
		char m_bytes[4 * 3];
		struct
		{
			int m_x;
			int m_y;
			int m_z;
			int m_id;
		};
	};
};


class CudaCountingSort
{
	public:
	CudaCountingSort(cuSceneInfo* info, int* histogram, int size, cudaStream_t stream);
	void Sort(cuAabbGridHash* const src, cuAabbGridHash* const dst);

	// this should be private but Cuda does not let private of protected lamddas 
	// to be passed as arguments to kerners.
	public:
	void Sort(const cuAabbGridHash* const src, cuAabbGridHash* const dst, int digit);

	bool SanityCheck(const cuAabbGridHash* const src);

	cuSceneInfo* m_info;
	int* m_histogram;
	cudaStream_t m_stream;
	int m_size;
	int m_blocks;
};

#endif