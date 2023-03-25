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

#include "ndCudaStdafx.h"
#include "ndCudaUtils.h"
#include "ndCudaSort.cuh"
#include "ndCudaDevice.h"
#include "ndCudaContext.h"
#include "ndCudaPrefixScan.cuh"
#include "ndCudaSortUnOrdered.cuh"
#include "ndCudaContextImplement.h"
#include "ndCudaContextImplementInternal.cuh"

#define	D_PROFILE_KERNELS
#define D_USE_EVENT_FOR_SYNC

#if 0
#define MAX_BLOCK_SZ 128
#define NUM_BANKS 32
#define LOG_NUM_BANKS 5

#include <iostream>
#include <iomanip>
#define checkCudaErrors(val) check( (val), #val, __FILE__, __LINE__)
template<typename T>
void check(T err, const char* const func, const char* const file, const int line) {
	if (err != cudaSuccess) {
		std::cerr << "CUDA error at: " << file << ":" << line << std::endl;
		std::cerr << cudaGetErrorString(err) << " " << func << std::endl;
		exit(1);
	}
}

//#define ZERO_BANK_CONFLICTS

#ifdef ZERO_BANK_CONFLICTS
#define CONFLICT_FREE_OFFSET(n) \
    ((n) >> NUM_BANKS + (n) >> (2 * LOG_NUM_BANKS))
#else
#define CONFLICT_FREE_OFFSET(n) ((n) >> LOG_NUM_BANKS)
#endif

__global__
void gpu_add_block_sums(unsigned int* const d_out,
	const unsigned int* const d_in,
	unsigned int* const d_block_sums,
	const size_t numElems)
{
	//unsigned int glbl_t_idx = blockDim.x * blockIdx.x + threadIdx.x;
	unsigned int d_block_sum_val = d_block_sums[blockIdx.x];

	//unsigned int d_in_val_0 = 0;
	//unsigned int d_in_val_1 = 0;

	// Simple implementation's performance is not significantly (if at all)
	//  better than previous verbose implementation
	unsigned int cpy_idx = 2 * blockIdx.x * blockDim.x + threadIdx.x;
	if (cpy_idx < numElems)
	{
		d_out[cpy_idx] = d_in[cpy_idx] + d_block_sum_val;
		if (cpy_idx + blockDim.x < numElems)
			d_out[cpy_idx + blockDim.x] = d_in[cpy_idx + blockDim.x] + d_block_sum_val;
	}

	//if (2 * glbl_t_idx < numElems)
	//{
	//    d_out[2 * glbl_t_idx] = d_in[2 * glbl_t_idx] + d_block_sum_val;
	//    if (2 * glbl_t_idx + 1 < numElems)
	//        d_out[2 * glbl_t_idx + 1] = d_in[2 * glbl_t_idx + 1] + d_block_sum_val;
	//}

	//if (2 * glbl_t_idx < numElems)
	//{
	//    d_in_val_0 = d_in[2 * glbl_t_idx];
	//    if (2 * glbl_t_idx + 1 < numElems)
	//        d_in_val_1 = d_in[2 * glbl_t_idx + 1];
	//}
	//else
	//    return;
	//__syncthreads();

	//d_out[2 * glbl_t_idx] = d_in_val_0 + d_block_sum_val;
	//if (2 * glbl_t_idx + 1 < numElems)
	//    d_out[2 * glbl_t_idx + 1] = d_in_val_1 + d_block_sum_val;
}

// Modified version of Mark Harris' implementation of the Blelloch scan
//  according to https://www.mimuw.edu.pl/~ps209291/kgkp/slides/scan.pdf
__global__
void gpu_prescan(unsigned int* const d_out,
	const unsigned int* const d_in,
	unsigned int* const d_block_sums,
	const unsigned int len,
	const unsigned int shmem_sz,
	const unsigned int max_elems_per_block)
{
	// Allocated on invocation
	extern __shared__ unsigned int s_out[];

	int thid = threadIdx.x;
	int ai = thid;
	int bi = thid + blockDim.x;

	// Zero out the shared memory
	// Helpful especially when input size is not power of two
	s_out[thid] = 0;
	s_out[thid + blockDim.x] = 0;
	// If CONFLICT_FREE_OFFSET is used, shared memory size
	//  must be a 2 * blockDim.x + blockDim.x/num_banks
	s_out[thid + blockDim.x + (blockDim.x >> LOG_NUM_BANKS)] = 0;

	__syncthreads();

	// Copy d_in to shared memory
	// Note that d_in's elements are scattered into shared memory
	//  in light of avoiding bank conflicts
	unsigned int cpy_idx = max_elems_per_block * blockIdx.x + threadIdx.x;
	if (cpy_idx < len)
	{
		s_out[ai + CONFLICT_FREE_OFFSET(ai)] = d_in[cpy_idx];
		if (cpy_idx + blockDim.x < len)
			s_out[bi + CONFLICT_FREE_OFFSET(bi)] = d_in[cpy_idx + blockDim.x];
	}

	// For both upsweep and downsweep:
	// Sequential indices with conflict free padding
	//  Amount of padding = target index / num banks
	//  This "shifts" the target indices by one every multiple
	//   of the num banks
	// offset controls the stride and starting index of 
	//  target elems at every iteration
	// d just controls which threads are active
	// Sweeps are pivoted on the last element of shared memory

	// Upsweep/Reduce step
	int offset = 1;
	for (int d = max_elems_per_block >> 1; d > 0; d >>= 1)
	{
		__syncthreads();

		if (thid < d)
		{
			int ai = offset * ((thid << 1) + 1) - 1;
			int bi = offset * ((thid << 1) + 2) - 1;
			ai += CONFLICT_FREE_OFFSET(ai);
			bi += CONFLICT_FREE_OFFSET(bi);

			s_out[bi] += s_out[ai];
		}
		offset <<= 1;
	}

	// Save the total sum on the global block sums array
	// Then clear the last element on the shared memory
	if (thid == 0)
	{
		d_block_sums[blockIdx.x] = s_out[max_elems_per_block - 1
			+ CONFLICT_FREE_OFFSET(max_elems_per_block - 1)];
		s_out[max_elems_per_block - 1
			+ CONFLICT_FREE_OFFSET(max_elems_per_block - 1)] = 0;
	}

	// Downsweep step
	for (int d = 1; d < max_elems_per_block; d <<= 1)
	{
		offset >>= 1;
		__syncthreads();

		if (thid < d)
		{
			int ai = offset * ((thid << 1) + 1) - 1;
			int bi = offset * ((thid << 1) + 2) - 1;
			ai += CONFLICT_FREE_OFFSET(ai);
			bi += CONFLICT_FREE_OFFSET(bi);

			unsigned int temp = s_out[ai];
			s_out[ai] = s_out[bi];
			s_out[bi] += temp;
		}
	}
	__syncthreads();

	// Copy contents of shared memory to global memory
	if (cpy_idx < len)
	{
		d_out[cpy_idx] = s_out[ai + CONFLICT_FREE_OFFSET(ai)];
		if (cpy_idx + blockDim.x < len)
			d_out[cpy_idx + blockDim.x] = s_out[bi + CONFLICT_FREE_OFFSET(bi)];
	}
}

void sum_scan_blelloch(unsigned int* const d_out, const unsigned int* const d_in, const size_t numElems)
{
	// Zero out d_out
	checkCudaErrors(cudaMemset(d_out, 0, numElems * sizeof(unsigned int)));

	// Set up number of threads and blocks

	unsigned int block_sz = MAX_BLOCK_SZ / 2;
	unsigned int max_elems_per_block = 2 * block_sz; // due to binary tree nature of algorithm

	// If input size is not power of two, the remainder will still need a whole block
	// Thus, number of blocks must be the ceiling of input size / max elems that a block can handle
	//unsigned int grid_sz = (unsigned int) std::ceil((double) numElems / (double) max_elems_per_block);
	// UPDATE: Instead of using ceiling and risking miscalculation due to precision, just automatically  
	//  add 1 to the grid size when the input size cannot be divided cleanly by the block's capacity
	unsigned int grid_sz = numElems / max_elems_per_block;
	// Take advantage of the fact that integer division drops the decimals
	if (numElems % max_elems_per_block != 0)
		grid_sz += 1;

	// Conflict free padding requires that shared memory be more than 2 * block_sz
	unsigned int shmem_sz = max_elems_per_block + ((max_elems_per_block) >> LOG_NUM_BANKS);

	// Allocate memory for array of total sums produced by each block
	// Array length must be the same as number of blocks
	unsigned int* d_block_sums;
	checkCudaErrors(cudaMalloc(&d_block_sums, sizeof(unsigned int) * grid_sz));
	checkCudaErrors(cudaMemset(d_block_sums, 0, sizeof(unsigned int) * grid_sz));

	// Sum scan data allocated to each block
	//gpu_sum_scan_blelloch<<<grid_sz, block_sz, sizeof(unsigned int) * max_elems_per_block >>>(d_out, d_in, d_block_sums, numElems);
	gpu_prescan << <grid_sz, block_sz, sizeof(unsigned int)* shmem_sz >> > (
		d_out,
		d_in,
		d_block_sums,
		numElems,
		shmem_sz,
		max_elems_per_block);

	// Sum scan total sums produced by each block
	// Use basic implementation if number of total sums is <= 2 * block_sz
	//  (This requires only one block to do the scan)
	if (grid_sz <= max_elems_per_block)
	{
		unsigned int* d_dummy_blocks_sums;
		checkCudaErrors(cudaMalloc(&d_dummy_blocks_sums, sizeof(unsigned int)));
		checkCudaErrors(cudaMemset(d_dummy_blocks_sums, 0, sizeof(unsigned int)));
		//gpu_sum_scan_blelloch<<<1, block_sz, sizeof(unsigned int) * max_elems_per_block>>>(d_block_sums, d_block_sums, d_dummy_blocks_sums, grid_sz);
		gpu_prescan << <1, block_sz, sizeof(unsigned int)* shmem_sz >> > (d_block_sums,
			d_block_sums,
			d_dummy_blocks_sums,
			grid_sz,
			shmem_sz,
			max_elems_per_block);
		checkCudaErrors(cudaFree(d_dummy_blocks_sums));
	}
	// Else, recurse on this same function as you'll need the full-blown scan
	//  for the block sums
	else
	{
		unsigned int* d_in_block_sums;
		checkCudaErrors(cudaMalloc(&d_in_block_sums, sizeof(unsigned int) * grid_sz));
		checkCudaErrors(cudaMemcpy(d_in_block_sums, d_block_sums, sizeof(unsigned int) * grid_sz, cudaMemcpyDeviceToDevice));
		sum_scan_blelloch(d_block_sums, d_in_block_sums, grid_sz);
		checkCudaErrors(cudaFree(d_in_block_sums));
	}

	//// Uncomment to examine block sums
	//unsigned int* h_block_sums = new unsigned int[grid_sz];
	//checkCudaErrors(cudaMemcpy(h_block_sums, d_block_sums, sizeof(unsigned int) * grid_sz, cudaMemcpyDeviceToHost));
	//std::cout << "Block sums: ";
	//for (int i = 0; i < grid_sz; ++i)
	//{
	//    std::cout << h_block_sums[i] << ", ";
	//}
	//std::cout << std::endl;
	//std::cout << "Block sums length: " << grid_sz << std::endl;
	//delete[] h_block_sums;

	// Add each block's total sum to its scan output
	// in order to get the final, global scanned array
	gpu_add_block_sums << <grid_sz, block_sz >> > (d_out, d_out, d_block_sums, numElems);

	checkCudaErrors(cudaFree(d_block_sums));
}


__global__ void gpu_radix_sort_local(unsigned int* d_out_sorted,
	unsigned int* d_prefix_sums,
	unsigned int* d_block_sums,
	unsigned int input_shift_width,
	unsigned int* d_in,
	unsigned int d_in_len,
	unsigned int max_elems_per_block)
{
	// need shared memory array for:
	// - block's share of the input data (local sort will be put here too)
	// - mask outputs
	// - scanned mask outputs
	// - merged scaned mask outputs ("local prefix sum")
	// - local sums of scanned mask outputs
	// - scanned local sums of scanned mask outputs

	// for all radix combinations:
	//  build mask output for current radix combination
	//  scan mask ouput
	//  store needed value from current prefix sum array to merged prefix sum array
	//  store total sum of mask output (obtained from scan) to global block sum array
	// calculate local sorted address from local prefix sum and scanned mask output's total sums
	// shuffle input block according to calculated local sorted addresses
	// shuffle local prefix sums according to calculated local sorted addresses
	// copy locally sorted array back to global memory
	// copy local prefix sum array back to global memory

	extern __shared__ unsigned int shmem[];
	unsigned int* s_data = shmem;
	// s_mask_out[] will be scanned in place
	unsigned int s_mask_out_len = max_elems_per_block + 1;
	unsigned int* s_mask_out = &s_data[max_elems_per_block];
	unsigned int* s_merged_scan_mask_out = &s_mask_out[s_mask_out_len];
	unsigned int* s_mask_out_sums = &s_merged_scan_mask_out[max_elems_per_block];
	unsigned int* s_scan_mask_out_sums = &s_mask_out_sums[4];

	unsigned int thid = threadIdx.x;

	// Copy block's portion of global input data to shared memory
	unsigned int cpy_idx = max_elems_per_block * blockIdx.x + thid;
	if (cpy_idx < d_in_len)
		s_data[thid] = d_in[cpy_idx];
	else
		s_data[thid] = 0;

	__syncthreads();

	// To extract the correct 2 bits, we first shift the number
	//  to the right until the correct 2 bits are in the 2 LSBs,
	//  then mask on the number with 11 (3) to remove the bits
	//  on the left
	unsigned int t_data = s_data[thid];
	unsigned int t_2bit_extract = (t_data >> input_shift_width) & 3;

	for (unsigned int i = 0; i < 4; ++i)
	{
		// Zero out s_mask_out
		s_mask_out[thid] = 0;
		if (thid == 0)
			s_mask_out[s_mask_out_len - 1] = 0;

		__syncthreads();

		// build bit mask output
		bool val_equals_i = false;
		if (cpy_idx < d_in_len)
		{
			val_equals_i = t_2bit_extract == i;
			s_mask_out[thid] = val_equals_i;
		}
		__syncthreads();

		// Scan mask outputs (Hillis-Steele)
		int partner = 0;
		unsigned int sum = 0;
		unsigned int max_steps = (unsigned int)log2f(max_elems_per_block);
		for (unsigned int d = 0; d < max_steps; d++) {
			partner = thid - (1 << d);
			if (partner >= 0) {
				sum = s_mask_out[thid] + s_mask_out[partner];
			}
			else {
				sum = s_mask_out[thid];
			}
			__syncthreads();
			s_mask_out[thid] = sum;
			__syncthreads();
		}

		// Shift elements to produce the same effect as exclusive scan
		unsigned int cpy_val = 0;
		cpy_val = s_mask_out[thid];
		__syncthreads();
		s_mask_out[thid + 1] = cpy_val;
		__syncthreads();

		if (thid == 0)
		{
			// Zero out first element to produce the same effect as exclusive scan
			s_mask_out[0] = 0;
			unsigned int total_sum = s_mask_out[s_mask_out_len - 1];
			s_mask_out_sums[i] = total_sum;
			d_block_sums[i * gridDim.x + blockIdx.x] = total_sum;
		}
		__syncthreads();

		if (val_equals_i && (cpy_idx < d_in_len))
		{
			s_merged_scan_mask_out[thid] = s_mask_out[thid];
		}

		__syncthreads();
	}

	// Scan mask output sums
	// Just do a naive scan since the array is really small
	if (thid == 0)
	{
		unsigned int run_sum = 0;
		for (unsigned int i = 0; i < 4; ++i)
		{
			s_scan_mask_out_sums[i] = run_sum;
			run_sum += s_mask_out_sums[i];
		}
	}

	__syncthreads();

	if (cpy_idx < d_in_len)
	{
		// Calculate the new indices of the input elements for sorting
		unsigned int t_prefix_sum = s_merged_scan_mask_out[thid];
		unsigned int new_pos = t_prefix_sum + s_scan_mask_out_sums[t_2bit_extract];

		__syncthreads();

		// Shuffle the block's input elements to actually sort them
		// Do this step for greater global memory transfer coalescing
		//  in next step
		s_data[new_pos] = t_data;
		s_merged_scan_mask_out[new_pos] = t_prefix_sum;

		__syncthreads();

		// Copy block - wise prefix sum results to global memory
		// Copy block-wise sort results to global 
		d_prefix_sums[cpy_idx] = s_merged_scan_mask_out[thid];
		d_out_sorted[cpy_idx] = s_data[thid];
	}
}

__global__ void gpu_glbl_shuffle(unsigned int* d_out,
	unsigned int* d_in,
	unsigned int* d_scan_block_sums,
	unsigned int* d_prefix_sums,
	unsigned int input_shift_width,
	unsigned int d_in_len,
	unsigned int max_elems_per_block)
{
	// get d = digit
	// get n = blockIdx
	// get m = local prefix sum array value
	// calculate global position = P_d[n] + m
	// copy input element to final position in d_out

	unsigned int thid = threadIdx.x;
	unsigned int cpy_idx = max_elems_per_block * blockIdx.x + thid;

	if (cpy_idx < d_in_len)
	{
		unsigned int t_data = d_in[cpy_idx];
		unsigned int t_2bit_extract = (t_data >> input_shift_width) & 3;
		unsigned int t_prefix_sum = d_prefix_sums[cpy_idx];
		unsigned int data_glbl_pos = d_scan_block_sums[t_2bit_extract * gridDim.x + blockIdx.x]
			+ t_prefix_sum;
		__syncthreads();
		d_out[data_glbl_pos] = t_data;
	}
}

// An attempt at the gpu radix sort variant described in this paper:
// https://vgc.poly.edu/~csilva/papers/cgf.pdf
void radix_sort(unsigned int* const d_out___, unsigned int* const d_in___, unsigned int d_in_len)
{
	const int Num = 1000000;
	d_in_len = Num;
	unsigned int* d_out = new unsigned int[Num];
	unsigned int* d_in = new unsigned int[Num];

	unsigned int block_sz = MAX_BLOCK_SZ;
	unsigned int max_elems_per_block = block_sz;
	unsigned int grid_sz = d_in_len / max_elems_per_block;
	// Take advantage of the fact that integer division drops the decimals
	if (d_in_len % max_elems_per_block != 0)
		grid_sz += 1;

	unsigned int* d_prefix_sums;
	unsigned int d_prefix_sums_len = d_in_len;
	checkCudaErrors(cudaMalloc(&d_prefix_sums, sizeof(unsigned int) * d_prefix_sums_len));
	checkCudaErrors(cudaMemset(d_prefix_sums, 0, sizeof(unsigned int) * d_prefix_sums_len));

	unsigned int* d_block_sums;
	unsigned int d_block_sums_len = 4 * grid_sz; // 4-way split
	checkCudaErrors(cudaMalloc(&d_block_sums, sizeof(unsigned int) * d_block_sums_len));
	checkCudaErrors(cudaMemset(d_block_sums, 0, sizeof(unsigned int) * d_block_sums_len));

	unsigned int* d_scan_block_sums;
	checkCudaErrors(cudaMalloc(&d_scan_block_sums, sizeof(unsigned int) * d_block_sums_len));
	checkCudaErrors(cudaMemset(d_scan_block_sums, 0, sizeof(unsigned int) * d_block_sums_len));

	// shared memory consists of 3 arrays the size of the block-wise input
	//  and 2 arrays the size of n in the current n-way split (4)
	unsigned int s_data_len = max_elems_per_block;
	unsigned int s_mask_out_len = max_elems_per_block + 1;
	unsigned int s_merged_scan_mask_out_len = max_elems_per_block;
	unsigned int s_mask_out_sums_len = 4; // 4-way split
	unsigned int s_scan_mask_out_sums_len = 4;
	unsigned int shmem_sz = (s_data_len
		+ s_mask_out_len
		+ s_merged_scan_mask_out_len
		+ s_mask_out_sums_len
		+ s_scan_mask_out_sums_len)
		* sizeof(unsigned int);

	cudaEvent_t start_event, stop_event;
	cudaEventCreate(&start_event);
	cudaEventCreate(&stop_event);

	int numIterations = 100;
	for (unsigned int xxx = 0; xxx <= numIterations; xxx++)
	{
		// for every 2 bits from LSB to MSB:
		//  block-wise radix sort (write blocks back to global memory)
		for (unsigned int shift_width = 0; shift_width <= 30; shift_width += 2)
		{
			gpu_radix_sort_local << <grid_sz, block_sz, shmem_sz >> > (d_out,
				d_prefix_sums,
				d_block_sums,
				shift_width,
				d_in,
				d_in_len,
				max_elems_per_block);

			//unsigned int* h_test = new unsigned int[d_in_len];
			//checkCudaErrors(cudaMemcpy(h_test, d_in, sizeof(unsigned int) * d_in_len, cudaMemcpyDeviceToHost));
			//for (unsigned int i = 0; i < d_in_len; ++i)
			//    std::cout << h_test[i] << " ";
			//std::cout << std::endl;
			//delete[] h_test;

			//// scan global block sum array
			//sum_scan_blelloch(d_scan_block_sums, d_block_sums, d_block_sums_len);
			//
			//// scatter/shuffle block-wise sorted array to final positions
			//gpu_glbl_shuffle << <grid_sz, block_sz >> > (d_in,
			//	d_out,
			//	d_scan_block_sums,
			//	d_prefix_sums,
			//	shift_width,
			//	d_in_len,
			//	max_elems_per_block);
		}
	}

	cudaEventRecord(stop_event, 0);
	cudaEventSynchronize(stop_event);
	float totalTime;
	cudaEventElapsedTime(&totalTime, start_event, stop_event);
	//totalTime += time;

	totalTime /= (1.0e3f * numIterations);
	printf("radixSortThrust, Throughput = %.4f MElements/s, Time = %.5f s, Size = %u elements\n",
		1.0e-6f * Num / totalTime, totalTime, Num);

	//checkCudaErrors(cudaMemcpy(d_out, d_in, sizeof(unsigned int) * d_in_len, cudaMemcpyDeviceToDevice));
	//checkCudaErrors(cudaFree(d_scan_block_sums));
	//checkCudaErrors(cudaFree(d_block_sums));
	//checkCudaErrors(cudaFree(d_prefix_sums));
}

#endif


ndCudaContextImplement::ndCudaContextImplement(ndCudaDevice* const device)
	:m_device(device)
	//,m_sceneInfoGpu(nullptr)
	//,m_sceneInfoCpu(nullptr)
	//,m_histogram()
	//,m_bodyBuffer()
	//,m_sceneGraph()
	//,m_bodyAabbCell()
	//,m_bodyAabbCellScratch()
	//,m_transformBuffer0()
	//,m_transformBuffer1()
	//,m_transformBufferCpu()
	//,m_solverMemCpuStream(0)
	//,m_solverComputeStream(0)
	//,m_timeInSeconds(0.0f)
	//,m_frameCounter(0)
{
	//cudaError_t cudaStatus;
	//cudaStatus = cudaStreamCreate(&m_solverMemCpuStream);
	//ndAssert(cudaStatus == cudaSuccess);
	//
	//cudaStatus = cudaStreamCreate(&m_solverComputeStream);
	//ndAssert(cudaStatus == cudaSuccess);
	//
	//cudaStatus = cudaMalloc((void**)&m_sceneInfoGpu, sizeof(ndCudaSceneInfo));
	//ndAssert(cudaStatus == cudaSuccess);
	//
	//cudaStatus = cudaMallocHost((void**)&m_sceneInfoCpu, sizeof(ndCudaSceneInfo));
	//ndAssert(cudaStatus == cudaSuccess);
	//
	//if (cudaStatus != cudaSuccess)
	//{
	//	ndAssert(0);
	//}
	//
	//*m_sceneInfoCpu = ndCudaSceneInfo();
	m_sortPrefixBuffer.SetCount(m_sortPrefixBuffer.GetCapacity());

	// ***********************************
	//m_src.SetCount(8);
	//m_src.SetCount(17);
	//m_src.SetCount(64);
	//m_src.SetCount(256);
	//m_src.SetCount(301);
	//m_src.SetCount(512);
	//m_src.SetCount(512 + 99);
	//m_src.SetCount(10000);
	//m_src.SetCount(100000);
	m_src.SetCount(1000000);
	for (int i = 0; i < m_src.GetCount(); ++i)
	{
		//m_src[i] = rand() % 256;
		m_src[i] = rand() % 1024;
		//m_src[i] = rand() & 0x7fffffff;
		//m_src[i] = m_src.GetCount() - i - 1;
		//m_src[i] = m_src[i] & 0xff;
		//m_src[i] = m_src.GetCount() - 1 - i;
	}

	//m_src[4] = 1;
	//m_src[9] = 1;
	//m_src[14] = 1;
	//m_src[0] = 255;
	//m_src[11] = 1;
	//m_src[20] = 0;
	//m_src[21] = 1;

	m_scan0.SetCount(1024 * 256);
	m_scan1.SetCount(1024 * 256);
	m_dst0.SetCount(m_src.GetCount());
	m_dst1.SetCount(m_src.GetCount());
	m_buf0.SetCount(m_src.GetCount());
	m_buf1.SetCount(m_src.GetCount());
	m_buf0.ReadData(&m_src[0], m_src.GetCount());
	m_buf1.ReadData(&m_dst1[0], m_dst1.GetCount());

	class GetKey0
	{
		public:
		int GetRadix(int item) const
		{
			return item & 0xff;
		};
	};

	class GetKey1
	{
		public:
		int GetRadix(int item) const
		{
			return (item >> 8) & 0xff;
		};
	};

	//ndCountingSort<int, GetKey0, 8>(m_src, m_dst0, m_scan0);
	//ndCountingSort<int, GetKey1, 8>(m_dst0, m_src, m_scan1);
	//for (int i = 1; i < m_dst0.GetCount(); ++i)
	//{
	//	//int a = key.GetRadix(m_dst0[i - 1]);
	//	//int b = key.GetRadix(m_dst0[i - 0]);
	//	int a = m_src[i - 1];
	//	int b = m_src[i - 0];
	//	ndAssert(a <= b);
	//}
}

ndCudaContextImplement::~ndCudaContextImplement()
{
	ndAssert(m_device);
	
	//cudaError_t cudaStatus;
	//cudaStatus = cudaFreeHost(m_sceneInfoCpu);
	//ndAssert(cudaStatus == cudaSuccess);
	//
	//cudaStatus = cudaFree(m_sceneInfoGpu);
	//ndAssert(cudaStatus == cudaSuccess);
	//
	//cudaStatus = cudaStreamDestroy(m_solverComputeStream);
	//ndAssert(cudaStatus == cudaSuccess);
	//
	//cudaStatus = cudaStreamDestroy(m_solverMemCpuStream);
	//ndAssert(cudaStatus == cudaSuccess);
	//
	//if (cudaStatus != cudaSuccess)
	//{
	//	ndAssert(0);
	//}
}

ndCudaDevice* ndCudaContextImplement::GetDevice() const
{
	return m_device;
}

const char* ndCudaContextImplement::GetStringId() const
{
	return m_device->m_prop.name;
}

int ndCudaContextImplement::GetComputeUnits() const
{
	return m_device->GetComputeUnits();
}

ndCudaDeviceBuffer<int>& ndCudaContextImplement::GetPrefixScanBuffer()
{
	return m_sortPrefixBuffer;
}

#if 0
ndCudaSpatialVector* ndCudaContextImplement::GetTransformBuffer()
{
	return &m_transformBufferCpu[0];
}

void ndCudaContextImplement::ResizeBuffers(int cpuBodyCount)
{
	ndCudaDeviceBuffer<unsigned>& histogram = m_histogram;
	ndCudaDeviceBuffer<ndCudaBodyProxy>& bodyBuffer = m_bodyBuffer;
	ndCudaDeviceBuffer<ndCudaSceneNode>& sceneGraph = m_sceneGraph;
	ndCudaDeviceBuffer<ndCudaBodyAabbCell>& bodyAabbCell0 = m_bodyAabbCell;
	ndCudaDeviceBuffer<ndCudaBodyAabbCell>& bodyAabbCell1 = m_bodyAabbCellScratch;
	ndCudaDeviceBuffer<ndCudaSpatialVector>& transformBuffer0 = m_transformBuffer0;
	ndCudaDeviceBuffer<ndCudaSpatialVector>& transformBuffer1 = m_transformBuffer1;
	
	histogram.SetCount(cpuBodyCount);
	bodyBuffer.SetCount(cpuBodyCount);
	sceneGraph.SetCount(cpuBodyCount * 2);
	bodyAabbCell0.SetCount(cpuBodyCount);
	bodyAabbCell1.SetCount(cpuBodyCount);
	transformBuffer0.SetCount(cpuBodyCount);
	transformBuffer1.SetCount(cpuBodyCount);

	ndCudaHostBuffer<ndCudaSpatialVector>& transformBufferCpu = m_transformBufferCpu;
	transformBufferCpu.SetCount(cpuBodyCount);
}

void ndCudaContextImplement::LoadBodyData(const ndCudaBodyProxy* const src, int cpuBodyCount)
{
	m_imageCpu.m_context->m_device->SyncDevice();
		
	ndCudaSceneInfo info;
	info.m_histogram = ndCudaBuffer<unsigned>(m_histogram);
	info.m_bodyArray = ndCudaBuffer<ndCudaBodyProxy>(m_bodyBuffer);
	info.m_sceneGraph = ndCudaBuffer<ndCudaSceneNode>(m_sceneGraph);
	info.m_bodyAabbCell = ndCudaBuffer<ndCudaBodyAabbCell>(m_bodyAabbCell);
	info.m_bodyAabbCellScratch = ndCudaBuffer<ndCudaBodyAabbCell>(m_bodyAabbCellScratch);
	info.m_transformBuffer0 = ndCudaBuffer<ndCudaSpatialVector>(m_transformBuffer0);
	info.m_transformBuffer1 = ndCudaBuffer<ndCudaSpatialVector>(m_transformBuffer1);
	
	*m_sceneInfoCpu = info;
	cudaError_t cudaStatus = cudaMemcpy(m_sceneInfoGpu, &info, sizeof(ndCudaSceneInfo), cudaMemcpyHostToDevice);
	ndAssert(cudaStatus == cudaSuccess);

	const int blocksCount = (cpuBodyCount + D_THREADS_PER_BLOCK - 1) / D_THREADS_PER_BLOCK;
	m_bodyBuffer.ReadData(src, cpuBodyCount);
	ndCudaInitTransforms << <blocksCount, D_THREADS_PER_BLOCK, 0, m_solverComputeStream >> > (*m_sceneInfoCpu);
	ndCudaGenerateSceneGraph << <1, 1, 0, m_solverComputeStream >> > (*m_sceneInfoCpu);
	
	m_imageCpu.m_context->m_device->SyncDevice();
	if (cudaStatus != cudaSuccess)
	{
		ndAssert(0);
	}
}

void ndCudaContextImplement::ValidateContextBuffers()
{
	ndCudaSceneInfo* const sceneInfo = m_sceneInfoCpu;
	if (!sceneInfo->m_frameIsValid)
	{
		m_device->SyncDevice();

		if (sceneInfo->m_histogram.m_size > sceneInfo->m_histogram.m_capacity)
		{
			sceneInfo->m_frameIsValid = 1;
			m_histogram.SetCount(sceneInfo->m_histogram.m_size);
			sceneInfo->m_histogram = ndCudaBuffer<unsigned>(m_histogram);
		}

		if (sceneInfo->m_bodyAabbCell.m_size > sceneInfo->m_bodyAabbCell.m_capacity)
		{
			sceneInfo->m_frameIsValid = 1;
			m_bodyAabbCell.SetCount(sceneInfo->m_bodyAabbCell.m_size);
			m_bodyAabbCellScratch.SetCount(sceneInfo->m_bodyAabbCell.m_size);
			sceneInfo->m_bodyAabbCell = ndCudaBuffer<ndCudaBodyAabbCell>(m_bodyAabbCell);
			sceneInfo->m_bodyAabbCellScratch = ndCudaBuffer<ndCudaBodyAabbCell>(m_bodyAabbCellScratch);
		}

		if (!sceneInfo->m_frameCount)
		{
			sceneInfo->m_frameIsValid = 1;
		}

		ndAssert(sceneInfo->m_frameIsValid);
		cudaError_t cudaStatus = cudaMemcpy(m_sceneInfoGpu, sceneInfo, sizeof(ndCudaSceneInfo), cudaMemcpyHostToDevice);
		ndAssert(cudaStatus == cudaSuccess);
		if (cudaStatus != cudaSuccess)
		{
			ndAssert(0);
		}
		m_imageCpu.m_context->m_device->SyncDevice();
	}
}

void ndCudaContextImplement::UpdateTransform()
{
	ndCudaSceneInfo* const infoGpu = m_sceneInfoGpu;
	ndCudaGetBodyTransforms << <1, 1, 0, m_solverComputeStream >> > (*infoGpu);
}

void ndCudaContextImplement::InitBodyArray()
{
	//auto CompactMovingBodies = ndMakeObject::ndFunction([this, &scans](int threadIndex, int threadCount)
	//{
	//	const ndArray<ndBodyKinematic*>& activeBodyArray = GetActiveBodyArray();
	//	ndBodyKinematic** const sceneBodyArray = &m_sceneBodyArray[0];
	//
	//	const ndArray<ndBodyKinematic*>& view = m_bodyList.m_view;
	//	int* const scan = &scans[threadIndex][0];
	//
	//	const ndStartEnd startEnd(view.GetCount(), threadIndex, threadCount);
	//	for (int i = startEnd.m_start; i < startEnd.m_end; ++i)
	//	{
	//		ndBodyKinematic* const body = activeBodyArray[i];
	//		const int key = body->m_sceneEquilibrium;
	//		const int index = scan[key];
	//		sceneBodyArray[index] = body;
	//		scan[key] ++;
	//	}
	//});
	//ParallelExecute(BuildBodyArray);
	//int sum = 0;
	//int threadCount = GetThreadCount();
	//for (int j = 0; j < 2; j++)
	//{
	//	for (int i = 0; i < threadCount; ++i)
	//	{
	//		const int count = scans[i][j];
	//		scans[i][j] = sum;
	//		sum += count;
	//	}
	//}
	//
	//int movingBodyCount = scans[0][1] - scans[0][0];
	//m_sceneBodyArray.SetCount(m_bodyList.GetCount());
	//if (movingBodyCount)
	//{
	//	ParallelExecute(CompactMovingBodies);
	//}
	//
	//m_sceneBodyArray.SetCount(movingBodyCount);
	//
	//ndBodyKinematic* const sentinelBody = m_sentinelBody;
	//sentinelBody->PrepareStep(GetActiveBodyArray().GetCount() - 1);
	//
	//sentinelBody->m_isStatic = 1;
	//sentinelBody->m_autoSleep = 1;
	//sentinelBody->m_equilibrium = 1;
	//sentinelBody->m_equilibrium0 = 1;
	//sentinelBody->m_isJointFence0 = 1;
	//sentinelBody->m_isJointFence1 = 1;
	//sentinelBody->m_isConstrained = 0;
	//sentinelBody->m_sceneEquilibrium = 1;
	//sentinelBody->m_weigh = ndFloat32(0.0f);

	auto GetItemsCount = [] __device__(const ndCudaSceneInfo & info)
	{
		return info.m_bodyAabbCell.m_size - 1;
	};

	auto GetSrcBuffer = [] __device__(const ndCudaSceneInfo &info)
	{
		return &info.m_bodyAabbCell.m_array[0].m_value;
	};

	auto GetDstBuffer = [] __device__(const ndCudaSceneInfo & info)
	{
		return &info.m_bodyAabbCellScratch.m_array[0].m_value;
	};

	auto GetSortKey_x = [] __device__(long long value)
	{
		ndCudaBodyAabbCell item;
		item.m_value = value;
		const unsigned key = item.m_key;
		return key & 0xff;
	};

	auto GetSortKey_y = [] __device__(long long value)
	{
		ndCudaBodyAabbCell item;
		item.m_value = value;
		const unsigned key = item.m_key;
		return (key>>8) & 0xff;
	};
	
	auto GetSortKey_z = [] __device__(long long value)
	{
		ndCudaBodyAabbCell item;
		item.m_value = value;
		const unsigned key = item.m_key;
		return (key >> 16) & 0xff;
	};
	
	auto GetSortKey_w = [] __device__(long long value)
	{
		ndCudaBodyAabbCell item;
		item.m_value = value;
		const unsigned key = item.m_key;
		return (key >> 24) & 0xff;
	};

	long long dommyType = 0;
	ndCudaSceneInfo* const infoGpu = m_sceneInfoGpu;

	ndCudaInitBodyArray << <1, 1, 0, m_solverComputeStream >> > (*infoGpu);
	ndCudaHillisSteelePrefixScan << <1, 1, 0, m_solverComputeStream >> > (*infoGpu);
	ndCudaGenerateGrids << <1, 1, 0, m_solverComputeStream >> > (*infoGpu);
	ndCudaCountingSort << <1, 1, 0, m_solverComputeStream >> > (*infoGpu, dommyType, GetSrcBuffer, GetDstBuffer, GetItemsCount, GetSortKey_x, 256);
	ndCudaCountingSort << <1, 1, 0, m_solverComputeStream >> > (*infoGpu, dommyType, GetDstBuffer, GetSrcBuffer, GetItemsCount, GetSortKey_y, 256);
	ndCudaCountingSort << <1, 1, 0, m_solverComputeStream >> > (*infoGpu, dommyType, GetSrcBuffer, GetDstBuffer, GetItemsCount, GetSortKey_z, 256);
	ndCudaCountingSort << <1, 1, 0, m_solverComputeStream >> > (*infoGpu, dommyType, GetDstBuffer, GetSrcBuffer, GetItemsCount, GetSortKey_w, 256);
	ndCudaCalculateBodyPairsCount << <1, 1, 0, m_solverComputeStream >> > (*infoGpu);
}

#endif

void ndCudaContextImplement::PrepareCleanup()
{
	m_device->SyncDevice();
}

void ndCudaContextImplement::Cleanup()
{
	m_device->SyncDevice();

#ifdef	D_USE_EVENT_FOR_SYNC
	m_device->m_lastError = cudaEventRecord(m_device->m_syncEvent, 0);
	ndAssert(m_device->m_lastError == cudaSuccess);
#endif
}

void ndCudaContextImplement::Begin()
{
#ifdef D_PROFILE_KERNELS
	m_device->m_lastError = cudaEventRecord(m_device->m_startTimer, 0);
	ndAssert(m_device->m_lastError == cudaSuccess);
#endif

	//// get the scene info from the update	
	//ndCudaSceneInfo* const gpuInfo = m_sceneInfoGpu;
	//ndCudaSceneInfo* const cpuInfo = m_sceneInfoCpu;
	//
	//cudaError_t cudaStatus = cudaMemcpyAsync(cpuInfo, gpuInfo, sizeof(ndCudaSceneInfo), cudaMemcpyDeviceToHost, m_solverMemCpuStream);
	//ndAssert(cudaStatus == cudaSuccess);
	//if (cudaStatus != cudaSuccess)
	//{
	//	ndAssert(0);
	//}
	//
	//m_timeInSeconds = double(cpuInfo->m_frameTimeInNanosecunds) * double(1.0e-9f);
	////printf("cpu frame:%d ms:%lld\n", cpuInfo->m_frameCount, cpuInfo->m_frameTimeInNanosecunds/1000000);
	//
	//const int frameCounter = m_frameCounter;
	//if (frameCounter)
	//{
	//	ndCudaHostBuffer<ndCudaSpatialVector>& cpuBuffer = m_transformBufferCpu;
	//	ndCudaDeviceBuffer<ndCudaSpatialVector>& gpuBuffer = (frameCounter & 1) ? m_transformBuffer1 : m_transformBuffer0;
	//	gpuBuffer.WriteData(&cpuBuffer[0], cpuBuffer.GetCount() - 1, m_solverMemCpuStream);
	//}
	//
	//ndCudaBeginFrame << < 1, 1, 0, m_solverComputeStream >> > (*gpuInfo);

	//class GetKey
	//{
	//	public:
	//	int GetRadix(int item) const
	//	{
	//		return item & 0xff;
	//	};
	//};
	//ndCountingSort<int, GetKey, 8>(m_src, m_dst0, m_scan0);
	//ndCountingSort<int, GetKey, 8>(m_src, m_dst0, m_scan0);
	//ndCountingSort<int, GetKey, 8>(m_src, m_dst0, m_scan0);
	//ndCountingSort<int, GetKey, 8>(m_src, m_dst0, m_scan0);

#if 1

	cudaEvent_t start_event, stop_event;
	cudaEventCreate(&start_event);
	cudaEventCreate(&stop_event);

	int numIterations = 1;
	//int numIterations = 100;
	cudaEventRecord(start_event, 0);
	for (int i = 0; i < numIterations; ++i)
	{
#if 1
		auto GetRadix = []  __host__ __device__(int item)
		{
			return item & (1024 - 1);
		};

		ndCountingSortUnOrdered<int, 8>(this, m_buf0, m_buf1, GetRadix);
		//ndCountingSortUnOrdered<int, 8>(this, m_buf1, m_buf0, GetRadix);
		//ndCountingSortUnOrdered<int, 8>(this, m_buf0, m_buf1, GetRadix);
		//ndCountingSortUnOrdered<int, 8>(this, m_buf1, m_buf0, GetRadix);
#else
		auto GetRadix0 = []  __host__ __device__(int item)
		{
			return item & 0xff;
		};

		auto GetRadix1 = []  __host__ __device__(int item)
		{
			return (item >> 8) & 0xff;
		};

		auto GetRadix2 = []  __host__ __device__(int item)
		{
			return (item >> 16) & 0xff;
		};

		auto GetRadix3 = []  __host__ __device__(int item)
		{
			return (item >> 24) & 0xff;
		};

		ndCountingSort<int, 8>(this, m_buf0, m_buf1, GetRadix0);
		ndCountingSort<int, 8>(this, m_buf1, m_buf0, GetRadix1);
		ndCountingSort<int, 8>(this, m_buf0, m_buf1, GetRadix2);
		ndCountingSort<int, 8>(this, m_buf1, m_buf0, GetRadix3);
#endif
	}
	cudaEventRecord(stop_event, 0);
	cudaEventSynchronize(stop_event);

	float totalTime;
	cudaEventElapsedTime(&totalTime, start_event, stop_event);

	totalTime = totalTime * 1.0e-3f;
	float gigKeys = float (m_buf0.GetCount() * numIterations) * 1.0e-9f;
	cudaExpandTraceMessage("newton sort, throughput = %f gigaKeys/seconds, time = %f s, size = %u elements\n", gigKeys / totalTime, totalTime/numIterations, m_buf0.GetCount());

#if 1
	m_device->SyncDevice();
	//m_buf1.WriteData(&m_dst0[0], m_dst0.GetCount());
	m_buf0.WriteData(&m_dst0[0], m_dst0.GetCount());
	//m_sortPrefixBuffer.WriteData(&m_scan1[0], m_scan1.GetCount());
	
	for (int i = 1; i < m_dst1.GetCount(); ++i)
	{
		//int a = GetRadix(m_dst1[i - 1]);
		//int b = GetRadix(m_dst1[i - 0]);
		int a = m_dst0[i - 1];
		int b = m_dst0[i - 0];
		ndAssert(a <= b);
	}
	m_buf0.WriteData(&m_dst1[0], m_dst1.GetCount());
	#endif
#endif
}

void ndCudaContextImplement::End()
{
#ifdef D_PROFILE_KERNELS
	cudaEventRecord(m_device->m_stopTimer, 0);
#endif

#ifdef	D_USE_EVENT_FOR_SYNC
	m_device->m_lastError = cudaEventRecord(m_device->m_syncEvent, 0);
	ndAssert(m_device->m_lastError == cudaSuccess);
	cudaEventSynchronize(m_device->m_syncEvent);
#else
	m_device->SyncDevice();
#endif

#ifdef D_PROFILE_KERNELS
	float elapsedTime;
	cudaEventElapsedTime(&elapsedTime, m_device->m_startTimer, m_device->m_stopTimer);

	m_device->m_timerFrames++;
	m_device->m_timeAcc += elapsedTime;
	if (m_device->m_timerFrames >= 60)
	{
		cuTrace (("kernels average time = %f ms\n", m_device->m_timeAcc / m_device->m_timerFrames));
		m_device->m_timerFrames = 0;
		m_device->m_timeAcc = 0.0f;
	}
#endif

	//m_frameCounter = m_frameCounter + 1;
	//ndCudaSceneInfo* const gpuInfo = m_sceneInfoGpu;
	//ndCudaEndFrame << < 1, 1, 0, m_solverComputeStream >> > (*gpuInfo, m_frameCounter);
}
