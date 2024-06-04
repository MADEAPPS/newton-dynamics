/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/
#ifndef __ND_BRAIN_GPU_BUFFER_H__
#define __ND_BRAIN_GPU_BUFFER_H__

#include "ndBrainStdafx.h"

class ndBrainVector;
class ndBrainGpuContext;
class ndBrainGpuBuffer;

#define ND_GPU_BUFFER_ALIGNMENT	32

#if !defined (D_USE_VULKAN_SDK)

class ndScopeMapBuffer
{
	public:
	ndScopeMapBuffer(ndBrainGpuBuffer& buffer):m_mappedMemory(nullptr), m_buffer(&buffer){}
	~ndScopeMapBuffer(){}
	void* GetPointer() const{return m_mappedMemory;}

	private:
	void* m_mappedMemory;
	ndBrainGpuBuffer* m_buffer;
};

class ndBrainGpuBuffer : public ndClassAlloc
{
	protected:
	ndBrainGpuBuffer(ndBrainGpuContext* const, ndInt32, ndUnsigned32):m_sizeInBytes(0){}
	virtual ~ndBrainGpuBuffer(){}

	public:
	void* GetBuffer() const{}
	ndInt32 SizeInBytes() const{}

	protected:
	ndInt32 m_sizeInBytes;
	friend class ndScopeMapBuffer;
};

class ndBrainGpuIntegerBuffer : public ndBrainGpuBuffer
{
	public:
	ndBrainGpuIntegerBuffer(ndBrainGpuContext* const context, ndInt32 size) : ndBrainGpuBuffer(context, size, 0) {}
	ndBrainGpuIntegerBuffer(ndBrainGpuContext* const context, const ndArray<ndInt32>&) : ndBrainGpuBuffer(context, 0, 0) {}
	void UnloadData(ndArray<ndInt32>&) {}
	void LoadData(const ndArray<ndInt32>&) {}
};

class ndBrainGpuFloatBuffer : public ndBrainGpuBuffer
{
	public:
	ndBrainGpuFloatBuffer(ndBrainGpuContext* const context, ndInt32 size): ndBrainGpuBuffer(context, size, 0){}
	ndBrainGpuFloatBuffer(ndBrainGpuContext* const context, const ndBrainVector&) : ndBrainGpuBuffer(context, 0, 0) {}
	void UnloadData(ndBrainVector&) {}
	void LoadData(const ndBrainVector&) {}
};

class ndBrainGpuUniformBuffer : public ndBrainGpuBuffer
{
	public:
	ndBrainGpuUniformBuffer(ndBrainGpuContext* const context, ndInt32, const void* const) : ndBrainGpuBuffer(context, 0, 0) {}
	void LoadData(ndInt32, const void* const) {}
};

#else

class ndScopeMapBuffer
{
	public:
	ndScopeMapBuffer(ndBrainGpuBuffer& buffer);
	~ndScopeMapBuffer();
	void* GetPointer() const;

	private:
	void* m_mappedMemory;
	ndBrainGpuBuffer* m_buffer;
};

class ndBrainGpuBuffer : public ndClassAlloc
{
	public:
	virtual ~ndBrainGpuBuffer();
	VkBuffer GetBuffer() const;
	ndInt32 SizeInBytes() const;

	protected:
	ndBrainGpuBuffer(ndBrainGpuContext* const context, ndInt32 sizeInByte, ndUnsigned32 bufferTypeFlags);
	uint32_t FindMemoryType(uint32_t memoryTypeBits, VkMemoryPropertyFlags properties);

	VkBuffer m_buffer;
	VkDeviceMemory m_bufferMemory;
	ndBrainGpuContext* m_context;
	ndInt32 m_sizeInBytes;
	friend class ndScopeMapBuffer;
};

// **************************************************************************
// 
// **************************************************************************
class ndBrainGpuIntegerBuffer : public ndBrainGpuBuffer
{
	public:
	ndBrainGpuIntegerBuffer(ndBrainGpuContext* const context, ndInt32 size);
	ndBrainGpuIntegerBuffer(ndBrainGpuContext* const context, const ndArray<ndInt32>& input);

	void UnloadData(ndArray<ndInt32>& output);
	void LoadData(const ndArray<ndInt32>& input);
};

class ndBrainGpuFloatBuffer : public ndBrainGpuBuffer
{
	public:
	ndBrainGpuFloatBuffer(ndBrainGpuContext* const context, ndInt32 size);
	ndBrainGpuFloatBuffer(ndBrainGpuContext* const context, const ndBrainVector& input);

	void UnloadData(ndBrainVector& output);
	void LoadData(const ndBrainVector& input);
};

class ndBrainGpuUniformBuffer : public ndBrainGpuBuffer
{
	public:
	ndBrainGpuUniformBuffer(ndBrainGpuContext* const context, ndInt32 sizeInBytes, const void* const data);
	void LoadData(ndInt32 sizeInBytes, const void* const data);
};

#endif


#endif