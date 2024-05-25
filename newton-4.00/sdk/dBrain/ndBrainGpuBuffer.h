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
#include "ndBrainGpuContext.h"

class ndBrainVector;

class ndBrainGpuBufferBase : public ndClassAlloc
{
	protected:
	class ndImplementation;
	
	ndBrainGpuBufferBase(ndBrainGpuContext* const context, ndInt32 sizeInByte);
	virtual ~ndBrainGpuBufferBase();

	ndImplementation* m_buffer;
	friend class ndScopeMapBuffer;
};

class ndScopeMapBuffer
{
	public:
	ndScopeMapBuffer(ndBrainGpuBufferBase& buffer);
	~ndScopeMapBuffer();

	void* GetPointer() const;

	private:
	void* m_mappedMemory;
	ndBrainGpuBufferBase* m_buffer;
};



class ndBrainGpuFloatBuffer : public ndBrainGpuBufferBase
{
	public:
	ndBrainGpuFloatBuffer(ndBrainGpuContext* const context, ndInt32 size);

	void UnloadData(ndBrainVector& output);
	void LoadData(const ndBrainVector& input);
};

#endif