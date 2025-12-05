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
#ifndef __ND_BRAIN_INTEGER_BUFFER_H__
#define __ND_BRAIN_INTEGER_BUFFER_H__

#include "ndBrainBuffer.h"

class ndBrainIntegerBuffer : public ndBrainBuffer
{
	public:
	ndBrainIntegerBuffer(ndBrainContext* const context, ndInt64 sizeInElements);
	ndBrainIntegerBuffer(ndBrainContext* const context, ndInt64 numberOfElements, const ndUnsigned32* const indexArray);

	virtual void* GetCpuPtr() override;
	virtual void* GetCpuPtr() const override;

	void MemoryToDevice(size_t offsetInBytes, size_t sizeInBytes, const void* const inputData);
	void MemoryFromDevice(size_t offsetInBytes, size_t sizeInBytes, void* const outputMemory) const;

	void CopyBuffer(const ndCopyBufferCommandInfo& descriptor, ndInt32 workGroupCount, const ndBrainIntegerBuffer& srcBuffer);

	//void GetRand(const ndCopyBufferCommandInfo& descriptor, ndInt32 workGroupCount, const ndBrainIntegerBuffer& srcBuffer);

	private:
	ndSharedPtr<ndArray<ndUnsigned32>> m_indexArray;

	friend class ndBrainTrainer;
};

#endif