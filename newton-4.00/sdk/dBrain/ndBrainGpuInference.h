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
#ifndef __ND_BRAIN_GPU_INFERENCE_H__
#define __ND_BRAIN_GPU_INFERENCE_H__

#include "ndBrainStdafx.h"
#include "ndBrainLayer.h"
#include "ndBrainVector.h"

class ndBrain;
class ndBrainGpuBuffer;
class ndBrainGpuContext;
//class ndBrainGpuFloatBuffer;
//class ndBrainGpuIntegerBuffer;

#if !defined (D_USE_VULKAN_SDK)

class ndBrainGpuInference : public ndClassAlloc
{
	public:
	ndBrainGpuInference(ndBrainGpuContext* const, ndBrain* const, const ndBrainMatrix&, ndInt32) {}
	virtual ~ndBrainGpuInference() {}

	void GetResults(ndBrainVector&) {}
	void GetWorkBuffer(ndBrainVector&) {}
	const ndList<ndSharedPtr<ndBrainGpuCommand>>& GetDisplayList() const {return m_displayList;}

	ndList<ndSharedPtr<ndBrainGpuCommand>> m_displayList;
};

#else

class ndBrainGpuInference : public ndClassAlloc
{
	class ndBrainLoadInputData;
	class ndBrainGetResultData;

	public:
	ndBrainGpuInference(ndBrainGpuContext* const context, ndBrain* const brain, const ndBrainMatrix& testDigits, ndInt32 inputBatchSize);
	virtual ~ndBrainGpuInference();

	void GetResults(ndBrainVector& results) const;
	void GetWorkBuffer(ndBrainVector& results) const;
	const ndArray<ndInt32>& GetWorkBufferOffsets() const;
	
	const ndList<ndSharedPtr<ndBrainGpuCommand>>& GetDisplayList() const;
	
	protected:
	void SetWorkingBuffer();
	void SetParameterVector();
	void BuildDisplayList(const ndBrainMatrix& input);
	void SetInputAndOutputBuffers(const ndBrainMatrix& input);

	ndBrain* m_brain;
	ndBrainGpuContext* m_context;

	ndBrainLayer::ndBufferOffsetPair m_paramBuffer;
	ndBrainLayer::ndBufferOffsetPair m_inputBuffer;
	ndBrainLayer::ndBufferOffsetPair m_outputBuffer;
	ndBrainLayer::ndBufferOffsetPair m_workingBuffer;

	ndList<ndSharedPtr<ndBrainGpuCommand>> m_displayList;
	ndInt32 m_inputBatchSize;
};


#endif

#endif