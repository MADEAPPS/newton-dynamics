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

class ndBrainGpuInference : public ndClassAlloc
{
	public:
	ndBrainGpuInference(ndBrainGpuContext* const, ndBrain* const, const ndBrainMatrix&, ndInt32) {}
	virtual ~ndBrainGpuInference() {}

	void GetResults(ndBrainVector&) {}
	void GetWorkBuffer(ndBrainVector&) {}
	const ndArray<ndInt32>& GetWorkBufferOffsets() const {return m_offsets;}
	const ndList<ndSharedPtr<ndBrainGpuCommand>>& GetDisplayList() const {return m_displayList;}

	ndArray<ndInt32>m_offsets;
	ndList<ndSharedPtr<ndBrainGpuCommand>> m_displayList;
};

#endif