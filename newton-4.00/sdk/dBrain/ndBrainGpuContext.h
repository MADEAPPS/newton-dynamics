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
#ifndef __ND_BRAIN_GPU_CONTEXT_H__
#define __ND_BRAIN_GPU_CONTEXT_H__

class ndBrainGpuFloatBuffer;

class ndBrainGpuContext: public ndClassAlloc
{
	class ndImplementation;

	public:
	ndBrainGpuContext();
	virtual ~ndBrainGpuContext();

	virtual ndInt32 Init();

	void* GetDevice() const;
	void* GetAllocator() const;
	void* GetPhysicalDevice() const;
	

	void ExecuteTest(ndBrainGpuFloatBuffer& buffer);

	private:
	ndImplementation* m_context;
};

#endif