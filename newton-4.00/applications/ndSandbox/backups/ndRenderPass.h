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
#ifndef __ND_RENDER_PASS_H__
#define __ND_RENDER_PASS_H__

class ndDemoEntity;
class ndDemoEntityManager;

class ndRenderPass: public ndClassAlloc
{
	public:
	ndRenderPass();
	virtual ~ndRenderPass();

	virtual void RenderScene(ndFloat32 timestep) = 0;

	virtual void Cleanup();
	virtual void Init(ndDemoEntityManager* const manager, ndInt32 arguments, ...);

	protected:
	ndDemoEntityManager* m_manager;
};

#endif