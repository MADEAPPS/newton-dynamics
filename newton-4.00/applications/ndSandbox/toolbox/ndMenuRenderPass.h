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
#ifndef __ND_MENU_RENDER_PASS_H__
#define __ND_MENU_RENDER_PASS_H__

class ndDemoEntityManager;

class ndMenuRenderPass: public ndRenderPassGui
{
	public:
	ndMenuRenderPass(ndDemoEntityManager* const owner);
	virtual ~ndMenuRenderPass();

	void RenderScene() override;

	ndDemoEntityManager* m_owner;
};

#endif