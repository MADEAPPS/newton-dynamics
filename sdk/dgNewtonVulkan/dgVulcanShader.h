/* Copyright (c) <2003-2019> <Julio Jerez, Newton Game Dynamics>
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

#ifndef _DG_VULCAN_SHADER_H_
#define _DG_VULCAN_SHADER_H_


#include "dgPhysicsStdafx.h"




// ******************************************************************************
//
// GPU stuff start here
//
// ******************************************************************************

#define DG_GPU_WORKGROUP_SIZE		256 
#define DG_GPU_BODY_INITIAL_COUNT	4096

class dgVulkanContext;

class dgVulkanShaderInfo
{
	public:
	dgVulkanShaderInfo();
	void CreateInitBody (dgVulkanContext& context);
	void Destroy (dgVulkanContext& context);

	private:
	VkShaderModule CreateShaderModule (dgVulkanContext& context, const char* const shaderName) const;

	VkPipeline m_pipeLine;
	VkShaderModule m_module;
	VkDescriptorSet m_descriptorSet;
	VkDescriptorSetLayout m_layout;
	VkPipelineLayout m_pipelineLayout;
	VkDescriptorPool m_descriptolPool;
};


#endif
