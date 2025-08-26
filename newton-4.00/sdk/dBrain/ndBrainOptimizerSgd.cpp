/* Copyright (c) <2003-2022> <Julio Jerez, Newton Game Dynamics>
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

#include "ndBrainStdafx.h"
#include "ndBrain.h"
#include "ndBrainMatrix.h"
#include "ndBrainThreadPool.h"
#include "ndBrainFloatBuffer.h"
#include "ndBrainOptimizerSgd.h"

ndBrainOptimizerSgd::ndBrainOptimizerSgd(const ndSharedPtr<ndBrainContext>& context)
	:ndBrainOptimizer(context)
	,m_vdw()
	,m_weightsAndBiasBuffer(nullptr)
	,m_weightsAndBiasGradientBuffer(nullptr)
	,m_blendFactor(ndBrainFloat(1.0f - 0.99f))
	,m_miniBatchScale(ndBrainFloat(1.0f))
{
}

void ndBrainOptimizerSgd::Init(ndInt32 minibatchSize, ndBrainFloatBuffer& weightsAndBiasBuffer, ndBrainFloatBuffer& weightsAndBiasGradientBuffer)
{
	ndInt32 sizeInFloats = ndInt32(weightsAndBiasBuffer.SizeInBytes() / sizeof(ndReal));
	m_miniBatchScale = ndBrainFloat(1.0f) / ndBrainFloat(minibatchSize);

	ndBrainVector buffer;
	buffer.SetCount(sizeInFloats);
	buffer.Set(ndReal(0.0f));
	m_vdw = ndSharedPtr<ndBrainFloatBuffer>(new ndBrainFloatBuffer(*m_context, buffer));

	m_weightsAndBiasBuffer = &weightsAndBiasBuffer;
	m_weightsAndBiasGradientBuffer = &weightsAndBiasGradientBuffer;
}

void ndBrainOptimizerSgd::ApplyLearnRate(ndBrainFloat learnRate)
{
	// TO DO remenet to add the reqularizer part
	m_weightsAndBiasGradientBuffer->Scale(m_miniBatchScale);
	m_vdw->Blend(*m_weightsAndBiasGradientBuffer, m_blendFactor);
	m_weightsAndBiasGradientBuffer->Set(m_vdw);
	m_weightsAndBiasGradientBuffer->Scale(learnRate * -1.0f);
	m_weightsAndBiasBuffer->Add(*m_weightsAndBiasGradientBuffer);
}