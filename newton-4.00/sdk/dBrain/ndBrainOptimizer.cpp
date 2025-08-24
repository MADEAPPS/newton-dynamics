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
#include "ndBrainTrainer.h"
#include "ndBrainOptimizer.h"
#include "ndBrainThreadPool.h"

ndBrainOptimizer::ndBrainOptimizer(const ndSharedPtr<ndBrainContext>& context)
	:ndClassAlloc()
	,m_context(context)
	,m_weighDecayRegularizer(ndBrainFloat(0.0f))
	,m_regularizerType(m_ridge)
	,m_commands()
{
}

ndBrainOptimizer::~ndBrainOptimizer()
{
}

ndRegularizerType ndBrainOptimizer::GetRegularizerType() const
{
	return m_regularizerType;
}

void ndBrainOptimizer::SetRegularizerType(ndRegularizerType type)
{
	m_regularizerType = type;
}

ndBrainFloat ndBrainOptimizer::GetRegularizer() const
{
	return m_weighDecayRegularizer;
}

void ndBrainOptimizer::SetRegularizer(ndBrainFloat regularizer)
{
	m_weighDecayRegularizer = ndClamp(regularizer, ndBrainFloat(0.0f), ndBrainFloat(0.01f));
}
