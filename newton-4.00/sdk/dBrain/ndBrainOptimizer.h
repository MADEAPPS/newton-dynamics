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

#ifndef _ND_BRAIN_OPTIMIZER_H__
#define _ND_BRAIN_OPTIMIZER_H__

#include "ndBrainStdafx.h"

class ndBrainContext;
class ndBrainTrainer;
class ndBrainThreadPool;

enum ndRegularizerType
{
	m_none,
	m_ridge,
	m_lasso,
};

class ndBrainOptimizer : public ndClassAlloc
{
	public: 
	ndBrainOptimizer(const ndSharedPtr<ndBrainContext>& context);
	virtual ~ndBrainOptimizer();

	ndBrainFloat GetRegularizer() const;
	ndRegularizerType GetRegularizerType() const;

	void SetRegularizer(ndBrainFloat regularizer);
	void SetRegularizerType(ndRegularizerType type);

	virtual void ApplyLearnRate(ndBrainFloat) = 0;
	virtual void Init(ndInt32, ndBrainFloatBuffer&, ndBrainFloatBuffer&) = 0;

	protected:
	ndSharedPtr<ndBrainContext> m_context;
	ndBrainFloat m_weighDecayRegularizer;
	ndRegularizerType m_regularizerType;
	ndList<ndSharedPtr<ndBrainBufferCommand>> m_commands;
};

#endif 

