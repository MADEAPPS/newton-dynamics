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
#include "ndBrainAgent.h"
#include "ndBrainSaveLoad.h"

ndBrainAgent::ndBrainAgent(const ndSharedPtr<ndBrain>& brain)
	:ndClassAlloc()
	,m_name()
	,m_brain(brain)
{
}

ndBrainAgent::ndBrainAgent(const ndBrainAgent& src)
	:ndClassAlloc()
	,m_name(src.m_name)
	,m_brain(src.m_brain)
{
}

ndBrainAgent::~ndBrainAgent()
{
}

ndSharedPtr<ndBrain>& ndBrainAgent::GetBrain()
{
	return m_brain;
}

void ndBrainAgent::SetBrain(const ndSharedPtr<ndBrain>& brain)
{
	m_brain = brain;
}

const ndString& ndBrainAgent::GetName() const
{
	return m_name;
}

void ndBrainAgent::SetName(const ndString& name)
{
	m_name = name;
}

void ndBrainAgent::SaveToFile(const char* const)
{
	ndAssert(0);
	//class SaveAgent: public ndBrainSave
	//{
	//	public:
	//	SaveAgent(const char* const pathFilename)
	//		:ndBrainSave()
	//	{
	//		m_file = fopen(pathFilename, "wb");
	//		ndAssert(m_file);
	//	}
	//
	//	~SaveAgent()
	//	{
	//		if (m_file)
	//		{
	//			fclose(m_file);
	//		}
	//	}
	//
	//	void WriteData(const char* const data) const
	//	{
	//		if (m_file)
	//		{
	//			fprintf(m_file, "%s", data);
	//		}
	//	}
	//
	//	FILE* m_file;
	//};
	//
	//SaveAgent saveAgent(pathFilename);
	//Save(&saveAgent);
}


