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

#ifndef _ND_BRAIN_AGENT_H__
#define _ND_BRAIN_AGENT_H__

class ndBrainSave;

class ndBrainAgent: public ndClassAlloc
{
	public: 
	ndBrainAgent(const ndSharedPtr<ndBrain>& brain);
	ndBrainAgent(const ndBrainAgent& src);
	virtual ~ndBrainAgent();

	virtual void Step() = 0;
	virtual void OptimizeStep() = 0;

	ndSharedPtr<ndBrain>& GetBrain(); 
	void SetBrain(const ndSharedPtr<ndBrain>& brain);

	const ndString& GetName() const;
	void SetName(const ndString& name);
	void SaveToFile(const char* const filename);

	virtual void InitWeights() = 0;
	virtual bool IsTrainer() const = 0;
	
	protected:
	virtual void ResetModel() = 0;
	virtual bool IsTerminal() const = 0;
	virtual ndBrainFloat CalculateReward() = 0;
	virtual ndInt32 GetEpisodeFrames() const = 0;
	virtual void Save(ndBrainSave* const loadSave) = 0;
	virtual void ApplyActions(ndBrainFloat* const actions)= 0;
	virtual void GetObservation(ndBrainFloat* const observation) = 0;

	ndString m_name;
	ndSharedPtr<ndBrain> m_brain;
};

#endif 

