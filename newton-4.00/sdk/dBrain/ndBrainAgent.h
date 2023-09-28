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
	ndBrainAgent();
	virtual ~ndBrainAgent();

	virtual void Step() = 0;
	virtual void OptimizeStep() = 0;

	const ndString& GetName() const;
	void SetName(const ndString& name);
	void SaveToFile(const char* const filename) const;

	virtual bool IsTrainer() const = 0;
	virtual void InitWeights() = 0;
	virtual void InitWeights(ndBrainFloat weighVariance, ndBrainFloat biasVariance) = 0;

	protected:
	virtual void ResetModel() const = 0;
	virtual bool IsTerminal() const = 0;
	virtual ndBrainFloat GetReward() const = 0;
	virtual ndBrainFloat GetCurrentValue() const = 0;
	virtual ndInt32 GetEpisodeFrames() const = 0;
	virtual void Save(ndBrainSave* const loadSave) const = 0;
	virtual void ApplyActions(ndBrainFloat* const actions) const = 0;
	virtual void GetObservation(ndBrainFloat* const state) const = 0;
	virtual void AddExploration(ndBrainFloat* const actions) const = 0;

	ndString m_name;
};

inline const ndString& ndBrainAgent::GetName() const
{
	return m_name;
}

inline void ndBrainAgent::SetName(const ndString& name)
{
	m_name = name;
}


#endif 

