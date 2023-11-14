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
#include "ndBrainSaveLoad.h"
#include "ndBrainLayerDropOut.h"

ndBrainLayerDropOut::ndBrainLayerDropOut(ndInt32 neurons, ndBrainFloat dropFraction)
	:ndBrainLayerActivation(neurons)
	,m_droppedNeuron()
	,m_dropFraction(dropFraction)
{
	m_droppedNeuron.SetCount(neurons);
	m_droppedNeuron.Set(ndBrainFloat(1.0f));
}

ndBrainLayerDropOut::ndBrainLayerDropOut(const ndBrainLayerDropOut& src)
	:ndBrainLayerActivation(src)
	,m_droppedNeuron(src.m_droppedNeuron)
	,m_dropFraction(src.m_dropFraction)
{
}

ndBrainLayer* ndBrainLayerDropOut::Clone() const
{
	return new ndBrainLayerDropOut(*this);
}

const char* ndBrainLayerDropOut::GetLabelId() const
{
	return "ndBrainLayerDropOut";
}

void ndBrainLayerDropOut::UpdateDropOut()
{
	ndAssert(0);
}

void ndBrainLayerDropOut::InputDerivative(const ndBrainVector&, const ndBrainVector& outputDerivative, ndBrainVector& inputDerivative) const
{
	ndAssert(outputDerivative.GetCount() == outputDerivative.GetCount());
	inputDerivative.Set(m_droppedNeuron);
	inputDerivative.Mul(outputDerivative);
}

void ndBrainLayerDropOut::MakePrediction(const ndBrainVector& input, ndBrainVector& output) const
{
	ndAssert(input.GetCount() == output.GetCount());
	output.Set(input);
	output.Mul(m_droppedNeuron);
}

void ndBrainLayerDropOut::Save(const ndBrainSave* const loadSave) const
{
	ndAssert(0);
	//char buffer[1024];
	//
	//sprintf(buffer, "\tinput_width %d\n", m_width);
	//loadSave->WriteData(buffer);
	//
	//sprintf(buffer, "\tinput_heigh %d\n", m_height);
	//loadSave->WriteData(buffer);
	//
	//sprintf(buffer, "\tinput_layers %d\n", m_channels);
	//loadSave->WriteData(buffer);
}

ndBrainLayer* ndBrainLayerDropOut::Load(const ndBrainLoad* const loadSave)
{
	ndAssert(0);
	//char buffer[1024];
	//loadSave->ReadString(buffer);
	//
	//loadSave->ReadString(buffer);
	//ndInt32 inputWidth = loadSave->ReadInt();
	// 
	//loadSave->ReadString(buffer);
	//ndInt32 inputHeight = loadSave->ReadInt();
	//
	//loadSave->ReadString(buffer);
	//ndInt32 inputLayers = loadSave->ReadInt();
	//
	//ndBrainLayerDropOut* const layer = new ndBrainLayerDropOut(inputWidth, inputHeight, inputLayers);
	//loadSave->ReadString(buffer);
	//return layer;
	return nullptr;
}