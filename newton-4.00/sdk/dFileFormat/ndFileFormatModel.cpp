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

#include "ndFileFormatStdafx.h"
#include "ndFileFormatSave.h"
#include "ndFileFormatModel.h"

ndFileFormatModel::ndFileFormatModel()
	:ndFileFormatRegistrar(ndModel::StaticClassName())
{
}

ndFileFormatModel::ndFileFormatModel(const char* const className)
	:ndFileFormatRegistrar(className)
{
}

void ndFileFormatModel::SaveModel(ndFileFormatSave* const, nd::TiXmlElement* const parentNode, const ndModel* const)
{
	xmlCreateClassNode(parentNode, D_MODEL_CLASS, ndModel::StaticClassName());
}

ndModel* ndFileFormatModel::LoadModel(const nd::TiXmlElement* const, const ndTree<ndSharedPtr<ndBody>, ndInt32>&, const ndTree<ndSharedPtr<ndJointBilateralConstraint>, ndInt32>&)
{
	ndAssert(0);
	return nullptr;
}

void ndFileFormatModel::LoadModel(const nd::TiXmlElement* const, const ndTree<ndSharedPtr<ndBody>, ndInt32>&, const ndTree<ndSharedPtr<ndJointBilateralConstraint>, ndInt32>&, ndModel* const)
{
}