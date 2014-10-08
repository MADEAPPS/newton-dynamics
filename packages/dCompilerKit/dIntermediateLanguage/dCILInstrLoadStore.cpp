/* Copyright (c) <2009> <Newton Game Dynamics>
*
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
*
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "dCILstdafx.h"
#include "dCIL.h"
#include "dCILInstr.h"
#include "dCILInstrLoadStore.h"


dCILInstrLoad::dCILInstrLoad (dCIL& program, const dString& name, const dArgType& type)
	:dCILInstr (program)
	,m_source (name, type)
{
	m_destination.m_label = program.NewTemp();
	m_destination.SetType (m_source);
}

const dCILInstr::dArg& dCILInstrLoad::GetResult() const
{
	return m_destination;
}

void dCILInstrLoad::Serialize(char* const textOut) const
{
	sprintf (textOut, "\t%s %s = [%s %s]\n", m_destination.GetTypeName().GetStr(), m_destination.m_label.GetStr(), m_source.GetTypeName().GetStr(), m_source.m_label.GetStr());
}
