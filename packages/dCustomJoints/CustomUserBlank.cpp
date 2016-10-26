/* Copyright (c) <2003-2016> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/


// CustomUserBlank.cpp: implementation of the CustomUserBlank class.
//
//////////////////////////////////////////////////////////////////////
#include "CustomJointLibraryStdAfx.h"
#include "CustomUserBlank.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


//dInitRtti(CustomUserBlank);

CustomUserBlank::CustomUserBlank(unsigned int maxDOF, NewtonBody* const child, NewtonBody* const parent)
   :CustomJoint(maxDOF, child, parent)
{
}

CustomUserBlank::~CustomUserBlank()
{
}

void CustomUserBlank::GetInfo (NewtonJointRecord* const info) const
{
   strcpy (info->m_descriptionType, GetTypeName());

   info->m_attachBody_0 = m_body0;
   info->m_attachBody_1 = m_body1;

   info->m_minLinearDof[0] = 0.0f;
   info->m_maxLinearDof[0] = 0.0f;

   info->m_minLinearDof[1] = 0.0f;
   info->m_maxLinearDof[1] = 0.0f;;

   info->m_minLinearDof[2] = 0.0f;
   info->m_maxLinearDof[2] = 0.0f;

   info->m_minAngularDof[0] = -D_CUSTOM_LARGE_VALUE ;
   info->m_maxAngularDof[0] =  D_CUSTOM_LARGE_VALUE ;
   info->m_minAngularDof[1] = -D_CUSTOM_LARGE_VALUE ;
   info->m_maxAngularDof[1] =  D_CUSTOM_LARGE_VALUE ;
   info->m_minAngularDof[2] = -D_CUSTOM_LARGE_VALUE ;
   info->m_maxAngularDof[2] =  D_CUSTOM_LARGE_VALUE ;
}


void CustomUserBlank::SubmitConstraints (dFloat timestep, int threadIndex)
{
}
