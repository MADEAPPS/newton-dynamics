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


// CustomUserBlank.cpp: implementation of the CustomUserBlank class.
//
//////////////////////////////////////////////////////////////////////
#include "CustomJointLibraryStdAfx.h"
#include "CustomUserBlank.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#define MIN_JOINT_PIN_LENGTH   50.0f

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
   strcpy (info->m_descriptionType, "customuser");

   info->m_attachBody_0 = m_body0;
   info->m_attachBody_1 = m_body1;

   info->m_minLinearDof[0] = 0.0f;
   info->m_maxLinearDof[0] = 0.0f;

   info->m_minLinearDof[1] = 0.0f;
   info->m_maxLinearDof[1] = 0.0f;;

   info->m_minLinearDof[2] = 0.0f;
   info->m_maxLinearDof[2] = 0.0f;

   info->m_minAngularDof[0] = -FLT_MAX ;
   info->m_maxAngularDof[0] =  FLT_MAX ;
   info->m_minAngularDof[1] = -FLT_MAX ;
   info->m_maxAngularDof[1] =  FLT_MAX ;
   info->m_minAngularDof[2] = -FLT_MAX ;
   info->m_maxAngularDof[2] =  FLT_MAX ;
}


void CustomUserBlank::SubmitConstraints (dFloat timestep, int threadIndex)
{
}
