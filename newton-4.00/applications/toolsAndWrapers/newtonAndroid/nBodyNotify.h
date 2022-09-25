/* Copyright (c) <2003-2021> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#ifndef _N_BODY_NOTIFY_H_
#define _N_BODY_NOTIFY_H_

#include "ndBodyNotify.h"

class nBodyNotify : public ndBodyNotify
{
	public:
	nBodyNotify()
		:ndBodyNotify(ndVector::m_zero)
	{
	}

};


#endif 

