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

#ifndef _ND_BRAIN_FLOAT4_H__
#define _ND_BRAIN_FLOAT4_H__

#include "ndBrainStdafx.h"

class ndBrainFloat4 
{
	public: 
	ndBrainFloat4();
	ndBrainFloat4(const ndBrainFloat4& src);
	~ndBrainFloat4();


	union
	{
		ndBrainFloat m_f[4];
		ndInt32 m_i[4];
		__m128 m_type;
		__m128i m_typeInt;
		struct
		{
			ndBrainFloat m_x;
			ndBrainFloat m_y;
			ndBrainFloat m_z;
			ndBrainFloat m_w;
		};
		struct
		{
			ndInt32 m_ix;
			ndInt32 m_iy;
			ndInt32 m_iz;
			ndInt32 m_iw;
		};
	};
};


#endif 
