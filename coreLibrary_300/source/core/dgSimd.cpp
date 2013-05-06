/* Copyright (c) <2003-2011> <Julio Jerez, Newton Game Dynamics>
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


#include "dgStdafx.h"
#include "dgTypes.h"
#include "dgSimd.h"


dgSimd dgSimd::m_triplexMask (dgInt32 (0xffffffff), dgInt32 (0xffffffff), dgInt32 (0xffffffff), dgInt32 (0));
dgSimd dgSimd::m_signMask (dgInt32 (0x7fffffff), dgInt32 (0x7fffffff), dgInt32 (0x7fffffff), dgInt32 (0x7fffffff));
dgSimd dgSimd::m_allOneMask (dgInt32 (0xffffffff), dgInt32 (0xffffffff), dgInt32 (0xffffffff), dgInt32 (0xffffffff));
dgSimd dgSimd::m_index_0123 (dgFloat32 (0.0f), dgFloat32 (1.0f), dgFloat32 (2.0f), dgFloat32 (3.0f));
dgSimd dgSimd::m_index_4567 (dgFloat32 (4.0f), dgFloat32 (5.0f), dgFloat32 (6.0f), dgFloat32 (7.0f));
dgSimd dgSimd::m_half (dgFloat32 (0.5f), dgFloat32 (0.5f), dgFloat32 (0.5f), dgFloat32 (0.5f));
dgSimd dgSimd::m_zero (dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f), dgFloat32 (0.0f));
dgSimd dgSimd::m_one (dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (1.0f), dgFloat32 (1.0f));
dgSimd dgSimd::m_negOne (dgFloat32 (-1.0f), dgFloat32 (-1.0f), dgFloat32 (-1.0f), dgFloat32 (-1.0f));
dgSimd dgSimd::m_three (dgFloat32 (3.0f), dgFloat32 (3.0f), dgFloat32 (3.0f), dgFloat32 (3.0f));

