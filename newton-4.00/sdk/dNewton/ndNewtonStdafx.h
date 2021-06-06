/* Copyright (c) <2003-2021> <Julio Jerez, Newton Game Dynamics>
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

// stdafx.h : include file for standard system include files,
//  or project specific include files that are used frequently, but
//      are changed infrequently
//

#ifndef _D_NEWTON_STDAFX_H__
#define _D_NEWTON_STDAFX_H__

#include <dCore.h>
#include <ndCollision.h>

#ifdef _D_NEWTON_DLL
	#ifdef _D_NEWTON_EXPORT_DLL
		#define D_NEWTON_API D_LIBRARY_EXPORT
	#else
		#define D_NEWTON_API D_LIBRARY_IMPORT
	#endif
#else
	#define D_NEWTON_API 
#endif

#endif 

