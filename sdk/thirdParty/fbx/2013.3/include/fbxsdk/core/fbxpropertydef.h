/****************************************************************************************
 
   Copyright (C) 2012 Autodesk, Inc.
   All rights reserved.
 
   Use of this software is subject to the terms of the Autodesk license agreement
   provided at the time of installation or download, or which otherwise accompanies
   this software in either electronic or hard copy form.
 
****************************************************************************************/

//! \file fbxpropertydef.h
#ifndef _FBXSDK_CORE_PROPERTY_DEFINITION_H_
#define _FBXSDK_CORE_PROPERTY_DEFINITION_H_

#include <fbxsdk/fbxsdk_def.h>

#include <fbxsdk/fbxsdk_nsbegin.h>

#define FBXSDK_PROPERTY_ID_NULL	-1
#define FBXSDK_PROPERTY_ID_ROOT 0

class FBXSDK_DLL FbxPropertyAttr
{
public:
	enum EInheritType
	{
		eOverride,
		eInherit,
		eDeleted
	};

	enum EFlags
	{
		eNone = 0,				//!< No flag.
		eAnimatable = 1,		//!< This property can be animated.
		eUser = 1<<1,			//!< This property is user-defined.
		eTemporary = 1<<2,		//!< System property.
		ePublished = 1<<3,		//!< This property has been connected to an animation curve node.
		eStatic = 1<<4,
		eNotSavable = 1<<5,		//!< This property will not be written to FBX file.
		eHidden = 1<<6,			//!< This property is hidden in UI.
		eUIDisabled = 1<<7,		//!< for dynamic UI
		eUIGroup = 1<<8,		//!< for dynamic UI
		eUIBoolGroup = 1<<9,	//!< for dynamic UI
		eUIExpanded = 1<<10,	//!< for dynamic UI
		eUINoCaption = 1<<11,	//!< for dynamic UI
		eUIPanel = 1<<12,		//!< for dynamic UI
		eUILeftLabel = 1<<13,	//!< for dynamic UI
		eUIHidden = 1<<14,		//!< for dynamic UI
        eLockedMember0 = 1 << 15,    //!< This property has its member #0 locked.
        eLockedMember1 = 1 << 16,    //!< This property has its member #1 locked.
        eLockedMember2 = 1 << 17,    //!< This property has its member #2 locked.
        eLockedMember3 = 1 << 18,    //!< This property has its member #3 locked.
        eLockedAll = ( eLockedMember0 | eLockedMember1 | eLockedMember2 | eLockedMember3 ),
        eMutedMember0 = 1 << 19,     //!< This property has its member #0 muted.
        eMutedMember1 = 1 << 20,     //!< This property has its member #1 muted.
        eMutedMember2 = 1 << 21,     //!< This property has its member #2 muted.
        eMutedMember3 = 1 << 22,     //!< This property has its member #3 muted.
        eMutedAll = ( eMutedMember0 | eMutedMember1 | eMutedMember2 | eMutedMember3 ),
		eFlagCount = 24,
		eAllFlags = (1<<(eFlagCount-1))-1,
	};

    static const int sLockedMembersMax       = 4;     //!< Maximum number of property sub-member that can be locked.
    static const int sLockedMembersBitOffset = 15;    //!< Number of bits to shift to get to the first locked member flag.
    static const int sMutedMembersMax        = 4;     //!< Maximum number of property sub-member that can be muted.
    static const int sMutedMembersBitOffset  = 19;    //!< Number of bits to shift to get to the first muted member flag.
};

#include <fbxsdk/fbxsdk_nsend.h>

#endif /* _FBXSDK_CORE_PROPERTY_DEFINITION_H_ */
