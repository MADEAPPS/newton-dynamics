/****************************************************************************************
 
   Copyright (C) 2012 Autodesk, Inc.
   All rights reserved.
 
   Use of this software is subject to the terms of the Autodesk license agreement
   provided at the time of installation or download, or which otherwise accompanies
   this software in either electronic or hard copy form.
 
****************************************************************************************/

//! \file fbxmarker.h
#ifndef _FBXSDK_SCENE_GEOMETRY_MARKER_H_
#define _FBXSDK_SCENE_GEOMETRY_MARKER_H_

#include <fbxsdk/fbxsdk_def.h>

#include <fbxsdk/scene/geometry/fbxnodeattribute.h>

#include <fbxsdk/fbxsdk_nsbegin.h>

/**	This node attribute contains the properties of a marker.
  * A FbxMarker can represent a motion capture marker or a HIK IK/FK effector.
  * \nosubgrouping
  */
class FBXSDK_DLL FbxMarker : public FbxNodeAttribute
{
	FBXSDK_OBJECT_DECLARE(FbxMarker, FbxNodeAttribute);

public:
	//! Return the type of node attribute which is EType::eMarker.
	virtual FbxNodeAttribute::EType GetAttributeType() const;

	//! Reset the marker to default values.
	void Reset();

	/** \enum EType Marker types.
	  * - \e eStandard
	  * - \e eOptical
	  * - \e eEffectorFK
	  * - \e eEffectorIK
	  */
	enum EType
	{
		eStandard, 
		eOptical, 
		eEffectorFK,
		eEffectorIK
	};

	/** Set marker type.
	  * \param pType The type of marker.
	  */
	void SetType(EType pType);

	/** Get marker type.
	  * \return The type of the marker.
	  */
	EType GetType() const;

	/** \enum ELook Marker look.
	  * - \e eCube
	  * - \e eHardCross
	  * - \e eLightCross
	  * - \e eSphere
      * - \e eCapsule
      * - \e eBox
      * - \e eBone
      * - \e eCircle
      * - \e eSquare
      * - \e eStick
	  * - \e eNone
	  */
	enum ELook
	{ 
		eCube, 
		eHardCross, 
		eLightCross, 
        eSphere,
        eCapsule,
        eBox,
        eBone,
        eCircle,
        eSquare,
        eStick,
		eNone
	};
	
	/**
	  * \name Default Animation Values
	  * This set of functions provides direct access to default
	  * animation values specific to a marker. The default animation 
	  * values are found in the default take node of the associated node.
	  * Hence, these functions only work if the marker has been associated
	  * with a node.
	  */
	//@{

	/** Get default occlusion.
	  * \return 0.0 if optical marker animation is valid by default, 1.0 if it is occluded by default.
	  * \remarks This function only works if marker type is set to FbxMarker::eOptical.
	  */
	double GetDefaultOcclusion() const;

	/** Set default occlusion.
	  * \param pOcclusion 0.0 if optical marker animation is valid by default, 1.0 if it is occluded by default.
	  * \remarks This function only works if marker type is set to FbxMarker::eOptical.
	  */
	void SetDefaultOcclusion(double pOcclusion);

	/** Get default IK reach translation.
	  * \return A value between 0.0 and 100.0, 100.0 means complete IK reach.
	  * \remarks This function only works if marker type is set to FbxMarker::eEffectorIK.
	  */
	double GetDefaultIKReachTranslation() const;

	/** Set default IK reach translation.
	  * \param pIKReachTranslation A value between 0.0 and 100.0, 100.0 means complete IK reach.
	  * \remarks This function only works if marker type is set to FbxMarker::eEffectorIK.
	  */
	void SetDefaultIKReachTranslation(double pIKReachTranslation);

	/** Get default IK reach rotation.
	  * \return A value between 0.0 and 100.0, 100.0 means complete IK reach.
	  * \remarks This function only works if marker type is set to FbxMarker::eEffectorIK.
	  */
	double GetDefaultIKReachRotation() const;

	/** Set default IK reach rotation.
	  * \param pIKReachRotation A value between 0.0 and 100.0, 100.0 means complete IK reach.
	  * \remarks This function only works if marker type is set to FbxMarker::eEffectorIK.
	  */
	void SetDefaultIKReachRotation(double pIKReachRotation);

	//@}

	/**
	  * \name Obsolete functions
	  */
	//@{

	/** Get default color.
	  * \param pColor Filled with appropriate data
	  * \return Input parameter filled with appropriate data.
	  * \remarks Marker color can not be animated anymore.
	  */
	FbxColor& GetDefaultColor(FbxColor& pColor) const;

	/** Set default color.
	  * \param pColor The marker color to be set.
	  * \remarks Marker color can not be animated anymore.
	  */
	void SetDefaultColor(FbxColor& pColor);

	//@}

	/**
	  * \name Property Names
	  */
	static const char*			sLook;
    static const char*            sDrawLink;
	static const char*			sSize;
	static const char*			sShowLabel;
	static const char*			sIKPivot;

	/**
	  * \name Property Default Values
	  */
	static const ELook			sDefaultLook;
    static const FbxBool        sDefaultDrawLink;
	static const FbxDouble		sDefaultSize;
	static const FbxBool		sDefaultShowLabel;
	static const FbxDouble3		sDefaultIKPivot;

	//////////////////////////////////////////////////////////////////////////
	//
	// Properties
	//
	//////////////////////////////////////////////////////////////////////////
	
	/** This property handles the marker's look.
	  *
      * To access this property do: Look.Get().
      * To set this property do: Look.Set(ELook).
      *
	  * Default value is eCube
	  */
	FbxPropertyT<ELook> Look;
	
    /** This property handles the marker's link visibility.
    *
    * To access this property do: DrawLink.Get().
    * To set this property do: DrawLink.Set(FbxBool).
    *
    * Default value is false
    */
    FbxPropertyT<FbxBool> DrawLink;
    
	/** This property handles the marker's size.
	  *
      * To access this property do: Size.Get().
      * To set this property do: Size.Set(FbxDouble).
      *
	  * Default value is 100
	  */
	FbxPropertyT<FbxDouble> Size;
	
	/** This property handles the marker's label visibility.
	  *
      * To access this property do: ShowLabel.Get().
      * To set this property do: ShowLabel.Set(FbxBool).
      *
	  * Default value is false
	  */
	FbxPropertyT<FbxBool> ShowLabel;
	
	/** This property handles the marker's pivot position.
	  *
      * To access this property do: IKPivot.Get().
      * To set this property do: IKPivot.Set(FbxDouble3).
      *
	  * Default value is (0., 0., 0.)
	  */
	FbxPropertyT<FbxDouble3> IKPivot;

	// Dynamic properties

	/** This method grants access to the occlusion property.
	  * \remarks If the marker is not of type Optical or the property
	  * is invalid, return NULL
	  */
	FbxProperty* GetOcclusion();

	/** This method grants access to the IKReachTranslation property.
	  * \remarks If the marker is not of type IK Effector or the property
	  * is invalid, return NULL
	  */
	FbxProperty* GetIKReachTranslation();
	/** This method grants access to the IKReachRotation property.
	  * \remarks If the marker is not of type IK Effector or the property
	  * is invalid, return NULL
	  */
	FbxProperty* GetIKReachRotation();

/*****************************************************************************************************************************
** WARNING! Anything beyond these lines is for internal use, may not be documented and is subject to change without notice! **
*****************************************************************************************************************************/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
	virtual FbxObject& Copy(const FbxObject& pObject);

protected:
	virtual void Construct(const FbxMarker* pFrom);
	virtual void ConstructProperties(bool pForceSet);

	void Reset( bool pResetProperties );

	/**
	  *	Used to retrieve the KProperty list from an attribute.
	  */

	virtual const char* GetTypeName() const;
	virtual FbxStringList GetTypeFlags() const;

	EType mType;

	FbxProperty dynProp; // temporary placeholder for either
	// the Occlusion, IKReachTranslation or IKReachRotation 
	// properties. Its address is returned in the GetOcclusion(),
	// GetIKReachTranslation() and GetIKReachRotation() if the property
	// is valid
#endif /* !DOXYGEN_SHOULD_SKIP_THIS *****************************************************************************************/
};

inline EFbxType FbxTypeOf(const FbxMarker::ELook&){ return eFbxEnum; }

#include <fbxsdk/fbxsdk_nsend.h>

#endif /* _FBXSDK_SCENE_GEOMETRY_MARKER_H_ */
