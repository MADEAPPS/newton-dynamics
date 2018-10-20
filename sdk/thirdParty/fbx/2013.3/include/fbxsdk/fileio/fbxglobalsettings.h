/****************************************************************************************
 
   Copyright (C) 2012 Autodesk, Inc.
   All rights reserved.
 
   Use of this software is subject to the terms of the Autodesk license agreement
   provided at the time of installation or download, or which otherwise accompanies
   this software in either electronic or hard copy form.
 
****************************************************************************************/

//! \file fbxglobalsettings.h
#ifndef _FBXSDK_FILEIO_GLOBAL_SETTINGS_H_
#define _FBXSDK_FILEIO_GLOBAL_SETTINGS_H_

#include <fbxsdk/fbxsdk_def.h>

#include <fbxsdk/core/fbxobject.h>
#include <fbxsdk/scene/fbxaxissystem.h>

#include <fbxsdk/fbxsdk_nsbegin.h>

/** \brief This class contains functions for accessing global settings.
  * \nosubgrouping
  */
class FBXSDK_DLL FbxGlobalSettings : public FbxObject
{
	FBXSDK_OBJECT_DECLARE(FbxGlobalSettings, FbxObject);

public:
    /** 
	  * \name Axis system
	  */
	//@{
    
	/** Sets the scene's coordinate system.
	  * \param pAxisSystem              The coordinate system to set.
	  */
    void SetAxisSystem(const FbxAxisSystem& pAxisSystem);
    
	/** Returns the scene's current coordinate system.
	  * \return                         The scene's current coordinate system.
	  */
    FbxAxisSystem GetAxisSystem();
    //@}

    /** Sets the coordinate system's original Up Axis when the scene is created.
      * \param pAxisSystem              The coordinate system whose Up Axis is copied.
      */
    void SetOriginalUpAxis(const FbxAxisSystem& pAxisSystem);

    /** Returns the coordinate system's original Up Axis.
      * \return                         The coordinate system's original Up Axis when the scene is created. 0 is X, 1 is Y, 2 is Z axis.
      */
    int GetOriginalUpAxis() const;
    //@}

    /** 
	  * \name System Units
	  */
	//@{

	/** Sets the unit of measurement used by the system.
	  * \param pOther                   The system unit to set. 
	  */
    void SetSystemUnit(const FbxSystemUnit& pOther);
    
	/** Returns the unit of measurement used by the system.
	  * \return                         The unit of measurement used by the system.     
	  */
    FbxSystemUnit GetSystemUnit() const;

    /** Sets the original unit of measurement used by the system.
      * \param pOther                   The original system unit to set. 
      */
    void SetOriginalSystemUnit(const FbxSystemUnit& pOther);

    /** Returns the original unit of measurement used by the system.
      * \return                         The original unit of measurement used by the system.
      */
    FbxSystemUnit GetOriginalSystemUnit() const;
    //@}


    /** 
	  * \name Light Settings
	  */
	//@{

    /** Sets the ambient color.
      * \param pAmbientColor            The ambient color to set.
      * \remarks                        The ambient color only uses the RGB channels.
      */
    void SetAmbientColor(FbxColor pAmbientColor);

    /** Returns the ambient color.
      * \return                         The ambient color.
      */
    FbxColor GetAmbientColor() const;

    //@}

    /** 
	  * \name Camera Settings
	  */
	//@{

    //! Defined camera name: PRODUCER_PERSPECTIVE
    static const char* ePRODUCER_PERSPECTIVE;

    //! Defined camera name: PRODUCER_TOP
    static const char* ePRODUCER_TOP;

    //! Defined camera name: PRODUCER_FRONT
    static const char* ePRODUCER_FRONT;

    //! Defined camera name: PRODUCER_BACK
    static const char* ePRODUCER_BACK;

    //! Defined camera name: PRODUCER_RIGHT
    static const char* ePRODUCER_RIGHT;

    //! Defined camera name: PRODUCER_LEFT
    static const char* ePRODUCER_LEFT;

    //! Defined camera name: PRODUCER_BOTTOM
    static const char* ePRODUCER_BOTTOM;

    /** Sets the default camera.
      * \param pCameraName              Name of the default camera.
      * \return                         \c true if camera name is valid, returns \c false if the camera does not have a valid name.
      * \remarks                        A valid camera name can be either one of the defined tokens (PRODUCER_PERSPECTIVE,
      *                                 PRODUCER_TOP, PRODUCER_FRONT, PRODUCER_BACK, PRODUCER_RIGHT, PRODUCER_LEFT and PRODUCER_BOTTOM) or the name
      *                                 of a camera inserted in the node tree under the scene's root node.
      */
    bool SetDefaultCamera(const char* pCameraName);

    /** Returns the default camera name.
      * \return                         The default camera name, or an empty string if no camera name has been set.
      */
    FbxString GetDefaultCamera() const;
    //@}

    /** 
	  * \name Time Settings
	  */
	//@{
    /** Sets the time mode.
    * \param pTimeMode                  One of the defined modes in class FbxTime.
    */
    void SetTimeMode(FbxTime::EMode pTimeMode);

    /** Returns the time mode.
    * \return                           The currently set TimeMode.
    */
    FbxTime::EMode GetTimeMode() const;

    /** Sets the default time span of the time line.
    * \param pTimeSpan                  The default time span of the time line.
    */
    void SetTimelineDefaultTimeSpan(const FbxTimeSpan& pTimeSpan);

    /** Returns the default time span of the time line.
    * \param pTimeSpan                  The default time span of the time line.
    */
    void GetTimelineDefaultTimeSpan(FbxTimeSpan& pTimeSpan) const;

    /** Set custom frame rate.
     *  This is meaningless if the time mode is not FbxTime::eCustom.
     */
    void SetCustomFrameRate(double pCustomFrameRate);

    /** Return frame rate if the time mode is FbxTime::eCustom.
     *  If the time mode is not FbxTime::eCustom, return -1.
     */
    double GetCustomFrameRate() const; 
    //@}

/*****************************************************************************************************************************
** WARNING! Anything beyond these lines is for internal use, may not be documented and is subject to change without notice! **
*****************************************************************************************************************************/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
	virtual FbxObject& Copy(const FbxObject& pObject);

protected:
	FbxPropertyT<FbxInt>	UpAxis;
	FbxPropertyT<FbxInt>	UpAxisSign;

	FbxPropertyT<FbxInt>	FrontAxis;
	FbxPropertyT<FbxInt>	FrontAxisSign;

	FbxPropertyT<FbxInt>	CoordAxis;
	FbxPropertyT<FbxInt>	CoordAxisSign;

    FbxPropertyT<FbxInt>	OriginalUpAxis;
    FbxPropertyT<FbxInt>	OriginalUpAxisSign;

	FbxPropertyT<FbxDouble>	UnitScaleFactor;
    FbxPropertyT<FbxDouble>	OriginalUnitScaleFactor;

    FbxPropertyT<FbxDouble3>   AmbientColor;
    FbxPropertyT<FbxString>    DefaultCamera;
    FbxPropertyT<FbxEnum>      TimeMode;
    FbxPropertyT<FbxTime>      TimeSpanStart;
    FbxPropertyT<FbxTime>      TimeSpanStop;
    FbxPropertyT<FbxDouble>   CustomFrameRate;

protected:
	virtual void Construct(const FbxGlobalSettings* pFrom);
	virtual void ConstructProperties(bool pForceSet);
	
private:
    void AxisSystemToProperties();
    void PropertiesToAxisSystem();

    void Init();

    FbxAxisSystem mAxisSystem;
#endif /* !DOXYGEN_SHOULD_SKIP_THIS *****************************************************************************************/
};

inline EFbxType FbxTypeOf(const FbxTime::EMode&){ return eFbxEnum; }

#include <fbxsdk/fbxsdk_nsend.h>

#endif /* _FBXSDK_FILEIO_GLOBAL_SETTINGS_H_ */
