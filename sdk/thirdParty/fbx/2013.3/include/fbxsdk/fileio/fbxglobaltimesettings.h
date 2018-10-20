/****************************************************************************************
 
   Copyright (C) 2012 Autodesk, Inc.
   All rights reserved.
 
   Use of this software is subject to the terms of the Autodesk license agreement
   provided at the time of installation or download, or which otherwise accompanies
   this software in either electronic or hard copy form.
 
****************************************************************************************/

//! \file fbxglobaltimesettings.h
#ifndef _FBXSDK_FILEIO_GLOBAL_TIME_SETTINGS_H_
#define _FBXSDK_FILEIO_GLOBAL_TIME_SETTINGS_H_

#include <fbxsdk/fbxsdk_def.h>

#include <fbxsdk/core/base/fbxstring.h>
#include <fbxsdk/core/base/fbxtime.h>

#include <fbxsdk/fbxsdk_nsbegin.h>

class FbxError;
class FbxGlobalTimeSettingsProperties;

/** This class contains functions for accessing global time settings.
  * \nosubgrouping
  * \remarks This class exists for FBX version 6.x and earlier. The new FBX v7.x file format that is 
  * now the default no longer uses it. The relevant data (a subset of this class) has been moved to
  * the FbxGlobalSettings object and should be used instead.
  */
class FBXSDK_DLL FbxGlobalTimeSettings
{

public:
    FBXSDK_FRIEND_NEW();

    //! Restores the default settings.
    void RestoreDefaultSettings();

    /** Sets the time mode.
      * \param pTimeMode            One of the defined modes in FbxTime class.
      */
    void SetTimeMode(FbxTime::EMode pTimeMode);

    /** Returns the time mode.
      * \return                     The currently set time mode.
      */
    FbxTime::EMode GetTimeMode() const;

    /** Sets the time protocol.
      * \param pTimeProtocol        One of the defined protocols in FbxTime class.
      */
    void SetTimeProtocol(FbxTime::EProtocol pTimeProtocol);

    /** Returns the time protocol.
      * \return                     The currently set time protocol.
      */
    FbxTime::EProtocol GetTimeProtocol() const;

    /** \enum ESnapOnFrameMode      Snap on frame mode.
      */
    enum ESnapOnFrameMode
    {
        eNoSnap,			//! No snap.
        eSnapOnFrame,		//! Snap on frame.
        ePlayOnFrame,		//! Play on frame.
        eSnapAndPlayOnFrame	//! Snap and play on frame.
    };

    /** Sets the snap on frame mode.
      * \param pSnapOnFrameMode     One of the following values: eNoSnap, eSnapOnFrame, ePlayOnFrame, or eSnapAndPlayOnFrame.
      */
    void SetSnapOnFrameMode(ESnapOnFrameMode pSnapOnFrameMode);

    /** Returns the snap on frame mode.
      * \return                     The currently set snap on frame mode.
      */
    ESnapOnFrameMode GetSnapOnFrameMode() const;

    /**
      * \name Time span of time line 
      */
    //@{

    /** Sets the default time span of time line.
      * \param pTimeSpan            The time span of time line.
      */
    void SetTimelineDefautTimeSpan(const FbxTimeSpan& pTimeSpan);

    /** Returns the default time span of time line.
      * \param pTimeSpan            Holds the default time span of time line.
      */
    void GetTimelineDefautTimeSpan(FbxTimeSpan& pTimeSpan) const;

    //@}

    /**
      * \name Time Markers
      */
    //@{

    /** This is a struct to define time markers.
      */
    struct FBXSDK_DLL TimeMarker
    {
        //! Default constructor.
        TimeMarker();

        /** Copy constructor.
          * \param pTimeMarker      Another time marker copied to this time marker.
          */
        TimeMarker(const TimeMarker& pTimeMarker);

        /** Assignment operator.
          * \param pTimeMarker      Another time marker assigned to this time marker.
          */
        TimeMarker& operator=(const TimeMarker& pTimeMarker);

        //! Marker name.
        FbxString mName; 

        //! Marker time.
        FbxTime mTime; 

        //! Loop flag.
        bool mLoop; 
    };

    /** Returns the number of time markers.
      * \return                     The number of time markers.
      */
    int GetTimeMarkerCount() const;

    /** Sets the index of the current time marker.
      * \param pIndex               The current time marker index.
      * \return                     \c True if successful, or returns \c false if the index is not valid.
      * \remarks                    If the index is not valid, FbxGlobalTimeSettings::GetLastErrorID() returns eIndexOutOfRange.
      */
    bool SetCurrentTimeMarker(int pIndex);

    /** Returns the current time marker index.
      * \return                     The current time marker index, or -1 if no current time marker has been set.
      */
    int GetCurrentTimeMarker() const;

    /** Returns the time marker at the given index.
      * \param pIndex               The time marker index.
      * \return                     A pointer to the time marker at the given index, or \c NULL if the index is out of range.
      * \remarks                    If the index is out of range, FbxGlobalTimeSettings::GetLastErrorID() returns eIndexOutOfRange.
      */
    TimeMarker* GetTimeMarker(int pIndex) const;

    /** Adds a time marker.
      * \param pTimeMarker          The new time marker to be added.
      */
    void AddTimeMarker(TimeMarker pTimeMarker);

    //! Removes all time markers and sets the current time marker index to -1.
    void RemoveAllTimeMarkers();

    //@}

    /** Assignment operator.
	  * \param pGlobalTimeSettings  FbxGlobalTimeSettings object assigned to this one.
	  */
    const FbxGlobalTimeSettings& operator=(const FbxGlobalTimeSettings& pGlobalTimeSettings);

    /**
      * \name Error Management
      */
    //@{

    /** Retrieves the error object.
     *  \return                     Reference to the error object.
     */
    FbxError& GetError();

    /** \enum EErrorCode                Error identifiers, most of these are only used internally.
      * - \e eIndexOutOfRange
      * - \e eErrorCount
      */
    enum EErrorCode
    {
        eIndexOutOfRange,
        eErrorCount
    };

    /** Returns the last error code.
     *  \return                     The last error code.
     */
    EErrorCode GetLastErrorID() const;

    /** Returns the last error string.
     *  \return                     Text description of the last error.
     */
    const char* GetLastErrorString() const;

    //@}

    /**
      * \name Obsolete Functions
      * These functions still work but are no longer relevant.
      */
    //@{

    /** Sets the snap on frame mode flag.
      * \param pSnapOnFrame         If \c true, snap on frame mode is set to eSnapOnFrame. If \c false, snap on frame mode is set to \c eNoSnap.
      * \remarks                    This function is replaced by FbxGlobalTimeSettings::SetSnapOnFrameMode().
      */
    void SetSnapOnFrame(bool pSnapOnFrame);

    /** Returns the snap on frame mode flag.
      * \return                     \c True if the snap on frame mode is set to either eSnapOnFrame or ePlayOnFrame, returns \c false if the snap on frame mode is set to \c eNoSnap.
      * \remarks                    This function is replaced by FbxGlobalTimeSettings::GetSnapOnFrameMode().
      */
    bool GetSnapOnFrame() const;

    //@}

/*****************************************************************************************************************************
** WARNING! Anything beyond these lines is for internal use, may not be documented and is subject to change without notice! **
*****************************************************************************************************************************/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
private:
    FbxGlobalTimeSettings();
    ~FbxGlobalTimeSettings();

    FbxGlobalTimeSettingsProperties* mPH;
#endif /* !DOXYGEN_SHOULD_SKIP_THIS *****************************************************************************************/
};

#include <fbxsdk/fbxsdk_nsend.h>

#endif /* _FBXSDK_FILEIO_GLOBAL_TIME_SETTINGS_H_ */
