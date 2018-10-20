/****************************************************************************************
 
   Copyright (C) 2012 Autodesk, Inc.
   All rights reserved.
 
   Use of this software is subject to the terms of the Autodesk license agreement
   provided at the time of installation or download, or which otherwise accompanies
   this software in either electronic or hard copy form.
 
****************************************************************************************/

//! \file fbxprogress.h
#ifndef _FBXSDK_FILEIO_PROGRESS_H_
#define _FBXSDK_FILEIO_PROGRESS_H_

#include <fbxsdk/fbxsdk_def.h>

#include <fbxsdk/core/base/fbxstring.h>

#include <fbxsdk/fbxsdk_nsbegin.h>

#define FBXSDK_PROGRESS_UPDATE_THRESHOLD 0.5f

typedef bool (*FbxProgressCallback)(float pPercentage, FbxString pStatus);

class FbxSpinLock;

/** Class for progress reporting
* \nosubgrouping
*/
class FBXSDK_DLL FbxProgress
{
public:
    FbxProgress();
    ~FbxProgress();
    
    /** Set the total amount of workload.
    * \param pTotal         Total amount of workload
    */
    void SetTotal(float pTotal);

    /** Report recent finished workload.
    * \param pDetaWorkload  Amount of workload
    * \param pStatus        Current progress status
    */
    void Add(float pDetaWorkload, const FbxString & pStatus);

    //! Reset the progress status.
    void Reset();

    /** Set the progress status to completed.
    * \param pStatus        Current progress status
    */
    void Complete(const FbxString & pStatus);

    /** Retrieve the progress status.
    * \param pStatus        Current progress status
    * \return               Percentage
    */
    float GetProgress(FbxString & pStatus) const;

    /** Register a callback function for progress reporting in single thread mode.
    * \param pCallback       Pointer of the callback function
    */
    void SetProgressCallback(FbxProgressCallback pCallback);

    /** Set whether this progress was canceled by users.
    * \param pIsCanceled       Set to \c true if canceled, \c false otherwise.
    */
    void Cancel(bool pIsCanceled)
    {
        mCanceled = pIsCanceled;
    }

    //! Query whether user canceled this progress.
    bool IsCanceled() const
    {
        return mCanceled;
    }

/*****************************************************************************************************************************
** WARNING! Anything beyond these lines is for internal use, may not be documented and is subject to change without notice! **
*****************************************************************************************************************************/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
private:
    float				GetPercentage() const;

    FbxSpinLock*		mLock;
    float				mTotal;
    float				mCurrent;
    FbxString			mStatus;
    float				mUpdateThreshold;
    float				mLastCallbackProgress;
    FbxProgressCallback	mCallback;
    bool				mCanceled;
#endif /* !DOXYGEN_SHOULD_SKIP_THIS *****************************************************************************************/
};

#include <fbxsdk/fbxsdk_nsend.h>

#endif /* _FBXSDK_FILEIO_PROGRESS_H_ */
