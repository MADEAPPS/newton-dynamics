/****************************************************************************************
 
   Copyright (C) 2012 Autodesk, Inc.
   All rights reserved.
 
   Use of this software is subject to the terms of the Autodesk license agreement
   provided at the time of installation or download, or which otherwise accompanies
   this software in either electronic or hard copy form.
 
****************************************************************************************/

//! \file fbxerror.h
#ifndef _FBXSDK_CORE_BASE_ERROR_H_
#define _FBXSDK_CORE_BASE_ERROR_H_

#include <fbxsdk/fbxsdk_def.h>

#include <fbxsdk/core/base/fbxstring.h>

#include <fbxsdk/fbxsdk_nsbegin.h>

/** This class provides access to the last error integer ID and is used for translating
  * from integer error ID to strings. Internally a FbxError instance maintains a translation
  * table from integer IDs to strings. Note that different classes and systems in FBX
  * (e.g. FbxManager) have different sets of error codes, and maintain different
  * instances of the FbxError class. The translation table is an array of strings (char*)
  * and the index into the string table is assumed to be the error number.
  * \see FbxManager::EErrorCode
  * \see FbxManager::GetError()
  * \nosubgrouping
  */
class FBXSDK_DLL FbxError
{
public:

    //! Default constructor.
    FbxError();

    // copy constructor.
    FbxError(const FbxError& rhs);

    /** Creates a FbxError from an array of strings. An error code is treated as index into the
    * array of strings when looking up the error code.
    *   \param pStringArray The error string table in use.
    *   \param pErrorCount Number of elements in the error string table.
    */
    FbxError(const char* pStringArray [], int pErrorCount);

    //! Destructor.
    ~FbxError();

    FbxError& operator=(const FbxError& rhs);

    /** Reset the object (clears the last error) and use the received error string table.
    *   \param pStringArray The error string table in use.
    *   \param pErrorCount Number of elements in the error string table.
    */
    void Reset(const char* pStringArray [], int pErrorCount);

    /** Get the size of the error string table.
    *   \return Number of elements in the error string table.
    */
    int GetErrorCount() const;

    /** Get the error message string.
    *   \param pIndex Error index.
    *   \return Error string.
    */
    const char* GetErrorString(int pIndex) const;

    /** Set the last error ID and the last error string.
    *   \param pIndex Error index.
    *   \param pString Error string.
    *   \remarks This method will also set the last error string to the default
    *   string value contained in the error string table for this error ID.
    */
    void SetLastError(int pIndex, const char* pString);

    /** Set the last error index.
    *   \param pIndex Error index.
    *   \remarks This method will also set the last error string to the default
    *   string value contained in the error string table for this error index.
    */
    void SetLastErrorID(int pIndex);

    /** Return the last error index.
    *   \return The last error index or -1 if none is set.
    */
    int GetLastErrorID() const;

    /** Get the message string associated with the last error.
    *   \return Error string or empty string if none is set.
    */
    const char* GetLastErrorString() const;

    /** Set the message string associated with the last error.
    *   \param pString Error string.
    *   This method should be called after FbxError::SetLastErrorID()
    *   in order to customize the error string.
    */
    void SetLastErrorString(const char * pString);

    //! Reset the last error.
    void ClearLastError();

/*****************************************************************************************************************************
** WARNING! Anything beyond these lines is for internal use, may not be documented and is subject to change without notice! **
*****************************************************************************************************************************/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
private:
	int				mLastErrorID;
	int				mErrorCount;
	FbxString		mLastErrorString;
	const char**	mStringArray;
#endif /* !DOXYGEN_SHOULD_SKIP_THIS *****************************************************************************************/
};

#include <fbxsdk/fbxsdk_nsend.h>

#endif /* _FBXSDK_CORE_BASE_ERROR_H_ */
