/****************************************************************************************
 
   Copyright (C) 2012 Autodesk, Inc.
   All rights reserved.
 
   Use of this software is subject to the terms of the Autodesk license agreement
   provided at the time of installation or download, or which otherwise accompanies
   this software in either electronic or hard copy form.
 
****************************************************************************************/

//! \file fbxfolder.h
#ifndef _FBXSDK_CORE_BASE_FOLDER_H_
#define _FBXSDK_CORE_BASE_FOLDER_H_

#include <fbxsdk/fbxsdk_def.h>

#include <fbxsdk/core/base/fbxstring.h>

#include <fbxsdk/fbxsdk_nsbegin.h>

class FBXSDK_DLL FbxFolder
{
public:
	enum EEntryType {eRegularEntry, eFolderEntry};

    FbxFolder();
    ~FbxFolder();

    bool Open(const char *pDirName_UTF8);			// Open the first file of the directory
    bool Next();									// Get next file
    void Close();									// Close the current operation
    bool Create(const char *pDirName_UTF8);
    void Delete (const char *pDirName_UTF8);
    bool IsOpen() const;							// Is there a directory open
    bool IsEmpty(const char *pDirName_UTF8) const;  // Does the directory contains files

	/** Remove the given directory.
	  * \param pFolderPath_UTF8 The given directory which ought to be removed.
	  * \param pRecursive Decided if remove the children directories.
	  * If it is true ,remove all the content in the given directory.
	  * If it is false, just remove the files under given directory and retain the children directories.
	  * \return Return true if all the directory be removed .Otherwise, return false.
	  */
	static bool Remove(const char* pFolderPath_UTF8, bool pRecursive=true);

	//! Test if the given directory path exist.
	static bool Exist(const char* pFolderPath_UTF8);

	/** Test if all directories for the given file path exist, if they don't, they will be created.
	  * \remarks A dummy file name can be used. 
	  * \code
	  * bool b = FbxFolder::EnsureExistance("/Users/myname/mydir_a/mydir_b/filename.abc");
	  * \endcode
	  * if /Users/myname already exist only mydir_a/mydir_b will be created.
	  * \return Return true on success, false on failure.
	  */
	 static bool EnsureExistance(const char* pFilePath_UTF8);

    EEntryType GetType() const;
    FbxString Name() const;                         // Get the current filename
    char* GetExt() const;							// Get the current file extension

private:
	struct FolderImpl;
    FolderImpl* mImpl;
};

#include <fbxsdk/fbxsdk_nsend.h>

#endif /* _FBXSDK_CORE_BASE_FOLDER_H_ */
