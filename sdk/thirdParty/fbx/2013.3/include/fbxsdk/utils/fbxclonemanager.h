/****************************************************************************************
 
   Copyright (C) 2012 Autodesk, Inc.
   All rights reserved.
 
   Use of this software is subject to the terms of the Autodesk license agreement
   provided at the time of installation or download, or which otherwise accompanies
   this software in either electronic or hard copy form.
 
****************************************************************************************/

//! \file fbxclonemanager.h
#ifndef _FBXSDK_UTILS_CLONE_MANAGER_H_
#define _FBXSDK_UTILS_CLONE_MANAGER_H_

#include <fbxsdk/fbxsdk_def.h>

#include <fbxsdk/core/fbxobject.h>
#include <fbxsdk/core/fbxquery.h>

#include <fbxsdk/fbxsdk_nsbegin.h>

/** The clone manager is a utility for cloning entire networks of FbxObject.
  * Options are available for specifying how the clones inherit the connections
  * of the original.
  *
  * Networks of FbxObject (inter-connected objects by OO, OP, PO or PP connections)
  * can be cloned. How the connections of clones are handled depends on mSrcPolicy and mExternalDstPolicy.
  *
  * To clone FbxObject instances and their dependents, put them into a CloneSet
  * and pass the CloneSet to this class:
  * \code
  * FbxCloneManager                  cloneManager;
  * FbxCloneManager::CloneSet        cloneSet;
  * FbxCloneManager::CloneSetElement defaultCloneOptions(FbxCloneManager::sConnectToClone,
  *                                       FbxCloneManager::sConnectToOriginal, FbxObject::eDeepClone);
  * cloneSet.Insert(someObject, defaultCloneOptions);
  * cloneManager.AddDependents(cloneSet, someObject, defaultCloneOptions);
  * cloneManager.Clone(cloneSet, scene)
  * \endcode
  *
  * \see FbxCloneManager::CloneSetElement
  * \see FbxCloneManager::CloneSet
  * \nosubgrouping
  */
class FBXSDK_DLL FbxCloneManager
{
public:

    //! Maximum depth to clone dependents.
    static const int sMaximumCloneDepth;

    /** Connect to objects that are connected to original object.
      * This is a flag to mSrcPolicy or mExternalDstPolicy.
      */
    static const int sConnectToOriginal;

    /** Connect to clones of objects that are connected to original object.
      * (only if those original objects are also in the clone set)
      * This is a flag to mSrcPolicy.
      */
    static const int sConnectToClone;

    /** This represents an element in FbxCloneManager::CloneSet to be cloned.
      * This class contains the option for specifying how connections are cloned and the
      * cloned object.
      * \see FbxCloneManager
      * \see FbxCloneManager::CloneSet
      */
    struct FBXSDK_DLL CloneSetElement
    {
    public:
        /** Constructor.
          * \param pSrcPolicy Specify how to handle source connections. Valid values are 0, sConnectToOriginal,
          *                   sConnectToClone or sConnectToOriginal|sConnectToClone.
          * \param pExternalDstPolicy Specify how to handle destination connections to objects NOT in
          *                           the clone set. Valid values are 0 or sConnectToOriginal.
          * \param pCloneType Specify the type of cloning. FbxObject::Clone uses the same parameter.
          */
        CloneSetElement( int pSrcPolicy = 0,
                         int pExternalDstPolicy = 0,
                         FbxObject::ECloneType pCloneType = FbxObject::eReferenceClone );

        //! the type of cloning to perform
        FbxObject::ECloneType mType;

        /** Policy on how to handle source connections on the original object. Valid values are 0
          * or any bitwise OR'd combination of sConnectToOriginal, and sConnectToClone.
          */
        int mSrcPolicy;

        /** policy on how to handle destination connections on the original object to
          * objects NOT in the clone set. (Destination connections to objects in the set
          * are handled by that object's source policy) Valid values are 0 or sConnectToOriginal.
          */
        int mExternalDstPolicy;

        /** This is a pointer to the newly created clone.
          * It is set after the call to FbxCloneManager::Clone()
          */
        FbxObject* mObjectClone;
    };

    /** \brief Functor to compare object pointers. This class is used internally in FbxCloneManager.
      */
    class FBXSDK_DLL ObjectCompare
    {
    public:
        /** Operator converting instance to int type.
          * \param pKeyA Left key to be compared.
          * \param pKeyB Right key to be compared.
          */
        inline int operator()(FbxObject* const& pKeyA, FbxObject* const& pKeyB) const
        {
            return (pKeyA < pKeyB) ? -1 : ((pKeyB < pKeyA) ? 1 : 0);
        }
    };

    /** The CloneSet is a collection of pointers to objects that will be cloned in Clone()
      * Attached to each object is a CloneSetElement. Its member variables dictate how
      * the corresponding object will be cloned, and how it will inherit connections
      * on the original object.
      */
    typedef FbxMap<FbxObject*,CloneSetElement,ObjectCompare> CloneSet;

    /** Constructor
      */
    FbxCloneManager();

    /** Destructor
      */
    virtual ~FbxCloneManager();

    /** This function simplifies the process of cloning one object and all its depedency graph by automatically preparing
      * the CloneSet and calling the Clone method using the code below:
      * \code
      * FbxCloneManager                  cloneManager;
      * FbxCloneManager::CloneSet        cloneSet;
      * FbxCloneManager::CloneSetElement defaultCloneOptions(FbxCloneManager::sConnectToClone,
      *                                                      0, 
      *                                                      FbxObject::eDeepClone);
      * cloneSet.Insert(pObject, defaultCloneOptions);
      * cloneManager.AddDependents(cloneSet, pObject, defaultCloneOptions);
      * cloneManager.Clone(cloneSet, pContainer)
      * \endcode
      *
      * \param pObject Object to clone.
      * \param pContainer This object (typically a scene or document) will contain the new clones.
      * \return The clone of \e pObject if all its depedency graph have been cloned successfully, NULL otherwise.
      * \remark \e pContainer should not be an FbxNode used to group the cloned dependency graph. All the cloned objects of the graph will 
      *         also connect to \e pContainer so all the node attributes will be interpreted as instances since they will connect to, at least,
      *         two FbxNodes. Also, if \pContainer is left \c NULL the cloned object should be manually connected to the scene if it is to
      *         be saved to disk.
	  *
	  * For example:
      * \code
      *     FbxNode* lN = FbxNode::Create(lScene, "Clone");
      *     if (lN)
      *     {
      *         lN->LclTranslation.Set(FbxDouble3(50,0,0));
      *         lScene->GetRootNode()->AddChild(lN);
      *
      *         FbxObject* cn = NULL;
      *         FbxObject* n = lScene->FindSrcObject("Mesh");
      *         if (n) cn = FbxCloneManager::Clone(n, lN);
      *     }
      * \endcode
      * will generate the following (unexpected) dependency graph:
      * \code
      *             "Clone" (FbxNode) 
      *                /   \               
      *               /      \
      *    "Mesh" (FbxNode) -- mesh (FbxNodeAttribute)
      * \endcode
      * To generate the (expected) graph:
      * \code
      *             "Clone" (FbxNode) 
      *                |
      *             "Mesh" (FbxNode) 
      *                |
      *              mesh (FbxNodeAttribute)
      * \endcode
      * The correct code to use is:
      * \code
      *     FbxNode* lN = FbxNode::Create(lScene, "Clone");
      *     if (lN)
      *     {
      *         lN->LclTranslation.Set(FbxDouble3(50,0,0));
      *         lScene->GetRootNode()->AddChild(lN);
      *
      *         FbxObject* cn = NULL;
      *         FbxObject* n = lScene->FindSrcObject("Mesh");
      *         if (n) cn = FbxCloneManager::Clone(n);
      *         if (cn) lN->AddChild((FbxNode*)cn);
      *     }
      * \endcode
      */
    static FbxObject* Clone(const FbxObject* pObject, FbxObject* pContainer = NULL);

    /** Clone all objects in the set using the given policies for duplication
      * of connections. Each CloneSetElement in the set will have its mObjectClone
      * pointer set to the newly created clone.
      * \param pSet Set of objects to clone
      * \param pContainer This object (typically a scene or document) will contain the new clones
      * \return true if all objects were cloned, false otherwise.
      */
    virtual bool Clone( CloneSet& pSet, FbxObject* pContainer = NULL ) const;

    /** Add all dependents of the given object to the CloneSet.
      * Dependents of items already in the set are ignored to prevent
      * infinite recursion on cyclic dependencies.
      * \param pSet The set to add items.
      * \param pObject Object to add dependents to
	  * \param pCloneOptions  
      * \param pTypes Types of dependent objects to consider
      * \param pDepth Maximum recursive depth. Valid range is [0,sMaximumCloneDepth]
        */
    virtual void AddDependents( CloneSet& pSet,
                        const FbxObject* pObject,
                        const CloneSetElement& pCloneOptions = CloneSetElement(),
                        FbxCriteria pTypes = FbxCriteria::ObjectType(FbxObject::ClassId),
                        int pDepth = sMaximumCloneDepth ) const;

/*****************************************************************************************************************************
** WARNING! Anything beyond these lines is for internal use, may not be documented and is subject to change without notice! **
*****************************************************************************************************************************/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
protected:
	bool CloneConnections( CloneSet::RecordType* pIterator, const CloneSet& pSet ) const;
#endif /* !DOXYGEN_SHOULD_SKIP_THIS *****************************************************************************************/
};

#include <fbxsdk/fbxsdk_nsend.h>

#endif /* _FBXSDK_UTILS_CLONE_MANAGER_H_ */
