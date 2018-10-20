/****************************************************************************************
 
   Copyright (C) 2012 Autodesk, Inc.
   All rights reserved.
 
   Use of this software is subject to the terms of the Autodesk license agreement
   provided at the time of installation or download, or which otherwise accompanies
   this software in either electronic or hard copy form.
 
****************************************************************************************/

//! \file fbxanimevaluator.h
#ifndef _FBXSDK_SCENE_ANIMATION_EVALUATOR_H_
#define _FBXSDK_SCENE_ANIMATION_EVALUATOR_H_

#include <fbxsdk/fbxsdk_def.h>

#include <fbxsdk/core/fbxobject.h>
#include <fbxsdk/scene/animation/fbxanimevalstate.h>
#include <fbxsdk/scene/animation/fbxanimstack.h>

#include <fbxsdk/fbxsdk_nsbegin.h>

struct FbxAnimEvaluator_internal;

/** The principal interface for animation evaluators. The animation evaluator is used to compute node transforms
  * and property values at specific times during an animation. Evaluators simplify the process of computing transform 
  * matrices by taking into account all of the parameters, such as pre- and post-rotations.
  * This class is abstract so that SDK users can implement their own evaluator if needed. The default evaluator used
  * by the FBX SDK is a FbxAnimEvalClassic. The default evaluator can be queried with the function 
  * FbxScene::GetEvaluator(), and can be changed using FbxScene::SetEvaluator().
  *
  * When working with scene nodes, the evaluator will always return an affine transform matrix that contains the
  * translation, rotation and scale of that node.
  *
  * When working with object properties, the evaluator will always return a structure that can contain as many components
  * as the property can have. For example, an RGB color property would return a structure containing 3 channels. The 
  * class FbxAnimCurveNode is used as a data container to store those values, because it can handle as many channels as 
  * needed, even if the property is not a real curve node .
  *
  * Below is a typical usage of the evaluator class to retrieve the global transform matrix of each node in a scene:
  * \code
  * //Here we assume the user already imported a scene...
  * for( int i = 0, c = MyScene->GetMemberCount(FbxNode::ClassId); i < c; ++i )
  * {
  *     FbxNode* CurrentNode = MyScene->GetMember(FbxNode::ClassId, i);
  *     FbxAMatrix& NodeGlobalTransform = MyScene->GetEvaluator()->GetNodeGlobalTransform(CurrentNode);
  * }
  *
  * //There is an equivalent call to retrieve a node's global transform, which is exactly the same as calling Scene->GetEvaluator() :
  * FbxAMatrix& NodeGlobalTransform = CurrentNode->EvaluateGlobalTransform();
  * \endcode
  *
  * Another typical usage of the evaluator class, but this time to retrieve the value of an animated color property on a material:
  * \code
  * //Assuming the user imported a scene with objects and materials...
  * FbxAnimCurveNode& Color = MyScene->GetEvaluator()->GetPropertyValue(MyMaterial->GetDiffuseColor());
  *
  * //The first parameter represent the index of the component for the property.
  * //The second parameter value of GetChannelValue is the default value in case the component is not animated.
  * float Red = Color.GetChannelValue<float>(0, 0.0f);
  * float Green = Color.GetChannelValue<float>(1, 0.0f);
  * float Blue = Color.GetChannelValue<float>(2, 0.0f);
  * \endcode
  *
  * \note Note that all the methods to retrieve global/local matrices as well as property values returns references. 
  * This is important for performance purposes, to prevent an extra memory copy.
  * \see FbxScene, FbxAnimEvalClassic, FbxAnimCurveNode
  */
class FBXSDK_DLL FbxAnimEvaluator : public FbxObject
{
    FBXSDK_ABSTRACT_OBJECT_DECLARE(FbxAnimEvaluator, FbxObject);

public:
    /** Set the evaluator context, which represent which animation stack should be evaluated. When no context is specified, the FBX SDK
	  * will try to automatically pick the first animation stack available in the scene and set it as the current context. If no animation
	  * stack are available, the evaluator will not be able to evaluate the scene's animation. Only one context can be evaluated at a time per evaluator.
      * \param pAnimStack The animation stack to evaluate when using this evaluator.
	  * \remark When changing the current context, the evaluator will automatically clear any animation evaluation cache present in memory.
	  * \see FbxAnimStack
      */
    void SetContext(FbxAnimStack* pAnimStack);

    /** Get the current evaluator context.
	  * \return The current animation stack used by the evaluator.
      * \remarks When no context is specified, the FBX SDK will try to automatically pick the first animation stack available
	  * in the scene and set it as the current context.
      */
	FbxAnimStack* GetContext();

    /** Returns a node's global transformation matrix at the specified time. The node's translation, rotation and scaling limits are taken into consideration.
      * \param pNode The node to evaluate.
      * \param pTime The time used for evaluate. If FBXSDK_TIME_INFINITE is used, this returns the default value, without animation curves evaluation.
	  * \param pPivotSet The pivot set to take into account
	  * \param pApplyTarget Applies the necessary transform to align into the target node
	  * \param pForceEval Force the evaluator to refresh the evaluation state cache even if its already up-to-date.
	  * \return The resulting global transform of the specified node at the specified time.
      */
	FbxAMatrix& GetNodeGlobalTransform(FbxNode* pNode, FbxTime pTime=FBXSDK_TIME_INFINITE, FbxNode::EPivotSet pPivotSet=FbxNode::eSourcePivot, bool pApplyTarget=false, bool pForceEval=false);

    /** Returns a node's local transformation matrix at the specified time. The node's translation, rotation and scaling limits are taken into consideration.
      * \param pNode The node to evaluate.
      * \param pTime The time used for evaluate. If FBXSDK_TIME_INFINITE is used, this returns the default value, without animation curves evaluation.
	  * \param pPivotSet The pivot set to take into account
	  * \param pApplyTarget Applies the necessary transform to align into the target node
	  * \param pForceEval Force the evaluator to refresh the evaluation state cache even if its already up-to-date.
	  * \return The resulting local transform of the specified node for the specified time.
	  * \remarks The local transform matrix is calculated in this way: ParentGlobal.Inverse * Global, all transforms such as pre/post rotation are taken into consideration.
	  * This will return a different value than LclTranslation, LclRotation and LclScaling at the specified time. To evaluate these properties separately
	  * without taking pre/post rotation, pivots and offsets into consideration, please use GetNodeLocalTranslation(), GetNodeLocalRotation() and GetNodeLocalScaling().
      */
    FbxAMatrix& GetNodeLocalTransform(FbxNode* pNode, FbxTime pTime=FBXSDK_TIME_INFINITE, FbxNode::EPivotSet pPivotSet=FbxNode::eSourcePivot, bool pApplyTarget=false, bool pForceEval=false);

	/** Returns the value of a node's LclTranslation property at the specified time. 
	  * No pivot, offsets, or any other transform is taken into consideration. The translation limit is applied.
	  * \param pNode The transform node to evaluate.
	  * \param pTime The time used for evaluate. If FBXSDK_TIME_INFINITE is used, this returns the default value, without animation curves evaluation.
	  * \param pPivotSet The pivot set to take into account
	  * \param pApplyTarget Applies the necessary transform to align into the target node
	  * \param pForceEval Force the evaluator to refresh the evaluation state cache even if its already up-to-date.
	  * \return The resulting value of LclTranslation property of the specified node at the specified time.
	  */
	FbxVector4& GetNodeLocalTranslation(FbxNode* pNode, FbxTime pTime=FBXSDK_TIME_INFINITE, FbxNode::EPivotSet pPivotSet=FbxNode::eSourcePivot, bool pApplyTarget=false, bool pForceEval=false);

	/** Returns the value of a node's LclRotation property at the specified time. 
	  * No pre/post rotation, rotation pivot, rotation offset or any other transform is taken into consideration. The rotation limit is applied.
	  * \param pNode The transform node to evaluate.
	  * \param pTime The time used for evaluate. If FBXSDK_TIME_INFINITE is used, this returns the default value, without animation curves evaluation.
	  * \param pPivotSet The pivot set to take into account
	  * \param pApplyTarget Applies the necessary transform to align into the target node
	  * \param pForceEval Force the evaluator to refresh the evaluation state cache even if its already up-to-date.
	  * \return The resulting value of LclRotation property of the specified node at the specified time.
	  */
	FbxVector4& GetNodeLocalRotation(FbxNode* pNode, FbxTime pTime=FBXSDK_TIME_INFINITE, FbxNode::EPivotSet pPivotSet=FbxNode::eSourcePivot, bool pApplyTarget=false, bool pForceEval=false);

	/** Returns the value of a node's LclScaling property at the specified time. 
	  * No scaling pivot, scaling offset or any other transform is taken into consideration. The scaling limit is applied.
	  * \param pNode The transform node to evaluate.
	  * \param pTime The time used for evaluate. If FBXSDK_TIME_INFINITE is used, this returns the default value, without animation curves evaluation.
	  * \param pPivotSet The pivot set to take into account
	  * \param pApplyTarget Applies the necessary transform to align into the target node
	  * \param pForceEval Force the evaluator to refresh the evaluation state cache even if its already up-to-date.
	  * \return The resulting value of LclScaling property of the specified node at the specified time.
	  */
	FbxVector4& GetNodeLocalScaling(FbxNode* pNode, FbxTime pTime=FBXSDK_TIME_INFINITE, FbxNode::EPivotSet pPivotSet=FbxNode::eSourcePivot, bool pApplyTarget=false, bool pForceEval=false);

	/** Get a property's value at the specified time.
      * \param pProperty The property to evaluate.
      * \param pTime The time used for evaluate.
	  * \param pForceEval Force the evaluator to refresh the evaluation state cache even if its already up-to-date.
      * \return The property value at the specified time stored in a curve node structure for easier access of complex types.
      */
    FbxAnimCurveNode& GetPropertyValue(FbxProperty& pProperty, FbxTime pTime, bool pForceEval=false);

    /** Validate if the given time value is within animation stack time span range.
      * \param pTime The time value to validate.
      * \return The new validated time, clamped by the animation stack time span range.
	  * \remarks If no animation stack are found, time zero is returned. This function is not used by the evaluator itself.
      */
    FbxTime ValidateTime(FbxTime pTime);

    /** A faster version of GetNodeGlobalTransform aimed at real-time applications. The optimization is achieved by providing an
	  * integer storage address that can be re-used for much faster access to the evaluation state cache result slot.
      * \param pNode The transform node to evaluate.
      * \param pDirectIndex The index used to retrieve the information in the evaluation state. The first time you evaluate a specific node,
	  * it must be set to -1. Then the returned index can be used for a quicker access to the evaluation state array by eliminating the search.
      * \param pTime The time used for evaluate. If FBXSDK_TIME_INFINITE is used, this returns the default value, without animation curves evaluation.
	  * \param pPivotSet The pivot set to take into account.
	  * \param pApplyTarget Applies the necessary transform to align into the target node.
	  * \param pForceEval Force the evaluator to refresh the evaluation state cache even if its already up-to-date.
	  * \return The resulting global transform of the specified node at the specified time.
      * \remarks If the scene change in the application, all direct indexes must be updated, and the evaluation state cache must be invalidated.
	  *  The translation, rotation and scaling limits are taken into consideration.
      */
	FbxAMatrix& GetNodeGlobalTransformFast(FbxNode* pNode, int& pDirectIndex, FbxTime pTime=FBXSDK_TIME_INFINITE, FbxNode::EPivotSet pPivotSet=FbxNode::eSourcePivot, bool pApplyTarget=false, bool pForceEval=false);

    /** A faster version of GetNodeLocalTransform aimed at real-time applications. The optimization is achieved by providing an
	  * integer storage address that can be re-used for much faster access to the evaluation state cache result slot.
      * \param pNode The transform node to evaluate.
      * \param pDirectIndex The index used to retrieve the information in the evaluation state. The first time you evaluate a specific node,
	  * it must be set to -1. Then the returned index can be used for a quicker access to the evaluation state array by eliminating the search.
      * \param pTime The time used for evaluate. If FBXSDK_TIME_INFINITE is used, this returns the default value, without animation curves evaluation.
	  * \param pPivotSet The pivot set to take into account.
	  * \param pApplyTarget Applies the necessary transform to align into the target node.
	  * \param pForceEval Force the evaluator to refresh the evaluation state cache even if its already up-to-date.
	  * \return The resulting local transform of the specified node for the specified time.
      * \remarks If the scene change in the application, all direct indexes must be updated, and the evaluation state cache must be invalidated.
	  * \The translation, rotation and scaling limits are taken into consideration.
	  * The local transform matrix is calculated in this way: ParentGlobal.Inverse * Global, all transforms such as pre/post rotation are taken into consideration.
	  * So to get TRS from returned matrix, it may different with values of LclTranslation, LclRotaion and LclScaling at specified time.
	  * To get values of properties LclTranslation, LclRotaion and LclScaling at the specified time, please use 
	  * GetNodeLocalTranslationFast(), GetNodeLocalRotationFast() and GetNodeLocalScalingFast().
      */
	FbxAMatrix& GetNodeLocalTransformFast(FbxNode* pNode, int& pDirectIndex, FbxTime pTime=FBXSDK_TIME_INFINITE, FbxNode::EPivotSet pPivotSet=FbxNode::eSourcePivot, bool pApplyTarget=false, bool pForceEval=false);

	/** A faster version of GetNodeLocalTranslation aimed at real-time applications. The optimization is achieved by providing an
	* integer storage address that can be re-used for much faster access to the evaluation state cache result slot.
	* \param pNode The transform node to evaluate.
	* \param pDirectIndex The index used to retrieve the information in the evaluation state. The first time you evaluate a specific node,
	* it must be set to -1. Then the returned index can be used for a quicker access to the evaluation state array by eliminating the search.
	* \param pTime The time used for evaluate.If FBXSDK_TIME_INFINITE is used, this returns the default value, without animation curves evaluation.
	* \param pPivotSet The pivot set to take into account.
	* \param pApplyTarget Applies the necessary transform to align into the target node
	* \param pForceEval Force the evaluator to refresh the evaluation state cache even if its already up-to-date.
	* \return The resulting value of LclTranslation property of the specified node at the specified time.
	* \remarks If the scene change in the application, all direct indexes must be updated, and the evaluation state cache must be invalidated.
	* Also, the translation limit is taken into consideration.
	*/
	FbxVector4& GetNodeLocalTranslationFast(FbxNode* pNode, int& pDirectIndex, FbxTime pTime=FBXSDK_TIME_INFINITE, FbxNode::EPivotSet pPivotSet=FbxNode::eSourcePivot, bool pApplyTarget=false, bool pForceEval=false);
  
	/** A faster version of GetNodeLocalRotation aimed at real-time applications. The optimization is achieved by providing an
	* integer storage address that can be re-used for much faster access to the evaluation state cache result slot.
	* \param pNode The transform node to evaluate.
	* \param pDirectIndex The index used to retrieve the information in the evaluation state. The first time you evaluate a specific node,
	* it must be set to -1. Then the returned index can be used for a quicker access to the evaluation state array by eliminating the search.
	* \param pTime The time used for evaluate.If FBXSDK_TIME_INFINITE is used, this returns the default value, without animation curves evaluation.
	* \param pPivotSet The pivot set to take into account.
	* \param pApplyTarget Applies the necessary transform to align into the target node.
	* \param pForceEval Force the evaluator to refresh the evaluation state cache even if its already up-to-date.
	* \return The resulting local rotation of the specified node at the specified time.
	* \remarks Only calculate value of LclRotation property, no pre/post rotation, rotation pivot, rotation offset or any other transform are taken into account.
	* If the scene change in the application, all direct indexes must be updated, and the evaluation state cache must be invalidated.
	* Also, the rotation limit is taken into consideration.
	*/
	FbxVector4& GetNodeLocalRotationFast(FbxNode* pNode, int& pDirectIndex, FbxTime pTime=FBXSDK_TIME_INFINITE, FbxNode::EPivotSet pPivotSet=FbxNode::eSourcePivot, bool pApplyTarget=false, bool pForceEval=false);
  
	/** A faster version of GetNodeLocalScaling aimed at real-time applications. The optimization is achieved by providing an
	* integer storage address that can be re-used for much faster access to the evaluation state cache result slot.
	* \param pNode The transform node to evaluate.
	* \param pDirectIndex The index used to retrieve the information in the evaluation state. The first time you evaluate a specific node,
	* it must be set to -1. Then the returned index can be used for a quicker access to the evaluation state array by eliminating the search.
	* \param pTime The time used for evaluate.If FBXSDK_TIME_INFINITE is used, this returns the default value, without animation curves evaluation.
	* \param pPivotSet The pivot set to take into account.
	* \param pApplyTarget Applies the necessary transform to align into the target node.
    * \param pForceEval Force the evaluator to refresh the evaluation state cache even if its already up-to-date.
	* \return The resulting local scaling of the specified node at the specified time.
	* \remarks Only calculate value of LclScaling property, no scaling pivot, scaling offset or any other transform are taken into account.
	* If the scene change in the application, all direct indexes must be updated, and the evaluation state cache must be invalidated.
	* Also, the scaling limit is taken into consideration.
	*/
	FbxVector4& GetNodeLocalScalingFast(FbxNode* pNode, int& pDirectIndex, FbxTime pTime=FBXSDK_TIME_INFINITE, FbxNode::EPivotSet pPivotSet=FbxNode::eSourcePivot, bool pApplyTarget=false, bool pForceEval=false);

    /** A faster version of GetPropertyValue aimed at real-time applications. The optimization is achieved by providing an
	  * integer storage address that can be re-used for much faster access to the evaluation state cache result slot.
      * \param pProperty The property to evaluate.
      * \param pDirectIndex The index used to retrieve the information in the evaluation state. The first time you evaluate a specific property,
	  *                     it must be set to -1. Then the returned index can be used for a quicker access to the evaluation state array.
	  * \param pTime The time used for evaluate.If FBXSDK_TIME_INFINITE is used, this returns the default value, without animation curves evaluation.
	  * \param pForceEval Force the evaluator to refresh the evaluation state cache even if its already up-to-date.
      * \return The property value at the specified time stored in a curve node structure for easier access of complex types.
      * \remarks If the scene change in the application, all direct indexes must be updated, and the evaluation state cache must be invalidated.
      */
    FbxAnimCurveNode& GetPropertyValueFast(FbxProperty& pProperty, int& pDirectIndex, FbxTime pTime=FBXSDK_TIME_INFINITE, bool pForceEval=false);

	/** Completely reset the evaluation state cache by deleting all entries. This reset also happens when changing the current context. This
	  * function is very useful in case the scene structure change, as noted in the remarks of fast functions.
	  */
	void ResetEvaluationState();

    /** Clears the specified node evaluation state cache, so the next time the evaluation is called for this node it get refreshed.
      * \param pNode The node that needs to be re-evaluated in next evaluation.
      */
	void InvalidateNode(FbxNode* pNode);

    /** Clears the specified property evaluation state cache, so the next time the evaluation is called for this property it get refreshed.
      * \param pProperty The property that needs to be re-evaluated in next evaluation.
      */
	void InvalidateProperty(FbxProperty& pProperty);

	/** Compute node local TRS from global transform. Doesn't change cached state for current time.
	* \return pRetLT Computed local translation.
	* \return pRetLR Computed local rotation.
	* \return pRetLS Computed local scaling.
	* \param pNode The transform node to evaluate.
	* \param pDirectIndex The index used to retrieve the information in the evaluation state. The first time you evaluate a specific node,
	*                     it must be set to -1. Then the returned index can be used for a quicker access to the evaluation state array by eliminating the search.
	* \param pGX Global transformation state.
	* \param pTime The time used for evaluate.If FBXSDK_TIME_INFINITE is used, this returns the default value, without animation curves evaluation.
	* \param pPivotSet The pivot set to take into account.
	* \param pApplyTarget Applies the necessary transform to align into the target node.
	* \param pForceEval Force the evaluator to refresh the evaluation state cache even if its already up-to-date.
	* \remarks If the scene change in the application, all direct indexes must be updated, and the evaluation state cache must be invalidated.
	*/
	void ComputeLocalTRSFromGlobal(FbxVector4& pRetLT, FbxVector4& pRetLR, FbxVector4& pRetLS, FbxNode* pNode, int& pDirectIndex, FbxAMatrix& pGX, FbxTime pTime=FBXSDK_TIME_INFINITE, FbxNode::EPivotSet pPivotSet=FbxNode::eSourcePivot, bool pApplyTarget=false, bool pForceEval=false);

/*****************************************************************************************************************************
** WARNING! Anything beyond these lines is for internal use, may not be documented and is subject to change without notice! **
*****************************************************************************************************************************/
#ifndef DOXYGEN_SHOULD_SKIP_THIS
protected:
	virtual void Construct(const FbxAnimEvaluator* pFrom);
    virtual void Destruct(bool pRecursive);

    virtual void EvaluateNodeTransform(FbxNodeEvalState* pResult, FbxNode* pNode, FbxTime pTime, FbxAnimStack* pStack, FbxNode::EPivotSet pPivotSet, bool pApplyTarget) = 0;
    virtual void EvaluatePropertyValue(FbxAnimCurveNode* pResult, FbxProperty& pProperty, FbxTime pTime, FbxAnimStack* pStack) = 0;

	FbxAnimEvalState*	GetEvalState(FbxTime pTime);
	FbxNodeEvalState*	GetNodeEvalState(FbxNode* pNode, int& pDirectIndex, FbxTime pTime, FbxNode::EPivotSet pPivotSet, bool pApplyTarget, bool pForceEval);

private:
	FbxAnimEvalState*	mEvalState;
	FbxAnimStack*		mAnimStack;
#endif /* !DOXYGEN_SHOULD_SKIP_THIS *****************************************************************************************/
};

#include <fbxsdk/fbxsdk_nsend.h>

#endif /* _FBXSDK_SCENE_ANIMATION_EVALUATOR_H_ */
