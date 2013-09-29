/////////////////////////////////////////////////////////////////////////////
// Name:        dSceneGraph.h
// Purpose:     
// Author:      Julio Jerez
// Modified by: 
// Created:     22/05/2010 08:02:08
// RCS-ID:      
// Copyright:   Copyright (c) <2010> <Newton Game Dynamics>
// License:     
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
// 
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely
/////////////////////////////////////////////////////////////////////////////

#ifndef _D_SCENE_GRAPH_H_
#define _D_SCENE_GRAPH_H_

class dNodeInfo;
class dGraphNode;
class TiXmlElement;

#define D_GRAPH_MAX_STACK_DEPTH	(1024 * 16)

class dSceneGraph: public dTree<dGraphNode, unsigned>
{
	public:
	DSCENE_API dSceneGraph(dNodeInfo* const rootInfo);
	DSCENE_API dSceneGraph(const dSceneGraph& me);

	virtual DSCENE_API ~dSceneGraph(void);

	virtual DSCENE_API dTreeNode* GetRootNode() const;
	virtual DSCENE_API void AddRootNode(dNodeInfo* const info);
	virtual DSCENE_API void DeleteRootNode ();

	virtual DSCENE_API void SetNodeInfo (dNodeInfo* const newInfo, dTreeNode* const node);
	virtual DSCENE_API dTreeNode* AddNode (dNodeInfo* const info, dTreeNode* const parent);
	virtual DSCENE_API void DeleteNode (dTreeNode* const node);

	virtual DSCENE_API void AddEdge (dTreeNode* const node1, dTreeNode* const node2);
	virtual DSCENE_API void DeleteEdge (dTreeNode* const node1, dTreeNode* const node2);

	virtual DSCENE_API void UnlinkEdge (dTreeNode* const node1, dTreeNode* const node2);
	virtual DSCENE_API bool HasLinkToRoot (dTreeNode* const node); 

	virtual DSCENE_API void Serialize (TiXmlElement* const parentNode) const;
	virtual DSCENE_API bool Deserialize (TiXmlElement* const parentNode);

	virtual DSCENE_API int GetLRU();
	virtual DSCENE_API void Cleanup();

	protected:
	int m_lru;
	dTreeNode* m_rootNode;
	friend class dGraphNode;
};


class dGraphNode: public dContainersAlloc
{
	class dLink: public dList<dSceneGraph::dTreeNode*>
	{
		friend class dScene;
		friend class dGraphNode;
		friend class dSceneGraph;
	};
	public:
	DSCENE_API dGraphNode ();
	DSCENE_API dGraphNode (const dGraphNode& node);
	virtual DSCENE_API ~dGraphNode ();

	virtual DSCENE_API dNodeInfo* GetNode() const {return m_nodeInfo;}
	virtual DSCENE_API void SetNode(dNodeInfo* const newInfo);

	virtual DSCENE_API void SetLRU (int lru) {m_lru = lru;}
	virtual DSCENE_API int GetLRU () const { return m_lru;}

	private:
	int m_lru;
	dNodeInfo* m_nodeInfo;
	dLink m_parents;
	dLink m_children;
	friend class dScene;
	friend class dSceneGraph;
};


#endif