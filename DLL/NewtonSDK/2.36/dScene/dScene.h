/////////////////////////////////////////////////////////////////////////////
// Name:        dScene.h
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

#ifndef _D_SCENE_H_
#define _D_SCENE_H_

#include "dSceneGraph.h"
//#include "dMaterialNodeInfo.h"

#define NE_MAX_LAYERS	8

struct NewtonWorld;
class dCameraNodeInfo;
class dRootNodeInfo;


enum neFileRevision
{
	m_currentRevision = 100,
};


class dScene: public dSceneGraph
{
public:
	dScene(NewtonWorld* newton);
	dScene(const dScene& me);
	virtual ~dScene(void);

	virtual void CleanUp();
	
	virtual dTreeNode* GetRootNode() const;

	virtual dTreeNode* CreateSceneNode(dTreeNode* const parent);
	virtual dTreeNode* CreateBoneNode(dTreeNode* const parent);
	virtual dTreeNode* CreateRigidbodyNode(dTreeNode* const parent);
	virtual dTreeNode* CreateCollisionBoxNode(dTreeNode* const parent);
	virtual dTreeNode* CreateCollisionConeNode(dTreeNode* const parent);
	virtual dTreeNode* CreateCollisionSphereNode(dTreeNode* const parent);
	virtual dTreeNode* CreateCollisionCapsuleNode(dTreeNode* const parent);
	virtual dTreeNode* CreateCollisionCylinderNode(dTreeNode* const parent);
	virtual dTreeNode* CreateCollisionCompoundNode(dTreeNode* const parent);
	virtual dTreeNode* CreateCollisionChamferCylinderNode(dTreeNode* const parent);
	virtual dTreeNode* CreateCollisionConvexHullNode(dTreeNode* const parent);
	virtual dTreeNode* CreateMeshNode(dTreeNode* const parent);
	virtual dTreeNode* CreateSkinModifierNode(dTreeNode* const parent);
	virtual dTreeNode* CreateMaterialNode (dTreeNode* const parent, int id);
	virtual dTreeNode* CreateTextureNode (const char* const pathName);

	virtual dTreeNode* CreateNode (const char* className, dTreeNode* const parent);
	virtual dTreeNode* CreateCollisionFromNewtonCollision(dTreeNode* const parent, NewtonCollision* collision);

	virtual void AddReference(dTreeNode* const parent, dTreeNode* const child);
	virtual void RemoveReference(dTreeNode* const node1, dTreeNode* const node2);

	virtual dTreeNode* GetFirstNode () const;
	virtual dTreeNode* GetNextNode (dTreeNode* const node) const;
	virtual dTreeNode* FindNode (dNodeInfo* const info) const;

	
	virtual void* GetFirstChild(dTreeNode* const parentNode) const;
	virtual void* GetNextChild(dTreeNode* const parentNode, void* const link) const;

	virtual void* GetFirstParent(dTreeNode* const childNode) const;
	virtual void* GetNextParent(dTreeNode* const childNode, void* const link) const;

	//virtual bool IsParentLink (void* const link) const;
	virtual dTreeNode* GetNodeFromLink (void* const child) const;

	virtual void CloneInfoFromNode(dTreeNode* const node);
	virtual dNodeInfo* GetInfoFromNode(dTreeNode* const node) const;
	
	
	virtual dTreeNode* FindCameraNode(int camIndex) const;
	virtual dTreeNode* FindTextureByTextId(dTreeNode* const parentNode, int textId) const;
	virtual dTreeNode* FindChildByType(dTreeNode* const parentNode, int type) const;
	virtual dTreeNode* FindParentByType(dTreeNode* const child, int type) const;

	virtual NewtonWorld* GetNewtonWorld() const {return m_newton;}

	virtual void DeleteNode (dTreeNode* const node);
//	void UnlinkNode (dTreeNode* const node);
	virtual void MergeScene (dScene* const scene);

	virtual void Serialize (const char* const fileName);
	virtual bool Deserialize (const char* const fileName);
	virtual void SerializeBinary (const char* const fileName);

	virtual void UpdateAllOOBB ();
	virtual dTreeNode* RayCast (const dVector& p0, const dVector& p1) const;

	virtual void BakeTransform (dMatrix& matrix) const;


	virtual void RemoveUnusedVertex();
	virtual void MergeDuplicateTextures();

	virtual void SetNodeLRU (dTreeNode* const node, int lru);
	virtual int GetNodeLRU (dTreeNode* const node) const;

	virtual void NewtonWorldToScene (const NewtonWorld* const world);
	virtual void SceneToNewtonWorld (NewtonWorld* world, dList<NewtonBody*>& loadedBodies);
	


	protected:
//	void BakeSceneNodeTransform (dTreeNode* neSceneNode, dMatrix& matrix) const;
//	void BakeSceneNodeTransformRecursive (dTreeNode* neSceneNode, dMatrix& matrix) const;
	dTreeNode* AddNode(dNodeInfo* const sceneInfo, dTreeNode* const parent);

	NewtonWorld* m_newton;
};


#endif