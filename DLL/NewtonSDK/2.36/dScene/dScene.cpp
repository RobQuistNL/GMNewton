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

#include "dPluginStdafx.h"
#include "dScene.h"
#include "dCameraNodeInfo.h"
#include "dRootNodeInfo.h"
#include "dBoneNodeInfo.h"
#include "dSceneNodeInfo.h"
#include "dMeshNodeInfo.h"
#include "dTextureNodeInfo.h"
#include "dMaterialNodeInfo.h"
#include "dRigidbodyNodeInfo.h"
#include "dCollisionBoxNodeInfo.h"
#include "dCollisionConeNodeInfo.h"
#include "dCollisionSphereNodeInfo.h"
#include "dCollisionCapsuleNodeInfo.h"
#include "dCollisionCylinderNodeInfo.h"
#include "dCollisionCompoundNodeInfo.h"
#include "dCollisionConvexHullNodeInfo.h"
#include "dGeometryNodeSkinModifierInfo.h"
#include "dCollisionChamferCylinderNodeInfo.h"

// this constructor is for the editor only
dScene::dScene(NewtonWorld* newton)
	:dSceneGraph(new dRootNodeInfo()), m_newton (newton)
{
	// create one camera for each view port
	dTreeNode* root = GetRootNode();
	for (int i = 0; i < 4; i ++) {
		dTreeNode* const node = CreateNode ("dCameraNodeInfo", root);
		dCameraNodeInfo* const camera = (dCameraNodeInfo*) GetInfoFromNode(node);

		char name[32];
		sprintf (name, "viewport%d", i);
		camera->SetName (name);
		camera->m_viewportIndex = i;
	}
}

dScene::dScene(const dScene& me)
	:dSceneGraph(me), m_newton (me.m_newton)
{
}

dScene::~dScene(void)
{
}

void dScene::CleanUp()
{
	dSceneGraph::Cleanup();
}

dScene::dTreeNode* dScene::GetRootNode() const
{
	return dSceneGraph::GetRootNode();
}

void dScene::AddReference(dTreeNode* const parent, dTreeNode* const child)
{
	 AddEdge (parent, child);
}

void dScene::RemoveReference(dTreeNode* const node1, dTreeNode* const node2)
{
	DeleteEdge (node1, node2);
}

dScene::dTreeNode* dScene::CreateNode (const char* className, dTreeNode* const parent)
{
	dTreeNode* node = NULL;
	dNodeInfo* const info = dNodeInfo::CreateFromClassName (className, this);
	if (info) {
		node = AddNode(info, parent);
		info->Release();
	}
	return node;
}


dScene::dTreeNode* dScene::CreateCollisionFromNewtonCollision(dTreeNode* const parent, NewtonCollision* collision)
{
	NewtonCollisionInfoRecord record;
	NewtonCollisionGetInfo(collision, & record);

	dNodeInfo* info = NULL;
	switch (record.m_collisionType)
	{
		case SERIALIZE_ID_SPHERE:
		{
			info = new dCollisionSphereNodeInfo(collision);
			break;
		}
		case SERIALIZE_ID_BOX:
		{
			info = new dCollisionBoxNodeInfo(collision);
			break;
		}

		case SERIALIZE_ID_CONE:
		{
			info = new dCollisionConeNodeInfo(collision);
			break;
		}

		case SERIALIZE_ID_CAPSULE:
		{
			info = new dCollisionCapsuleNodeInfo(collision);
			break;
		}
		case SERIALIZE_ID_CYLINDER:
		{
			info = new dCollisionCylinderNodeInfo(collision);
			break;
		}
		case SERIALIZE_ID_CHAMFERCYLINDER:
		{
			info = new dCollisionChamferCylinderNodeInfo(collision);
			break;
		}

		case SERIALIZE_ID_CONVEXHULL:
		{
			info = new dCollisionConvexHullNodeInfo(collision);
			break;
		}

		case SERIALIZE_ID_COMPOUND:
		{
			info = new dCollisionCompoundNodeInfo(collision);
			break;
		}


		case SERIALIZE_ID_CONVEXMODIFIER:
		case SERIALIZE_ID_TREE:
		case SERIALIZE_ID_NULL:
		case SERIALIZE_ID_HEIGHTFIELD:
		case SERIALIZE_ID_USERMESH:
		case SERIALIZE_ID_SCENE:
		case SERIALIZE_ID_COMPOUND_BREAKABLE:
		{
			_ASSERTE(0);
			break;
		}
	}

	dTreeNode* const node = AddNode(info, parent);
	info->Release();
	return node;
}


dScene::dTreeNode* dScene::AddNode(dNodeInfo* const sceneInfo, dTreeNode* const parent)
{
	dTreeNode* const node = dSceneGraph::AddNode (sceneInfo, parent);
	return node;
}


dScene::dTreeNode* dScene::CreateSceneNode(dTreeNode* const parent)
{
	return CreateNode ("dSceneNodeInfo", parent);
}

dScene::dTreeNode* dScene::CreateBoneNode(dTreeNode* const parent)
{
	return CreateNode ("dBoneNodeInfo", parent);
}

dScene::dTreeNode* dScene::CreateRigidbodyNode(dTreeNode* const parent)
{
	return CreateNode ("dRigidbodyNodeInfo", parent);
}

dScene::dTreeNode* dScene::CreateCollisionBoxNode(dTreeNode* const parent)
{
	return CreateNode ("dCollisionBoxNodeInfo", parent);
}

dScene::dTreeNode* dScene::CreateCollisionConeNode(dTreeNode* const parent)
{
	return CreateNode ("dCollisionConeNodeInfo", parent);
}

dScene::dTreeNode* dScene::CreateCollisionSphereNode(dTreeNode* const parent)
{
	return CreateNode ("dCollisionSphereNodeInfo", parent);
}

dScene::dTreeNode* dScene::CreateCollisionCapsuleNode(dTreeNode* const parent)
{
	return CreateNode ("dCollisionCapsuleNodeInfo", parent);
}

dScene::dTreeNode* dScene::CreateCollisionCylinderNode(dTreeNode* const parent)
{
	return CreateNode ("dCollisionCylinderNodeInfo", parent);
}

dScene::dTreeNode* dScene::CreateCollisionCompoundNode(dTreeNode* const parent)
{
	return CreateNode ("dCollisionCompoundNodeInfo", parent);
}

dScene::dTreeNode* dScene::CreateCollisionChamferCylinderNode(dTreeNode* const parent)
{
	return CreateNode ("dCollisionChamferCylinderNodeInfo", parent);
}

dScene::dTreeNode* dScene::CreateCollisionConvexHullNode(dTreeNode* const parent)
{
	return CreateNode ("dCollisionConvexHullNodeInfo", parent);
}

dScene::dTreeNode* dScene::CreateMeshNode(dTreeNode* const parent)
{
	return CreateNode ("dMeshNodeInfo", parent);
}

dScene::dTreeNode* dScene::CreateSkinModifierNode(dTreeNode* const parent)
{
	return CreateNode ("dGeometryNodeSkinModifierInfo", parent);
}


dScene::dTreeNode* dScene::CreateTextureNode (const char* const pathName)
{
	dTreeNode* root = GetRootNode();

	// see if this texture is already exist
	unsigned crc = dCRC (dGetNameFromPath(pathName));
	for (void* ptr = GetFirstChild(root); ptr; ptr = GetNextChild(root, ptr)) {
		dNodeInfo* const info = GetInfoFromNode(GetNodeFromLink (ptr));
		if (info->IsType(dTextureNodeInfo::GetRttiType())) {
			dTextureNodeInfo* texture = (dTextureNodeInfo*) info;
			if (crc == unsigned (texture->GetId())) {
				// we found a texture, return the node
				return GetNodeFromLink (ptr);
			}
		}
	}

	dTreeNode* const node = CreateNode ("dTextureNodeInfo", root);
	((dTextureNodeInfo*) GetInfoFromNode(node))->SetPathName (pathName);

	return node;
}

dScene::dTreeNode* dScene::CreateMaterialNode (dTreeNode* const parent, int id)
{
	dScene::dTreeNode* const node = CreateNode ("dMaterialNodeInfo", parent);
	((dMaterialNodeInfo*) GetInfoFromNode(node))->m_id = id;
	return node;
}





dScene::dTreeNode* dScene::GetFirstNode () const
{
	Iterator iter (*this);
	iter.Begin();
	return iter.GetNode();
}

dScene::dTreeNode* dScene::FindNode (dNodeInfo* const info) const
{
	return Find (info->GetUniqueID());
}

dScene::dTreeNode* dScene::GetNextNode (dTreeNode* const node) const
{
	Iterator iter (*this);
	iter.Set (node);
	iter++;
	return iter.GetNode();
}


void* dScene::GetFirstChild(dTreeNode* const parentNode) const
{
	dGraphNode& root = parentNode->GetInfo();
	return root.m_children.GetFirst();
}

void* dScene::GetNextChild(dTreeNode* const parentNode, void* const link) const
{
	dGraphNode::dLink::dListNode* const node = (dGraphNode::dLink::dListNode*) link;
	return node->GetNext();
}

void* dScene::GetFirstParent(dTreeNode* const childNode) const
{
	dGraphNode& root = childNode->GetInfo();
	return root.m_parents.GetFirst();
}

void* dScene::GetNextParent(dTreeNode* const childNode, void* const link) const
{
	dGraphNode::dLink::dListNode* const node = (dGraphNode::dLink::dListNode*) link;
	return node->GetNext();
}


dScene::dTreeNode* dScene::GetNodeFromLink (void* const child) const
{
	dGraphNode::dLink::dListNode* const node = (dGraphNode::dLink::dListNode*) child;
	return node->GetInfo();
}

dNodeInfo* dScene::GetInfoFromNode(dTreeNode* const node) const
{	
	return node->GetInfo().GetNode();
}

void dScene::CloneInfoFromNode(dTreeNode* const node)
{
	dNodeInfo* copy = node->GetInfo().GetNode()->MakeCopy();
//	dNodeInfo* copy = GetInfoFromNode (node)->MakeCopy();
	_ASSERTE (copy->GetUniqueID() == GetInfoFromNode (node)->GetUniqueID());
	_ASSERTE (copy->GetTypeId() == GetInfoFromNode(node)->GetTypeId()) ;
	node->GetInfo().SetNode(copy);
	copy->Release();
	
//	ReplaceKey (node, copy);

}

dScene::dTreeNode* dScene::FindCameraNode(int camIndex) const
{
	int index = 0;
	dTreeNode* camera = NULL;

	dTreeNode* const rootNode = GetRootNode();
	for (void* child = GetFirstChild (rootNode); child; child = GetNextChild(rootNode, child)) {
		dTreeNode* camNode = GetNodeFromLink (child);

		dNodeInfo* const info = GetInfoFromNode(camNode);
		if (info->IsType(dCameraNodeInfo::GetRttiType())) {
			if (index < camIndex) {
				index ++;
			} else {
				camera = camNode;
				break;
			}
		}
	}
	_ASSERTE (camera);
	_ASSERTE (GetInfoFromNode(camera)->IsType(dCameraNodeInfo::GetRttiType()));
	return camera;
}

dScene::dTreeNode* dScene::FindTextureByTextId(dTreeNode* const parentNode, int textId) const
{
	for (void* ptr = GetFirstChild(parentNode); ptr; ptr = GetNextChild(parentNode, ptr)) {
		dScene::dTreeNode* const node = GetNodeFromLink(ptr);
		dNodeInfo* const info = GetInfoFromNode(node);
		if (info->IsType(dTextureNodeInfo::GetRttiType())) {
			if (((dTextureNodeInfo*) info)->GetId() == textId) {
				return node;
			}
		}
	}
	return NULL;
}


dScene::dTreeNode* dScene::FindChildByType(dTreeNode* const parentNode, int type) const
{
	for (void* child = GetFirstChild (parentNode); child; child = GetNextChild(parentNode, child)) {
		dTreeNode* const tmpNode = GetNodeFromLink (child);
		dNodeInfo* const info = GetInfoFromNode(tmpNode);
		if (info->IsType(type)) {
			return tmpNode;
		}
	}
	return NULL;
}

dScene::dTreeNode* dScene::FindParentByType(dTreeNode* const childNode, int type) const
{
	for (void* parent = GetFirstParent(childNode); parent; parent = GetNextChild(childNode, parent)) {
		dTreeNode* const tmpNode = GetNodeFromLink (parent);
		dNodeInfo* const info = GetInfoFromNode(tmpNode);
		if (info->IsType(type)) {
			return tmpNode;
		}
	}
	return NULL;
}

/*
void dScene::BakeSceneNodeTransform (dTreeNode* sceneNode, dMatrix& matrix) const
{
	_ASSERTE (GetInfoFromNode(sceneNode)->IsType(dSceneNodeInfo::GetRttiType()));

	dSceneNodeInfo* sceneInfo = (dSceneNodeInfo*) GetInfoFromNode(sceneNode);
	sceneInfo->BakeTransform (matrix);

	for (void* ptr = GetFirstChild(sceneNode); ptr; ptr = GetNextChild(sceneNode, ptr)) {
		dTreeNode* geomNode = GetNodeFromLink(ptr);
		if (GetInfoFromNode(geomNode)->IsType(dGeometryNodeInfo::GetRttiType())) {
			dGeometryNodeInfo* geom = (dGeometryNodeInfo*)GetInfoFromNode(geomNode);
			geom->BakeTransform(matrix);
		}
	}
}


void dScene::BakeSceneNodeTransformRecursive (dTreeNode* sceneNode, dMatrix& matrix) const
{
	BakeSceneNodeTransform (sceneNode, matrix);
	for (void* ptr = GetFirstChild(sceneNode); ptr; ptr = GetNextChild(sceneNode, ptr)) {
		sceneNode = GetNodeFromLink(ptr);
		if (GetInfoFromNode(sceneNode)->IsType(dSceneNodeInfo::GetRttiType())) {
			BakeSceneNodeTransformRecursive (sceneNode, matrix);
		}
	}
}
*/

void dScene::BakeTransform (dMatrix& matrix) const
{
	Iterator iter (*this);

	for (iter.Begin(); iter; iter ++) {
		dTreeNode* const node = iter.GetNode();
		dNodeInfo* const nodeInfo = GetInfoFromNode(node);
		nodeInfo->BakeTransform (matrix);
	}
//	for (void* ptr = GetFirstChild(root); ptr; ptr = GetNextChild(root, ptr)) {
//		dTreeNode* sceneNode = GetNodeFromLink(ptr);
//		if (GetInfoFromNode(sceneNode)->IsType(dSceneNodeInfo::GetRttiType())) {
//			BakeSceneNodeTransformRecursive (sceneNode, matrix);
//		}
//	}
	

}


void dScene::Serialize (const char* const fileName)
{
	TiXmlDocument asciifile;
	TiXmlDeclaration* decl = new TiXmlDeclaration( "1.0", "", "" );
	asciifile.LinkEndChild (decl);

	TiXmlElement* parentNode = new TiXmlElement ("alchemedia");
	asciifile.LinkEndChild(parentNode);

	// save the file description and version
	TiXmlElement* header = new TiXmlElement ("header");
	parentNode->LinkEndChild(header);

	// save configuration for the main frame window
	header->SetAttribute ("description", "alchemedia file format");
	header->SetAttribute ("revision", m_currentRevision);


	// need to remove unused vertices before saving, otherwise Deserialize will not work,
	RemoveUnusedVertex();

	// save file content
	dSceneGraph::Serialize (parentNode);

	// save the file
	asciifile.SaveFile (fileName);
}

bool dScene::Deserialize (const char* const fileName)
{
	// apply last Configuration
	TiXmlDocument doc (fileName);
	doc.LoadFile();

	bool state = true;
	const TiXmlElement* root = doc.RootElement();
	if (root && doc.FirstChild ("alchemedia") ){

		TiXmlElement* root = (TiXmlElement*) doc.FirstChild ("alchemedia");
		_ASSERTE (root);

		TiXmlElement* header = (TiXmlElement*) root->FirstChild ("header");
		_ASSERTE (header);

		int revision;
		header->Attribute("revision", &revision);
		
		TiXmlElement* nodes = (TiXmlElement*) root->FirstChild ("nodes");
		_ASSERTE (nodes);

		state =  dSceneGraph::Deserialize (nodes, revision);
	}
	return state;
}

void dScene::SerializeBinary(const char* const fileName)
{
	_ASSERTE (0);

/*
	FILE* file = fopen (fileName, "wb");
	if (file) {
		fprintf (file, "alchemedia\n");
		fprintf (file, "header\n");
		fprintf (file, "description: alchemedia file format\n");
		fprintf (file, "revision\n");
		int revision = m_currentRevision;
		fwrite (&revision, 1, sizeof (int), file);

		dSceneGraph::SerializeBinary (file);

		fclose (file);
	}
*/
}


void dScene::MergeScene (dScene* const scene)
{
	dTree<dTreeNode*,dTreeNode*> map;
	Iterator iter (*scene);
	map.Insert(GetRootNode(), scene->GetRootNode());
	for (iter.Begin(); iter; iter ++) {
		dTreeNode* const node = iter.GetNode();
		dNodeInfo* const info = scene->GetInfoFromNode(node);
		if (!(info->IsType(dCameraNodeInfo::GetRttiType()) || info->IsType(dRootNodeInfo::GetRttiType()))) {
			_ASSERTE (!Find (info->GetUniqueID()));
			dTreeNode* newNode = AddNode (info, NULL);
			map.Insert(newNode, node);
		}
	}

	//now connect all edges
	dTree<dTreeNode*,dTreeNode*>::Iterator mapIter (map);
	for (mapIter.Begin(); mapIter; mapIter ++) {
		dTreeNode* srcNode = mapIter.GetKey();
		dGraphNode& srcInfoHeader = mapIter.GetNode()->GetInfo()->GetInfo();
		for (void* ptr = scene->GetFirstChild (srcNode); ptr; ptr = scene->GetNextChild(srcNode, ptr)) {
			dTreeNode* srcLinkNode = scene->GetNodeFromLink(ptr);

			dTree<dTreeNode*,dTreeNode*>::dTreeNode* mapSaved = map.Find(srcLinkNode);
			if (mapSaved) {
				dTreeNode* const node = mapSaved->GetInfo();
				srcInfoHeader.m_children.Append(node);
			}
		}

		for (void* ptr = scene->GetFirstParent (srcNode); ptr; ptr = scene->GetNextParent(srcNode, ptr)) {
			dTreeNode* srcLinkNode = scene->GetNodeFromLink(ptr);

			dTree<dTreeNode*,dTreeNode*>::dTreeNode* mapSaved = map.Find(srcLinkNode);
			if (mapSaved) {
				dTreeNode* const node = mapSaved->GetInfo();
				srcInfoHeader.m_parents.Append(node);
			}
		}

	}
}

void dScene::DeleteNode (dTreeNode* const node)
{
	dSceneGraph::DeleteNode (node);
}

void dScene::MergeDuplicateTextures()
{
return;
	dTreeNode* const root = GetRootNode();

	for (void* ptr0 = GetFirstChild (root); ptr0; ptr0 = GetNextChild(root, ptr0)) {
		dTreeNode* texNode0 = GetNodeFromLink(ptr0);
		dNodeInfo* const info = GetInfoFromNode(texNode0);
		if (info->IsType(dTextureNodeInfo::GetRttiType())) {
			dTextureNodeInfo* const tex0 = (dTextureNodeInfo*) info;
			const char* const texName0 = tex0->GetPathName();

			void* next = NULL;
			for (void* ptr1 = GetNextChild(root, ptr0); ptr1; ptr1 = next) {
				next = GetNextChild(root, ptr1);
				dTreeNode* texNode1 = GetNodeFromLink(ptr1);
				dNodeInfo* const info1 = GetInfoFromNode(texNode1);
				if (info1->IsType(dTextureNodeInfo::GetRttiType())) {
					dTextureNodeInfo* const tex1 = (dTextureNodeInfo*) info1;
					const char* const texName1 = tex1->GetPathName();
					if (!stricmp(texName0, texName1)) {
						
						for (void* ptr3 = GetFirstChild(texNode1); ptr3; ptr3 = GetNextChild(texNode1, ptr3)) {
							dTreeNode* const matNode = GetNodeFromLink(ptr3);
							dNodeInfo* const info3 = GetInfoFromNode(matNode);
							if (info3->IsType(dMaterialNodeInfo::GetRttiType())) {
								AddReference(matNode, texNode0);
							}
						}
						DeleteNode(texNode1);
					}
				}
			}
		}
	}
}

void dScene::RemoveUnusedVertex()
{
	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		dTreeNode* const node = iter.GetNode();
		dNodeInfo* const info = node->GetInfo().GetNode();
		if (info->IsType(dMeshNodeInfo::GetRttiType())) {
			dMeshNodeInfo* const mesh = (dMeshNodeInfo*) info;
			mesh->RemoveUnusedVertices(this, node);
		}
	}
}


void dScene::SetNodeLRU (dTreeNode* const node, int lru) 
{
	node->GetInfo().SetLRU(lru);
}

int dScene::GetNodeLRU (dTreeNode* const node) const 
{ 
	return node->GetInfo().GetLRU();
}


void dScene::UpdateAllOOBB ()
{
	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		dTreeNode* const node = iter.GetNode();
		dNodeInfo* const info = GetInfoFromNode(node);
		if (info->IsType(dSceneNodeInfo::GetRttiType())){
			dSceneNodeInfo* sceneInfo = (dSceneNodeInfo*) GetInfoFromNode(node);
			dScene::dTreeNode* geomNode = FindChildByType(node, dGeometryNodeInfo::GetRttiType());
			dGeometryNodeInfo* geom = NULL;
			if (geomNode) {
				geom = (dGeometryNodeInfo*) GetInfoFromNode(geomNode);
			}
			sceneInfo->UpdateOOBB(geom);
		}
	}
}

dScene::dTreeNode* dScene::RayCast (const dVector& p0, const dVector& p1) const
{
	dFloat t = 1.2f;
	dTreeNode* sceneNode = NULL;
	dVector p2 (p1) ;
	Iterator iter (*this);
	for (iter.Begin(); iter; iter ++) {
		dTreeNode* const node = iter.GetNode();
		dNodeInfo* const info = GetInfoFromNode(node);
		if (info->IsType(dSceneNodeInfo::GetRttiType())){
			dSceneNodeInfo* sceneInfo = (dSceneNodeInfo*) GetInfoFromNode(node);
			dFloat t1 = sceneInfo->RayCast(p0, p2);
			if (t1 < t) {
				dScene::dTreeNode* geomNode = FindChildByType(node, dGeometryNodeInfo::GetRttiType());
				if (geomNode) {
					dMatrix matrix (sceneInfo->GetTransform().Inverse4x4());
					dVector q0 (matrix.RotateVector4x4(p0));
					dVector q1 (matrix.RotateVector4x4(p2));
					dGeometryNodeInfo* geom = (dGeometryNodeInfo*) GetInfoFromNode(geomNode);
					t1 = geom->RayCast(q0, q1);
				}
				if (t1 < t) {
					t = t1;
					p2 = p0 + (p2 - p0).Scale (t);
					sceneNode = node;
				}
			}
		}
	}
	
	return sceneNode;
}


void dScene::SceneToNewtonWorld (NewtonWorld* world, dList<NewtonBody*>& loadedBodies)
{
	// Load the Physics scene
	for (dTreeNode* node = GetFirstNode (); node; node = GetNextNode (node)) {
		dNodeInfo* const info = GetInfoFromNode(node);

		if (info->GetTypeId() == dRigidbodyNodeInfo::GetRttiType()) {
			dRigidbodyNodeInfo* const bodyData = (dRigidbodyNodeInfo*) info;
			NewtonBody* const rigidBody = bodyData->CreateNewtonBody(world, this, node);
			loadedBodies.Append(rigidBody);
		}
	}
}


struct dSceneNodeCollisionPair
{
	dScene::dTreeNode* m_mesh;
	dScene::dTreeNode* m_collision;
};

void dScene::NewtonWorldToScene (const NewtonWorld* const world)
{
	// search for all collision mesh and create make a dictionary
	dTree<dSceneNodeCollisionPair, NewtonCollision*> dictionary;
	

	dScene::dTreeNode* materialNode = CreateMaterialNode (GetRootNode(), 0);
	for (NewtonBody* body = NewtonWorldGetFirstBody(world); body; body = NewtonWorldGetNextBody(world, body)) {
		NewtonCollision* collision = NewtonBodyGetCollision(body);
		dTree<dSceneNodeCollisionPair, NewtonCollision*>::dTreeNode* node = dictionary.Find(collision);
		if (!node) {
			NewtonMesh* const mesh = NewtonMeshCreateFromCollision(collision);
			dScene::dTreeNode* const meshNode = CreateMeshNode(GetRootNode());
			AddReference(meshNode, materialNode);

			dMeshNodeInfo* const info = (dMeshNodeInfo*)GetInfoFromNode(meshNode);
			info->ReplaceMesh (mesh);

			NewtonCollisionInfoRecord collsionRecord;
			NewtonCollisionGetInfo(collision, &collsionRecord);

			// extract the offset matrix form the collision
			dMatrix& offsetMatrix = *((dMatrix*)&collsionRecord.m_offsetMatrix[0][0]);
			info->BakeTransform (offsetMatrix.Inverse());
			info->SetPivotMatrix(offsetMatrix * info->GetPivotMatrix());

			dScene::dTreeNode* collisionNode = CreateCollisionFromNewtonCollision(GetRootNode(), collision);

			dSceneNodeCollisionPair pair;
			pair.m_mesh = meshNode;
			pair.m_collision = collisionNode;

			node = dictionary.Insert(pair, collision);
		} 
		
		// add a visual mesh
		dSceneNodeCollisionPair& info = node->GetInfo();
		dScene::dTreeNode* const sceneNode = CreateSceneNode(GetRootNode());
		dSceneNodeInfo* const sceneInfo = (dSceneNodeInfo*) GetInfoFromNode(sceneNode);
		dMatrix matrix;
		NewtonBodyGetMatrix(body, &matrix[0][0]);
		sceneInfo->SetTransform(matrix);
		AddReference(sceneNode, info.m_mesh);


		// add a rigid body
		dScene::dTreeNode* const sceneBody = CreateRigidbodyNode(sceneNode);
		AddReference(sceneBody, info.m_collision);

		dRigidbodyNodeInfo* const bodyInfo = (dRigidbodyNodeInfo*) GetInfoFromNode(sceneBody);

		dVector com;
		NewtonBodyGetCentreOfMass(body, &com[0]);
		bodyInfo->SetCenterOfMass(com);

		dVector massMatrix;
		NewtonBodyGetMassMatrix(body, &massMatrix.m_w, &massMatrix.m_x, &massMatrix.m_y, &massMatrix.m_z);
		bodyInfo->SetMassMatrix(massMatrix);

		dVector veloc;
		NewtonBodyGetVelocity(body, &veloc[0]);
		bodyInfo->SetVelocity(veloc);

		dVector omega;
		NewtonBodyGetOmega(body, &omega[0]);
		bodyInfo->SetOmega(omega);

		dVariable* var = bodyInfo->CreateVariable ("rigidBodyType");
		var->SetValue("default gravity");
	}

	void* nextPtr = NULL;
	for (void* ptr = GetFirstChild (GetRootNode()); ptr; ptr = nextPtr) {
		nextPtr = GetNextChild(GetRootNode(), ptr);
		dScene::dTreeNode* const node = GetNodeFromLink(ptr);
		dNodeInfo* const info = GetInfoFromNode(node);
		if ((info->IsType(dMeshNodeInfo::GetRttiType())) || (info->IsType(dCollisionNodeInfo::GetRttiType()))) {
			RemoveReference(node, GetRootNode());	
		}
	}
	RemoveReference(materialNode, GetRootNode());	
}