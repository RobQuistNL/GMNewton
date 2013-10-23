/////////////////////////////////////////////////////////////////////////////
// Name:        dRigidbodyNodeInfo.h
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

#ifndef _D_RIGIDBODY_NODE_H_
#define _D_RIGIDBODY_NODE_H_

#include "dNodeInfo.h"


#define D_RIGIDBODY_BINDING_FUNTION_NAME_SIZE 64

class dRigidbodyNodeInfo: public dNodeInfo
{
	public:
	D_DEFINE_CLASS_NODE(dRigidbodyNodeInfo,dNodeInfo)

	dRigidbodyNodeInfo();
	dRigidbodyNodeInfo(dScene* const world);
	virtual ~dRigidbodyNodeInfo(void);

	virtual void BakeTransform (const dMatrix& transform);

	virtual void SetMassMatrix (const dVector& massMatrix);
	virtual void SetCenterOfMass (const dVector& centerOfMass);
	virtual void SetOmega (const dVector& omega);
	virtual void SetVelocity (const dVector& velocity);

	virtual const dVector& GetMassMatrix () const;
	virtual const dVector& GetCenterOfMass () const;
	virtual const dVector& GetOmega () const;
	virtual const dVector& GetVelocity () const;

	


//	virtual void UpdateOOBB (dGeometryNodeInfo* geomInfo);
//	virtual dFloat RayCast (const dVector& p0, const dVector& p1) const;

	// draw scene in wire frame mode
	virtual void DrawWireFrame(dScene* const world, dScene::dTreeNode* const myNode, const dVector& color) const;

	// draw scene in solid wire frame mode
	virtual void DrawSolidWireFrame(dScene* const world, dScene::dTreeNode* const myNode, const dVector& color) const;

	// draw scene in Gouraud shaded normal textured mode 
	virtual void DrawGouraudShaded(dScene* const world, dScene::dTreeNode* const myNode, const dVector& color) const;

	// Draw selection gizmo
	virtual void DrawGizmo(dScene* const world, dScene::dTreeNode* const myNode, const dMatrix& coordinaSystem, const dVector& color, dGizmoMode mode, dFloat size) const;
	virtual dGizmoHandle GetHighlightedGizmoHandle(dScene* const world, dScene::dTreeNode* const myNode, const dMatrix& coordinaSystem, const dVector& screenPosition, dGizmoMode mode, dFloat size) const;
	virtual void DrawGizmoHandle(dScene* world, const dMatrix& coordinaSystem, dGizmoMode mode, dGizmoHandle handle, const dVector& color, dFloat size) const;

	virtual NewtonBody* CreateNewtonBody (NewtonWorld* const world, dScene* const scene, dScene::dTreeNode* const myNode) const;
	protected:
	virtual void SerializeBinary (FILE* const file);
	virtual void Serialize (TiXmlElement* const rootNode) const; 
	virtual bool Deserialize (TiXmlElement* const rootNode, int revisionNumber);
	
	dVector m_centerOfMass;
	dVector m_massMatrix;
	dVector m_velocity;
	dVector m_omega;
	dVector m_internalDamp;
};





#endif