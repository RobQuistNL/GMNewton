/////////////////////////////////////////////////////////////////////////////
// Name:        dGeometryNodeModifierInfo.h
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

#ifndef _D_GEOMETRY_NODE_MODIFIER_INFO_H_
#define _D_GEOMETRY_NODE_MODIFIER_INFO_H_

#include "dNodeInfo.h"

class dGeometryNodeModifierInfo: public dNodeInfo
{
	public:
	D_DEFINE_CLASS_NODE(dGeometryNodeModifierInfo,dNodeInfo)

	dGeometryNodeModifierInfo();
	dGeometryNodeModifierInfo(dScene* const world);
	dGeometryNodeModifierInfo(const dGeometryNodeModifierInfo& me);
	virtual ~dGeometryNodeModifierInfo(void);

	virtual void RemoveUnusedVertices(const int* const verteMap);

//	virtual const dMatrix& GetPivotMatrix () const;
//	virtual void SetPivotMatrix (const dMatrix& matrix);
//	virtual void BakeTransform (const dMatrix& matrix);
//	virtual void CalculateOOBBGizmo (const dMatrix& matrix, dVector& p0, dVector& p1) const {};
//	virtual dFloat RayCast (const dVector& p0, const dVector& p1) const {return 1.0f;}
	virtual void SerializeBinary (FILE* const file) {};
	virtual void Serialize (TiXmlElement* const rootNode) const;
	virtual bool Deserialize (TiXmlElement* const rootNode, int revisionNumber);

//	dMatrix m_matrix;
};





#endif