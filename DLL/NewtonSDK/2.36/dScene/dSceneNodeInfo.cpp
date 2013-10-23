/////////////////////////////////////////////////////////////////////////////
// Name:        dSceneNodeInfo.h
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
#include "dDrawUtils.h"
#include "dSceneNodeInfo.h"
#include "dGeometryNodeInfo.h"




D_IMPLEMENT_CLASS_NODE(dSceneNodeInfo);

dSceneNodeInfo::dSceneNodeInfo(dScene* const world) 
	:dNodeInfo (), 
	m_position (0.0f, 0.0f, 0.0f, 1.0f),  // node location in global space
	m_euler (0.0f, 0.0f, 0.0f, 1.0f),     // node orientation matrix R: x = pitch, y = yaw, z = roll, 
	m_scale (1.0f, 1.0f, 1.0f, 1.0f),	// local scale: x = scale_x, y = scale_y, z = scale_z, 
	m_eigenScaleAxis (GetIdentityMatrix()),
	m_solidColor (0.75f, 0.75f, 0.0f, 0.0f),
	m_editorMinOOBB (0.0f, 0.0f, 0.0f, 0.0f), 
	m_editorMaxOOBB (0.0f, 0.0f, 0.0f, 0.0f)
{
	SetName ("sceneNode");
}

dSceneNodeInfo::dSceneNodeInfo()
	:dNodeInfo (), 
	 m_position (0.0f, 0.0f, 0.0f, 1.0f),  // node location in global space
	 m_euler (0.0f, 0.0f, 0.0f, 1.0f),     // node orientation matrix R: x = pitch, y = yaw, z = roll, 
	 m_scale (1.0f, 1.0f, 1.0f, 1.0f),	// local scale: x = scale_x, y = scale_y, z = scale_z, 
	 m_eigenScaleAxis (GetIdentityMatrix()),
	 m_solidColor (0.75f, 0.75f, 0.0f, 0.0f),
	 m_editorMinOOBB (0.0f, 0.0f, 0.0f, 0.0f), 
	 m_editorMaxOOBB (0.0f, 0.0f, 0.0f, 0.0f)
{
	SetName ("sceneNode");
}

dSceneNodeInfo::~dSceneNodeInfo(void)
{
}



dMatrix dSceneNodeInfo::GetTransform () const
{
//	return dMatrix (CalculateOrthoMatrix()), m_scale, m_eigenScaleAxis);
	return dMatrix (CalculateScaleMatrix() * CalculateOrthoMatrix());
}

void dSceneNodeInfo::SetTransform (const dMatrix& matrix)
{
	dMatrix transform;
	matrix.PolarDecomposition(transform, m_scale, m_eigenScaleAxis);
	m_position = matrix.m_posit;
	m_euler = transform.GetXYZ_EulerAngles ();
}

dMatrix dSceneNodeInfo::CalculateScaleMatrix() const
{
	return dMatrix (GetIdentityMatrix(), m_scale, m_eigenScaleAxis);
}

dMatrix dSceneNodeInfo::CalculateOrthoMatrix() const
{
	dMatrix matrix (dPitchMatrix(m_euler.m_x) * dYawMatrix(m_euler.m_y) * dRollMatrix(m_euler.m_z));
	matrix.m_posit = m_position;
	return matrix;
}




dVector dSceneNodeInfo::GetPosition () const
{
	return m_position;
}

void dSceneNodeInfo::SetPosition (const dVector& position)
{
	m_position = position;
}

dVector dSceneNodeInfo::GetEulers () const
{
	return m_euler;	
}

void dSceneNodeInfo::SetEulers (const dVector& angles)
{
	m_euler = angles;
}

dVector dSceneNodeInfo::GetScale () const
{
	return m_scale;
}

void dSceneNodeInfo::SetScale (const dVector& scale)
{
	m_scale = scale;
}

dVector dSceneNodeInfo::GetColor () const
{
	return m_solidColor;
}

void dSceneNodeInfo::SetColor (const dVector& color)
{
	m_solidColor = color;
}




void dSceneNodeInfo::BakeTransform (const dMatrix& transform)
{
	SetTransform (transform.Inverse4x4() * GetTransform() * transform);
}

void dSceneNodeInfo::UpdateOOBB (dGeometryNodeInfo* geomInfo)
{
	if (geomInfo) {
		dVector euler (GetEulers ());
		dMatrix matrix (euler.m_x, euler.m_y, euler.m_z, GetPosition());
		dMatrix scaleMatrix (GetTransform() * matrix.Inverse());
		
		geomInfo->CalculateOOBBGizmo (scaleMatrix, m_editorMinOOBB, m_editorMaxOOBB);
	} else {
		// null will have a small OOBB for picking
		m_editorMinOOBB = dVector (-0.01f, -0.01f, -0.01f, -0.01f) ;	
		m_editorMaxOOBB = dVector ( 0.0f,   0.0f,   0.0f,   0.0f) ;	
	}
}

dFloat dSceneNodeInfo::RayCast (const dVector& p0, const dVector& p1) const
{
	// for now just calculate the OOBB ray cast
	dVector euler (GetEulers ());
	dMatrix matrix (euler.m_x, euler.m_y, euler.m_z, GetPosition());

	dVector localP0 (matrix.UntransformVector(p0));
	dVector localP1 (matrix.UntransformVector(p1)); 

	if ((localP0.m_x >= m_editorMinOOBB.m_x) &&(localP0.m_y >= m_editorMinOOBB.m_y) && (localP0.m_z >= m_editorMinOOBB.m_z) && 
		(localP1.m_x <= m_editorMaxOOBB.m_x) &&(localP1.m_y <= m_editorMaxOOBB.m_y) && (localP1.m_z <= m_editorMaxOOBB.m_z)) {
			return 0.0f;
	} else {
		return dBoxRayCast (localP0, localP1, m_editorMinOOBB, m_editorMaxOOBB);
	}
}


void dSceneNodeInfo::Serialize (TiXmlElement* const rootNode) const
{
	SerialiseBase(dNodeInfo, rootNode);

	char tmp[1024];

	TiXmlElement* color = new TiXmlElement ("color");
	rootNode->LinkEndChild(color);
	dFloatArrayToString (&m_solidColor[0], 4, tmp, sizeof (tmp));
	color->SetAttribute("float4", tmp);

	TiXmlElement* matrix = new TiXmlElement ("transform");
	rootNode->LinkEndChild(matrix);
	dFloatArrayToString (&m_position[0], 4, tmp, sizeof (tmp));
	matrix->SetAttribute("position", tmp);

	dFloatArrayToString (&m_euler[0], 4, tmp, sizeof (tmp));
	matrix->SetAttribute("eulerAngles", tmp);

	dFloatArrayToString (&m_scale[0], 4, tmp, sizeof (tmp));
	matrix->SetAttribute("localScale", tmp);

	dFloatArrayToString (&m_eigenScaleAxis[0][0], 16, tmp, sizeof (tmp));
	matrix->SetAttribute("stretchAxis", tmp);
}

bool dSceneNodeInfo::Deserialize (TiXmlElement* const rootNode, int revisionNumber) 
{
	DeserialiseBase(dNodeInfo, rootNode, revisionNumber);

	TiXmlElement* tcolor = (TiXmlElement*) rootNode->FirstChild ("color");
	dStringToFloatArray (tcolor->Attribute("float4"), &m_solidColor[0], 4);

	TiXmlElement* transformNode = (TiXmlElement*) rootNode->FirstChild ("transform");
	dStringToFloatArray (transformNode->Attribute("position"), &m_position[0], 4);
	dStringToFloatArray (transformNode->Attribute("eulerAngles"), &m_euler[0], 4);
	dStringToFloatArray (transformNode->Attribute("localScale"), &m_scale[0], 4);
	dStringToFloatArray (transformNode->Attribute("stretchAxis"), &m_eigenScaleAxis[0][0], 16);

	return true;
}


void dSceneNodeInfo::SerializeBinary (FILE* const file) 
{
	fprintf (file, "%s\n%s\n", GetClassName(), GetName());
	_ASSERTE (0);
//	fwrite (&m_matrix[0][0], 1, sizeof (dMatrix), file);
}







