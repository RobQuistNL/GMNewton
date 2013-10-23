/////////////////////////////////////////////////////////////////////////////
// Name:        dCameraNodeInfo.h
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
#include "dSceneNodeInfo.h"
#include "dCameraNodeInfo.h"
#include "dGeometryNodeInfo.h"


D_IMPLEMENT_CLASS_NODE(dCameraNodeInfo);


dCameraNodeInfo::dCameraNodeInfo()
	:dNodeInfo(), m_origin (0.0f, 0.0f, 0.0f, 0.0f), m_roll (30.0f * 3.141592f / 180.0f), m_yaw (30.0f * 3.141592f / 180.0f), m_viewportIndex(0)
{
}

dCameraNodeInfo::dCameraNodeInfo(dScene* const world)
	:dNodeInfo(), m_origin (0.0f, 0.0f, 0.0f, 0.0f), m_roll (30.0f * 3.141592f / 180.0f), m_yaw (30.0f * 3.141592f / 180.0f), m_viewportIndex(0)
{
}

dCameraNodeInfo::dCameraNodeInfo(const dCameraNodeInfo& me)
	:dNodeInfo(me), m_origin(me.m_origin), m_roll (me.m_roll), m_yaw (me.m_yaw), m_viewportIndex(0)
{
}


dCameraNodeInfo::~dCameraNodeInfo(void)
{
}


void dCameraNodeInfo::SetCameraOrigin (const dVector& location)
{
	m_origin = location;
}

void dCameraNodeInfo::SetCameraYaw(dFloat yaw)
{
	m_yaw = yaw;
}

void dCameraNodeInfo::SetCameraRoll(dFloat roll)
{
	m_roll = roll;
}

const dVector& dCameraNodeInfo::GetCameraOrigin () const
{
	return m_origin;
}

dFloat dCameraNodeInfo::GetCameraYaw() const
{
	return m_yaw;
}

dFloat dCameraNodeInfo::GetCameraRoll() const
{
	return m_roll;
}


void dCameraNodeInfo::Serialize (TiXmlElement* const rootNode) const
{
	SerialiseBase(dNodeInfo, rootNode);

	TiXmlElement* position = new TiXmlElement ("position");
	rootNode->LinkEndChild(position);
	position->SetDoubleAttribute("x", double (m_origin.m_x));
	position->SetDoubleAttribute("y", double (m_origin.m_y));
	position->SetDoubleAttribute("z", double (m_origin.m_z));

	TiXmlElement* rotation = new TiXmlElement ("rotation");
	rootNode->LinkEndChild(rotation);
	rotation->SetDoubleAttribute("pitch", double (m_roll * 180.0f / 3.141592f));
	rotation->SetDoubleAttribute("yaw", double (m_yaw * 180.0f / 3.141592f));

/*
	TiXmlElement* miscelaneus = new TiXmlElement ("miscelaneus");
	rootNode->LinkEndChild(miscelaneus);
	miscelaneus->SetDoubleAttribute("dist", double (m_distance));
	miscelaneus->SetDoubleAttribute("viewportOrthoSize", double (m_2dViewportSize));
	
	switch (m_projectionMode) {
		case m_perspective:
			miscelaneus->SetAttribute("projectionMode", "perspective");
			break;

		case m_orthographic:
			miscelaneus->SetAttribute("projectionMode", "orthographic");
			break;

		case m_front:
			miscelaneus->SetAttribute("projectionMode", "front");
			break;

		case m_back:
			miscelaneus->SetAttribute("projectionMode", "back");
			break;

		case m_left:
			miscelaneus->SetAttribute("projectionMode", "left");
			break;

		case m_right:
			miscelaneus->SetAttribute("projectionMode", "right");
			break;

		case m_top:
			miscelaneus->SetAttribute("projectionMode", "top");
			break;

		case m_bottom:
			miscelaneus->SetAttribute("projectionMode", "bottom");
			break;
	}
*/
}

bool dCameraNodeInfo::Deserialize (TiXmlElement* const rootNode, int revisionNumber) 
{
	DeserialiseBase(dNodeInfo, rootNode, revisionNumber);
	double x;
	double y;
	double z;
	TiXmlElement* position = (TiXmlElement*) rootNode->FirstChild ("position");
	position->Attribute("x", &x);
	position->Attribute("y", &y);
	position->Attribute("z", &z);
	m_origin = dVector (dFloat (x), dFloat (y), dFloat (z), 0.0f);

	TiXmlElement* rotation = (TiXmlElement*) rootNode->FirstChild ("rotation");
	rotation->Attribute("pitch", &x);
	rotation->Attribute("yaw", &y);

	m_roll = dFloat (x) * 3.141592f / 180.0f; 
	m_yaw = dFloat (y) * 3.141592f / 180.0f; 

/*
	TiXmlElement* miscelaneus = (TiXmlElement*) rootNode->FirstChild ("miscelaneus");
	miscelaneus->Attribute("dist", &x);
	miscelaneus->Attribute("viewportOrthoSize", &y);
	const char* projection = miscelaneus->Attribute("projectionMode");

	m_distance = dFloat (x);
	m_2dViewportSize = dFloat (y);

	if (!strcmp (projection, "perspective")) {
		m_projectionMode = m_perspective; 
	} else if (!strcmp (projection, "orthographic")) { 
		m_projectionMode = m_orthographic; 
	} else if (!strcmp (projection, "front")) { 
		m_projectionMode = m_front; 
	} else if (!strcmp (projection, "back")) { 
		m_projectionMode = m_back; 
	} else if (!strcmp (projection, "left")) { 
		m_projectionMode = m_left; 
	} else if (!strcmp (projection, "right")) { 
		m_projectionMode = m_right; 
	} else if (!strcmp (projection, "top")) { 
		m_projectionMode = m_top; 
	} else if (!strcmp (projection, "bottom")) { 
		m_projectionMode = m_bottom; 
	}
*/
	return true;
}

void dCameraNodeInfo::SerializeBinary (FILE* const file) 
{
	_ASSERTE (0);
/*
	fprintf (file, "%s\n%s\n", GetClassName(), GetName());

	fwrite (&m_pointOfInterest.m_x, 3, sizeof (dFloat), file);
	fwrite (&m_roll, 1, sizeof (dFloat), file);
	fwrite (&m_yaw, 1, sizeof (dFloat), file);
	fwrite (&m_distance, 1, sizeof (dFloat), file);
	fwrite (&m_2dViewportSize, 1, sizeof (dFloat), file);

	int projection = m_projectionMode;
	fwrite (&projection, 1, sizeof (int), file);
*/
}








