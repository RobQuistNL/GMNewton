/////////////////////////////////////////////////////////////////////////////
// Name:        dTextureNodeInfo.h
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
#include "dTextureNodeInfo.h"


D_IMPLEMENT_CLASS_NODE(dTextureNodeInfo);

dTextureNodeInfo::dTextureNodeInfo()
	:dNodeInfo (), m_id (0), m_internalUsage(-1) 
{

}

dTextureNodeInfo::dTextureNodeInfo(dScene* const world)
	:dNodeInfo (), m_id (0), m_internalUsage(-1) 
{
	SetName ("texture");
}

dTextureNodeInfo::dTextureNodeInfo(const char* const pathName)
	:dNodeInfo (), m_internalUsage(-1) 
{
	SetName ("texture");
	SetPathName (pathName);
}

dTextureNodeInfo::~dTextureNodeInfo(void)
{
	if (m_internalUsage != -1)  {
		_ASSERTE (0);
	}
}

void dTextureNodeInfo::SetPathName (const char* const path)
{
	const char* ptr = dGetNameFromPath (path);
	strncpy (m_path, ptr, sizeof (m_path));
	m_id = dCRC (m_path);
}


void dTextureNodeInfo::Serialize (TiXmlElement* const rootNode) const
{
	SerialiseBase(dNodeInfo, rootNode);

	rootNode->SetAttribute("path", m_path);
}

bool dTextureNodeInfo::Deserialize (TiXmlElement* const rootNode, int revisionNumber) 
{
	DeserialiseBase(dNodeInfo, rootNode, revisionNumber);

	SetPathName (rootNode->Attribute ("path"));

	return true;
}

void dTextureNodeInfo::SerializeBinary (FILE* const file) 
{
	fprintf (file, "%s\n%s\n", GetClassName(), GetName());

	fwrite (&m_id, 1, sizeof (int), file);
	fprintf (file, "%s\n", m_path);
}
