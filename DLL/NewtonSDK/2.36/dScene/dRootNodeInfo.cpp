/////////////////////////////////////////////////////////////////////////////
// Name:        dRootNodeInfo.h
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
#include "dRootNodeInfo.h"


D_IMPLEMENT_CLASS_NODE(dRootNodeInfo);

dRootNodeInfo::dRootNodeInfo(dScene* const world) 
{
	SetName ("rootNode");
}

dRootNodeInfo::dRootNodeInfo()
{
	SetName ("rootNode");
}

dRootNodeInfo::~dRootNodeInfo(void)
{
}



void dRootNodeInfo::Serialize (TiXmlElement* const rootNode) const
{
	SerialiseBase(dNodeInfo, rootNode);
}

bool dRootNodeInfo::Deserialize (TiXmlElement* const rootNode, int revisionNumber) 
{
	DeserialiseBase(dNodeInfo, rootNode, revisionNumber);
	return true;
}

void dRootNodeInfo::SerializeBinary (FILE* const file) 
{
	fprintf (file, "%s\n%s\n", GetClassName(), GetName());
}