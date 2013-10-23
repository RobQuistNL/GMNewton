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

#ifndef _D_TEXTURE_NODE_H_
#define _D_TEXTURE_NODE_H_

#include "dNodeInfo.h"

#define D_MAX_TEXTURE_PATH_NAME_LENGTH	128

class dTextureNodeInfo: public dNodeInfo
{
	public:
	D_DEFINE_CLASS_NODE(dTextureNodeInfo,dNodeInfo)

	dTextureNodeInfo(); 
	dTextureNodeInfo(dScene* const world); 
	dTextureNodeInfo(const char* const pathName);
	virtual ~dTextureNodeInfo(void);

	virtual int GetId () const {return m_id;}
	virtual const char* GetPathName () const {return m_path;}
	virtual void SetPathName (const char* const path);

	int GetInternalId() const {return m_internalUsage;}
	void SetInternalId(int id) {m_internalUsage = id;}


	protected:
	virtual void SerializeBinary (FILE* const file);
	virtual void Serialize (TiXmlElement* const rootNode) const; 
	virtual bool Deserialize (TiXmlElement* const rootNode, int revisionNumber);

	int m_id;
	int m_internalUsage;
	char m_path[D_MAX_TEXTURE_PATH_NAME_LENGTH];
	
};





#endif