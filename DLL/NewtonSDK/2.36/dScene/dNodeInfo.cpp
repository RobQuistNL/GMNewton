/////////////////////////////////////////////////////////////////////////////
// Name:        dNodeInfo.h
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
#include "dNodeInfo.h"

dInitRtti(dNodeInfo);



unsigned dNodeInfo::m_uniqueIDCounter = 0;

dTree<const dNodeInfo*, int>& dNodeInfo::GetSingletonDictionary()
{
	static dTree<const dNodeInfo*, int> dictionary;
	return dictionary;
}


dNodeInfo::dRegisterSingleton::dRegisterSingleton (const char* const className, const dNodeInfo* const singleton)
{
	int crc = dCRC (className);
	dTree<const dNodeInfo*, int>& dictionary = dNodeInfo::GetSingletonDictionary();
	dictionary.Insert (singleton, crc);
}



dNodeInfo::dNodeInfo() 
	:dClassInfo(), dVariableList() 
{
	m_name[0] = 0; 
	m_uniqueID = m_uniqueIDCounter;
	m_uniqueIDCounter ++;
}


dNodeInfo::dNodeInfo(const dNodeInfo& me)
	:dClassInfo(), dVariableList(me), m_uniqueID (me.m_uniqueID)
{
	SetName (me.m_name);
}

dNodeInfo::~dNodeInfo(void)
{
}

dNodeInfo* dNodeInfo::MakeCopy () const
{
	return new dNodeInfo();
}

dNodeInfo* dNodeInfo::MetaFunction(dScene* const world) const
{
	return MakeCopy();
}

void dNodeInfo::ReplaceSingletonClass (const char* const className, const dNodeInfo* const singleton)
{
	int crc = dCRC (className);
	dTree<const dNodeInfo*, int>& dictionary = dNodeInfo::GetSingletonDictionary();
	dTree<const dNodeInfo*, int>::dTreeNode* const node = dictionary.Find(crc);
	if (node) {
		node->GetInfo() = singleton;
	} else {
		dRegisterSingleton (className, singleton);
	}
}

dNodeInfo* dNodeInfo::CreateFromClassName (const char* className, dScene* world)
{
	int crc = dCRC (className);
	dTree<const dNodeInfo*, int>& dictionary = dNodeInfo::GetSingletonDictionary();
	dTree<const dNodeInfo*, int>::dTreeNode* const node = dictionary.Find(crc);
	if (node) {
		const dNodeInfo* const singleton = node->GetInfo();
		return singleton->MetaFunction(world);
	}
	return NULL;
}


const char* dNodeInfo::GetName () const
{
	return m_name;
}

void dNodeInfo::SetName (const char* name)
{
	strncpy (m_name, name, NE_MAX_NAME_SIZE - 1);
}

const char* dNodeInfo::GetClassName () const
{
	return "dNodeInfo";
}

const char* dNodeInfo::GetBaseClassName ()	const
{
	return "";
}

/*
dVariableList& dNodeInfo::GetVariableList()
{
	return m_variables; 
}

dVariable* dNodeInfo::FindVariable(const char* name) const
{
	return m_variables.FindVariable(name);
}

dVariable* dNodeInfo::CreateVariable (const char* name)
{
	return m_variables.CreateVariable(name);
}
*/





void dNodeInfo::SerializeBinary (FILE* const file) const
{
	_ASSERTE (0);
}

void dNodeInfo::Serialize (TiXmlElement* const rootNode) const
{
	rootNode->SetAttribute("name", m_name);
	dVariableList::Serialize(rootNode);
}


bool dNodeInfo::Deserialize (TiXmlElement* const rootNode, int revisionNumber)
{
	SetName (rootNode->Attribute("name"));
	dVariableList::Deserialize(rootNode);

	return true;
}