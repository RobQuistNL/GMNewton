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

#ifndef _D_NODEINFO_H_
#define _D_NODEINFO_H_

#include "dScene.h"
#include "dVariable.h"

#define NE_MAX_NAME_SIZE	64


class dNodeInfo;


#define D_DEFINE_CLASS_NODE_ESSENCIALS(className,baseClass)	\
	dAddRtti(baseClass);									\
	virtual dNodeInfo* MakeCopy () const					\
	{														\
		return new className(*this);						\
	}														\
	virtual dNodeInfo* MetaFunction(dScene* const world) const	\
	{														\
		return new className(world);						\
	}														\
	static const char* BaseClassName ()						\
	{														\
		return #baseClass;									\
	}														\
	static const className& GetSingleton()					\
	{														\
		return m_singletonClass;							\
	}														\
	static className m_singletonClass;


#define D_DEFINE_CLASS_NODE(className,baseClass)			\
	virtual const char* GetClassName () const				\
	{														\
		return #className;									\
	}														\
	D_DEFINE_CLASS_NODE_ESSENCIALS(className,baseClass)		




#define D_IMPLEMENT_CLASS_NODE(className)			\
	dInitRtti(className);							\
	className className::m_singletonClass;			\
	static className::dRegisterSingleton m_registerSingletonAgent (#className, &className::m_singletonClass);


#define SerialiseBase(baseClass,rootNode)							\
	TiXmlElement* baseClassNode = new TiXmlElement (#baseClass);	\
	rootNode->LinkEndChild(baseClassNode);							\
	baseClass::Serialize(baseClassNode);							

#define DeserialiseBase(baseClass,rootNode,revision)												\
	TiXmlElement* baseClassNode = (TiXmlElement*) rootNode->FirstChild (baseClass::GetClassName());	\
	baseClass::Deserialize (baseClassNode, revision);


class dNodeInfo: public dClassInfo, public dVariableList
{
	public:

	enum dGizmoMode
	{
		m_selection,
		m_translation,
		m_rotation,
		m_scale
	};

	enum dGizmoHandle
	{
		m_noHandle,
		m_xHandle,
		m_yHandle,
		m_zHandle,
		m_xyHandle,
		m_yzHandle,
		m_zxHandle,
		m_xyzHandle
	};

	class dRegisterSingleton
	{	
		public:
		dRegisterSingleton (const char* const className, const dNodeInfo* const singleton);
	};

	
	dNodeInfo();
	dNodeInfo(const dNodeInfo& me);
	virtual ~dNodeInfo(void);
	virtual dNodeInfo* MakeCopy () const;
	virtual const char* GetClassName () const;		
	virtual const char* GetBaseClassName ()	const;
	virtual dNodeInfo* MetaFunction(dScene* const world) const;
	

	virtual const char* GetName () const;
	virtual void SetName (const char* name);
	
	virtual void SerializeBinary (FILE* const file) const;
	virtual void Serialize (TiXmlElement* const rootNode) const; 
	virtual bool Deserialize (TiXmlElement* const rootNode, int revisionNumber);

	// draw scene in wire frame mode
	virtual void DrawWireFrame(dScene* const world, dScene::dTreeNode* const myNode, const dVector& color) const{};

	// draw scene in solid wire frame mode
	virtual void DrawSolidWireFrame(dScene* const world, dScene::dTreeNode* const myNode, const dVector& color) const{};

	// draw scene in Gouraud shaded 
	virtual void DrawGouraudShaded(dScene* const world, dScene::dTreeNode* const myNode, const dVector& color) const{};

	// draw scene in using shaders 
	virtual void DrawShaded(dScene* const world, dScene::dTreeNode* const myNode, const dVector& color) const{};

	// Draw selection gizmo
	virtual void DrawGizmo(dScene* const world, dScene::dTreeNode* const myNode, const dMatrix& coordinaSystem, const dVector& color, dGizmoMode mode, dFloat size) const{};
	virtual dGizmoHandle GetHighlightedGizmoHandle(dScene* const world, dScene::dTreeNode* const myNode, const dMatrix& coordinaSystem, const dVector& screenPosition, dGizmoMode mode, dFloat size) const {return m_noHandle;}
	virtual void DrawGizmoHandle(dScene* world, const dMatrix& coordinaSystem, dGizmoMode mode, dGizmoHandle handle, const dVector& color, dFloat size) const {};


	virtual void BakeTransform (const dMatrix& transform){};
	virtual unsigned GetUniqueID() const {return m_uniqueID;}

//	virtual dVariableList& GetVariableList();
//	virtual dVariable* CreateVariable (const char* name);
//	virtual dVariable* FindVariable(const char* name) const;

	static dNodeInfo* CreateFromClassName (const char* className, dScene* world);
	static dTree<const dNodeInfo*, int>& GetSingletonDictionary();
	static void ReplaceSingletonClass (const char* const className, const dNodeInfo* const singleton);

	dAddRtti(dClassInfo);

	private:
	char m_name[NE_MAX_NAME_SIZE];
//	dVariableList m_variables;

	unsigned m_uniqueID;
	static unsigned m_uniqueIDCounter;
};





#endif
