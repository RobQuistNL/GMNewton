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

#ifndef _D_CAMERA_NODE_INFO_H_
#define _D_CAMERA_NODE_INFO_H_


#include "dNodeInfo.h"



class dScene;

class dCameraNodeInfo: public dNodeInfo
{
	public:
	D_DEFINE_CLASS_NODE(dCameraNodeInfo,dNodeInfo)

	dCameraNodeInfo();
	dCameraNodeInfo(dScene* const world);
	dCameraNodeInfo(const dCameraNodeInfo& me);
	
	virtual ~dCameraNodeInfo(void);

	

	// Set the Camera Matrix
	virtual void SetCameraMatrix(void) {};

	virtual void SetCameraYaw(dFloat yaw);
	virtual void SetCameraRoll(dFloat roll);
	virtual void SetCameraOrigin (const dVector& location);

	virtual dFloat GetCameraYaw() const;
	virtual dFloat GetCameraRoll() const;
	virtual const dVector& GetCameraOrigin () const;


	// Get/Set the view  mode for this camera
	virtual void BakeTransform (const dMatrix& transform) {};

	protected:
	virtual void SerializeBinary (FILE* const file);
	virtual void Serialize (TiXmlElement* const rootNode) const; 
	virtual bool Deserialize (TiXmlElement* const rootNode, int revisionNumber);
	
	// camera focus point
//	dVector m_pointOfInterest;
	dVector m_origin;
		
	// camera pitch angle
	dFloat m_roll;
	// camera yaw angle
	dFloat m_yaw;

	// my viewport index in teh editor
	int m_viewportIndex;

	friend class dScene;
};


#endif