/* Copyright (c) <2003-2022> <Newton Game Dynamics>
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely
 */

package com.javaNewton;

import com.newton.nRigidBodyType;
import com.newton.ndRigidBodyGlue;
import com.newton.ndShapeInstanceGlue;

public class nRigidBody
{
    public nRigidBody(nRigidBodyType type)
    {
        m_notify = null;
        m_shapeInstance = null;
        m_nativeObject = new ndRigidBodyGlue(type);
    }

    public void SetMatrix(nMatrix matrix)
    {
        m_nativeObject.SetMatrix(matrix.CreateNative());
    }

    public void SetMassMatrix(float mass, nShapeInstance shape)
    {
        m_nativeObject.SetMassMatrix(mass, shape.GetNativeObject());
    }

    public void SetNotify(nBodyNotify notify)
    {
        m_notify = notify;
        m_nativeObject.SetNotifyCallback(notify);
    }

    public int GetId()
    {
        return m_nativeObject.GetId();
    }

    public void SetCollisionShape(nShapeInstance shapeInstance)
    {
        m_nativeObject.SetCollisionShape(shapeInstance.GetNativeObject());
    }

    public nShapeInstance GetCollisionShape()
    {
        return m_shapeInstance;
    }

    public ndRigidBodyGlue GetNativeObject()
    {
        return m_nativeObject;
    }

    private nBodyNotify m_notify;
    private nShapeInstance m_shapeInstance;
    private ndRigidBodyGlue m_nativeObject;
}
