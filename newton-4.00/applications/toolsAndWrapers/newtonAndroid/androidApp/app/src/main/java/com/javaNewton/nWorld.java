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

import com.newton.ndWorldGlue;
import java.util.Map;
import java.util.HashMap;
import java.util.Iterator;
import java.util.ListIterator;

public class nWorld
{
    public nWorld()
    {
        m_bodyMap = new HashMap<>();
        m_nativeObject = new ndWorldGlue();
    }

    public void CleanUp()
    {
        Iterator<Map.Entry<Integer, nRigidBody>> it = m_bodyMap.entrySet().iterator();
        while (it.hasNext())
        {
            Map.Entry<Integer, nRigidBody> entry = (Map.Entry<Integer, nRigidBody>) it.next();
            nRigidBody body = entry.getValue();
            m_nativeObject.RemoveBody(body.GetNativeObject());
        }
        m_bodyMap.clear();
    }

    public void Sync()
    {
        m_nativeObject.Sync();
    }

    public void SetSubSteps(int substeps)
    {
        m_nativeObject.SetSubSteps(substeps);
    }

    public void Update(float timestep)
    {
        m_nativeObject.Update(timestep);
    }

    public void AddBody(nRigidBody body)
    {
        m_nativeObject.AddBody(body.GetNativeObject());
        m_bodyMap.put (body.GetId(), body);
    }

    public void RemoveBody(nRigidBody body)
    {
        m_nativeObject.RemoveBody(body.GetNativeObject());
        m_bodyMap.remove (body.GetId());
    }

    private ndWorldGlue m_nativeObject;
    private HashMap<Integer, nRigidBody> m_bodyMap = new HashMap<>();
}
