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

import com.newton.ndShapeInstanceGlue;

public class nShapeInstance
{
    public nShapeInstance(nShape shape)
    {
        m_nativeObject = new ndShapeInstanceGlue(shape.GetNativeObject());
    }

    //public nShapeInstance(nShapeInstance shapeInstance)
    //{
    //    m_nativeObject = new ndShapeInstanceGlue(shapeInstance.m_nativeObject);
    //}

    //public nShape GetShape()
    //{
    //    return m_shape;
    //}
    public ndShapeInstanceGlue GetNativeObject()
    {
        return m_nativeObject;
    }

    private ndShapeInstanceGlue m_nativeObject;
}
