package com.javaNewton;

import com.newton.ndShapeInstanceGlue;

public class nShapeInstance
{
    public nShapeInstance(nShape shape)
    {
        m_nativeObject = new ndShapeInstanceGlue(shape.GetNativeObject());
    }

    public nShapeInstance(ndShapeInstanceGlue nativeObject)
    {
        m_nativeObject = nativeObject;
    }

    public ndShapeInstanceGlue GetNativeObject()
    {
        return m_nativeObject;
    }

    private ndShapeInstanceGlue m_nativeObject;
}
