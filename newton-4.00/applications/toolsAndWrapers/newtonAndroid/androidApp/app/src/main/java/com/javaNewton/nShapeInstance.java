package com.javaNewton;

import com.newton.ndShapeInstanceGlue;

public class nShapeInstance
{
    public nShapeInstance(nShape shape)
    {
        m_shape = shape;
        m_nativeObject = new ndShapeInstanceGlue(shape.GetNativeObject());
    }

    public nShape GetShape()
    {
        return m_shape;
    }

    public ndShapeInstanceGlue GetNativeObject()
    {
        return m_nativeObject;
    }

    private nShape m_shape;
    private ndShapeInstanceGlue m_nativeObject;
}
