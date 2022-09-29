package com.javaNewton;

import com.newton.ndShapeGlue;
import com.newton.ndShapeInstanceGlue;

public class nShapeInstance
{
    public nShapeInstance(ndShapeGlue shape)
    {
        m_nativeObject = new ndShapeInstanceGlue(shape);
    }

    public ndShapeInstanceGlue GetNativeObject()
    {
        return m_nativeObject;
    }

    private ndShapeInstanceGlue m_nativeObject;
}
