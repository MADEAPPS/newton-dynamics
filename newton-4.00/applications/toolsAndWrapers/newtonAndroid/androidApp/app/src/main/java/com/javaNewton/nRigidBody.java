package com.javaNewton;

import com.newton.nRigidBodyType;
import com.newton.ndRigidBodyGlue;
import com.newton.ndShapeGlue;
import com.newton.ndShapeInstance;

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
        m_nativeObject.SetMassMatrix(mass, shape);
    }

    public void SetNotify(nBodyNotify notify)
    {
        m_notify = notify;
        notify.SetBody(this);
    }

    public int GetId()
    {
        return m_nativeObject.GetId();
    }

    public void SetCollisionShape(nShapeInstance shapeInstance)
    {
        m_nativeObject.SetCollisionShape(shapeInstance);
        m_shapeInstance = new nShapeInstance(new ndShapeGlue(m_nativeObject.GetShape()));
    }

    //public ndShapeInstanceGlue GetCollisionShape()
    //{
    //    return m_shapeInstance;
    //}

    public ndRigidBodyGlue GetNativeObject()
    {
        return m_nativeObject;
    }

    private nBodyNotify m_notify;
    private ndRigidBodyGlue m_nativeObject;
    private ndShapeInstance m_shapeInstance;
}
