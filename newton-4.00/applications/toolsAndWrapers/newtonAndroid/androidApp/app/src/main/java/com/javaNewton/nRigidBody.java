package com.javaNewton;

import com.newton.nRigidBodyType;
import com.newton.ndRigidBodyGlue;
import com.newton.ndShapeInstanceGlue;

public class nRigidBody
{
    public nRigidBody(nRigidBodyType type)
    {
        m_notify = null;
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
        notify.SetBody(this);
    }

    public int GetId()
    {
        return m_nativeObject.GetId();
    }

    public void SetCollisionShape(nShape shape)
    {
        m_shape = shape;
        m_nativeObject.SetCollisionShape(new ndShapeInstanceGlue(shape.GetNativeObject()));
    }

    public nShapeInstance GetCollisionShape()
    {
        return new nShapeInstance(m_nativeObject.GetCollisionShape());
    }

    public ndRigidBodyGlue GetNativeObject()
    {
        return m_nativeObject;
    }

    private nShape m_shape;
    private nBodyNotify m_notify;
    private ndRigidBodyGlue m_nativeObject;
}
