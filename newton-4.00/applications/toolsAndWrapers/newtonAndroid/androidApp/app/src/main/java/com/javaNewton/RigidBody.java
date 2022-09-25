package com.javaNewton;

import com.newton.nBodyNotify;
import com.newton.nRigidBody;

public class RigidBody extends nRigidBody
{
    private nBodyNotify notify;

    public RigidBody(nRigidBody.Type type)
    {
        super(type);
    }

    @Override
    public void SetNotifyCallback(nBodyNotify notifyCallBack)
    {
        notify = notifyCallBack;
        super.SetNotifyCallback(notify);
    }
}
