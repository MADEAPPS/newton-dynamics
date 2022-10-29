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

package com.example.androidapp;

import android.app.ActivityManager;
import android.content.pm.ConfigurationInfo;
import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.view.View;

public class MainActivity extends AppCompatActivity
{
    // make sure you load the engine dynamic library
    static
    {
        System.loadLibrary("ndNewton");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);

		ActivityManager activityManager = (ActivityManager) getSystemService(ACTIVITY_SERVICE);
		ConfigurationInfo configurationInfo = activityManager.getDeviceConfigurationInfo();
		System.out.println(Double.parseDouble(configurationInfo.getGlEsVersion()));

		m_renderLoop = null;
    }

	@Override
	protected void onStart()
	{
		super.onStart();
		m_renderLoop = new RenderLoop(this);
		setContentView (m_renderLoop.GetView());
		m_renderLoop.start();
	}

	@Override
	protected void onStop()
	{
		super.onStop();
		m_renderLoop.OnTerminate();
	}

	@Override
	protected void onPause()
	{
		super.onPause();
		m_renderLoop.OnPause();
	}

	@Override
	protected void onResume()
	{
		super.onResume();
		m_renderLoop.OnResume();
	}

	@Override
	public void onWindowFocusChanged(boolean hasFocus)
	{
		super.onWindowFocusChanged(hasFocus);
		if (hasFocus) {
			getWindow().getDecorView().setSystemUiVisibility(
					View.SYSTEM_UI_FLAG_LAYOUT_STABLE |
					View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION |
					View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN |
					View.SYSTEM_UI_FLAG_HIDE_NAVIGATION	|
					View.SYSTEM_UI_FLAG_FULLSCREEN);
		}
	}

	private RenderLoop m_renderLoop;
}