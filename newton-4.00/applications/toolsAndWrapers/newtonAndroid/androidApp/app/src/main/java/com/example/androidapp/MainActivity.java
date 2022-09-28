package com.example.androidapp;

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

	private RenderLoop m_renderLoop;

    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.activity_main);

		m_renderLoop = new RenderLoop(this);
		setContentView (m_renderLoop.GetView());
		m_renderLoop.start();
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
	protected void onStop()
	{
		super.onStop();
		m_renderLoop.OnTerminate();
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
}