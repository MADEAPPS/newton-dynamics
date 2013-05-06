/* Copyright (c) <2009> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#include "toolbox_stdafx.h"
#include "DemoDialogHelpers.h"
#include "DemoEntityManager.h"

/*
SelectThreadCount::SelectThreadCount(DemoEntityManager* const canvas)
	:QDialog (NULL)
{
	setWindowTitle (QApplication::translate("newtonMain", "Select micro threads", 0, QApplication::UnicodeUTF8));
	resize(256, 64);

	QVBoxLayout* const vbox = new QVBoxLayout(this);

	QLabel* const label = new QLabel ("xxx", this);
	m_slider = new QSlider (Qt::Horizontal, this);

	int maxthreads = NewtonGetMaxThreadsCount(canvas->GetNewton());
	int pos = NewtonGetThreadsCount(canvas->GetNewton());

	label->setNum (pos);
	m_slider->setMaximum(maxthreads);
	m_slider->setSliderPosition (pos);

	vbox->addWidget (label);
	vbox->addWidget (m_slider);
	connect (m_slider, SIGNAL (valueChanged(int)), label, SLOT (setNum (int)));

	setLayout(vbox); 
}

int SelectThreadCount::GetThreadCount() const 
{
	return m_slider->sliderPosition();
}



SelecCameraControl::SelecCameraControl (DemoEntityManager* const canvas)
	:QDialog (NULL)
{
	setWindowTitle (QApplication::translate("newtonMain", "Set Camera Controls", 0, QApplication::UnicodeUTF8));
	resize(256, 128);

	//		QVBoxLayout* const vbox = new QVBoxLayout(this);
	//		QLabel* const label = new QLabel ("xxx", this);
	//		m_slider = new QSlider (Qt::Horizontal, this);
	//		int maxthreads = NewtonGetMaxThreadsCount(canvas->GetNewton());
	//		int pos = NewtonGetThreadsCount(canvas->GetNewton());
	//		label->setNum (pos);
	//		m_slider->setMaximum(maxthreads);
	//		m_slider->setSliderPosition (pos);
	//		vbox->addWidget (label);
	//		vbox->addWidget (m_slider);
	//		connect (m_slider, SIGNAL (valueChanged(int)), label, SLOT (setNum (int)));
	//		setLayout(vbox); 
}

int SelecCameraControl::GetThreadCount() const 
{
	return m_slider->sliderPosition();
}

*/