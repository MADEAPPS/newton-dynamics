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

// this vehicle UI class was implemented by Dave Gravel,
// so sole some of the Open Gl errors with legacy glBegin/GlEnd 
// operation with deprecated since OpenGl 3.3 
// Thank you very much Dave

#include "ndSandboxStdafx.h"
#include "ndVehicleUI.h"
#include "ndPngToOpenGl.h"
#include "ndDemoEntityManager.h"

const GLchar* ndVehicleUI::m_vertexShader = R""""(

	#version 330 core

	in vec3 Position;
	in vec2 UV;
	out vec2 Frag_UV;
	out vec4 Frag_Color;
	uniform mat4 ProjMtx;
	uniform mat4 ModMtx;
	uniform float ptsize;
	uniform vec4 color;

	void main()
	{
		Frag_UV = UV;
		Frag_Color = color;
		gl_Position = ProjMtx * ModMtx * vec4(Position.xy * ptsize,0.0,1.0);
	}
)"""";

const GLchar* ndVehicleUI::m_fragmentShader = R""""(

	#version 330 core

	uniform sampler2D UIText;
	in vec2 Frag_UV;
	in vec4 Frag_Color;
	out vec4 Out_Color;

	void main()
	{
		Out_Color = Frag_Color * texture(UIText, Frag_UV.st);
	}
)"""";

ndVehicleUI::ndVehicleUI(ndDemoEntityManager* const scene)
	:ndUIEntity(scene)
	,m_shaderHandle(0)
	,m_vehicle(nullptr)
	,m_vboDyn(0)
	,m_vboSta(0)
	,m_vaoDyn(0)
	,m_vaoSta(0)
	,m_iboDyn(0)
	,m_iboSta(0)
{
	m_gears = LoadTexture("gears_font.png");
	m_odometer = LoadTexture("kmh_dial.png");
	m_tachometer = LoadTexture("rpm_dial.png");
	m_redNeedle = LoadTexture("needle_red.png");
	m_greenNeedle = LoadTexture("needle_green.png");

	CreateBufferUI();
};

ndVehicleUI::~ndVehicleUI()
{
	ReleaseTexture(m_gears);
	ReleaseTexture(m_odometer);
	ReleaseTexture(m_redNeedle);
	ReleaseTexture(m_tachometer);
	ReleaseTexture(m_greenNeedle);

	if (m_shaderHandle)
	{
		glDeleteProgram(m_shaderHandle);
	}

	if (m_iboSta)
	{
		glDeleteBuffers(1, &m_iboSta);
	}

	if (m_vboSta)
	{
		glDeleteBuffers(1, &m_vboSta);
	}

	if (m_vaoSta)
	{
		glDeleteVertexArrays(1, &m_vaoSta);
	}
	if (m_iboDyn)
	{
		glDeleteBuffers(1, &m_iboDyn);
	}

	if (m_vboDyn)
	{
		glDeleteBuffers(1, &m_vboDyn);
	}

	if (m_vaoDyn)
	{
		glDeleteVertexArrays(1, &m_vaoDyn);
	}
};

void ndVehicleUI::SetVehicle(ndMultiBodyVehicle* const vehicle)
{
	m_vehicle = vehicle;
}

void ndVehicleUI::CreateOrthoViewMatrix(ndFloat32 origin_x, const ndFloat32 origin_y, ndMatrix& projmatrix)
{
	ndFloat32 sizeX = (ndFloat32)(1.0f * (ndFloat32)m_scene->GetWidth());
	ndFloat32 sizeY = (ndFloat32)(1.0f * (ndFloat32)m_scene->GetHeight());
	ndFloat32 L = origin_x;
	ndFloat32 R = origin_x + sizeX;
	ndFloat32 T = origin_y;
	ndFloat32 B = origin_y + sizeY;
	//projmatrix = ndMatrix(
	//{ 2.0f / (R - L), 0.0f,   0.0f, 0.0f },
	//{ 0.0f,   2.0f / (T - B), 0.0f, 0.0f },
	//{ 0.0f,   0.0f,          -1.0f, 0.0f },
	//{ (R + L) / (L - R), (T + B) / (B - T), 0.0f, 1.0f });

	projmatrix = ndMatrix(
		ndVector(2.0f / (R - L), 0.0f, 0.0f, 0.0f),
		ndVector(0.0f, 2.0f / (T - B), 0.0f, 0.0f),
		ndVector(0.0f, 0.0f, -1.0f, 0.0f),
		ndVector((R + L) / (L - R), (T + B) / (B - T), 0.0f, 1.0f));

}

void ndVehicleUI::CreateBufferUI()
{
	if (!m_vaoDyn)
	{
		m_shaderHandle = glCreateProgram();

		GLuint m_vertHandle = glCreateShader(GL_VERTEX_SHADER);
		GLuint m_fragHandle = glCreateShader(GL_FRAGMENT_SHADER);

		GLint Result = GL_FALSE;
		ndInt32 InfoLogLength = 0;

		glShaderSource(m_vertHandle, 1, &m_vertexShader, NULL);
		glCompileShader(m_vertHandle);
		// Check Vertex Shader
		glGetShaderiv(m_vertHandle, GL_COMPILE_STATUS, &Result);
		glGetShaderiv(m_vertHandle, GL_INFO_LOG_LENGTH, &InfoLogLength);
		if (InfoLogLength > 0)
		{
			printf("Vertex shader error! \n");
			//	std::vector<char> VertexShaderErrorMessage(InfoLogLength + 1);
			//	glGetShaderInfoLog(g_VertHandle3D, InfoLogLength, NULL, &VertexShaderErrorMessage[0]);
			//	printf("Vertex %s\n", &VertexShaderErrorMessage[0]);
		}
		glShaderSource(m_fragHandle, 1, &m_fragmentShader, NULL);
		glCompileShader(m_fragHandle);

		// Check Fragment Shader
		glGetShaderiv(m_fragHandle, GL_COMPILE_STATUS, &Result);
		glGetShaderiv(m_fragHandle, GL_INFO_LOG_LENGTH, &InfoLogLength);
		if (InfoLogLength > 0)
		{
			printf("Fragment shader error! \n");
			//	std::vector<char> FragmentShaderErrorMessage(InfoLogLength + 1);
			//	glGetShaderInfoLog(g_FragHandle3D, InfoLogLength, NULL, &FragmentShaderErrorMessage[0]);
			//	printf("Fragment %s\n", &FragmentShaderErrorMessage[0]);
		}

		glAttachShader(m_shaderHandle, m_vertHandle);
		glAttachShader(m_shaderHandle, m_fragHandle);

		glLinkProgram(m_shaderHandle);

		glDetachShader(m_shaderHandle, m_vertHandle);
		glDetachShader(m_shaderHandle, m_fragHandle);

		glDeleteShader(m_vertHandle);
		glDeleteShader(m_fragHandle);

		m_vertDyn[0].m_posit.m_x = -1.0f;
		m_vertDyn[0].m_posit.m_y = -1.0f;
		m_vertDyn[0].m_posit.m_z = 0.0f;
		m_vertDyn[0].m_uv.m_u = 0.0f;
		m_vertDyn[0].m_uv.m_v = 0.0f;
		//
		m_vertDyn[1].m_posit.m_x = -1.0f;
		m_vertDyn[1].m_posit.m_y = 1.0f;
		m_vertDyn[1].m_posit.m_z = 0.0f;
		m_vertDyn[1].m_uv.m_u = 0.0f;
		m_vertDyn[1].m_uv.m_v = 1.0f;

		m_vertDyn[2].m_posit.m_x = 1.0f;
		m_vertDyn[2].m_posit.m_y = 1.0f;
		m_vertDyn[2].m_posit.m_z = 0.0f;
		m_vertDyn[2].m_uv.m_u = -1.0f;
		m_vertDyn[2].m_uv.m_v = 1.0f;

		m_vertDyn[3].m_posit.m_x = 1.0f;
		m_vertDyn[3].m_posit.m_y = -1.0f;
		m_vertDyn[3].m_posit.m_z = 0.0f;
		m_vertDyn[3].m_uv.m_u = -1.0f;
		m_vertDyn[3].m_uv.m_v = 0.0f;

		m_indxDyn[0] = 0;
		m_indxDyn[1] = 1;
		m_indxDyn[2] = 2;
		m_indxDyn[3] = 2;
		m_indxDyn[4] = 3;
		m_indxDyn[5] = 0;

		glGenVertexArrays(1, &m_vaoDyn);
		glBindVertexArray(m_vaoDyn);

		glGenBuffers(1, &m_vboDyn);
		glBindBuffer(GL_ARRAY_BUFFER, m_vboDyn);
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_vertDyn), &m_vertDyn[0], GL_DYNAMIC_DRAW);

		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glPositionUV), (void*)0);

		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(glPositionUV), (void*)(offsetof(glPositionUV, m_uv)));

		glGenBuffers(1, &m_iboDyn);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_iboDyn);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(m_indxDyn), &m_indxDyn[0], GL_STATIC_DRAW);

		// Don't unbind this buffer, Let's it in opengl memory.
		//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

		// remove this buffer from memory because it is updated in runtime.
		// you need to bind this buffer at any render pass.
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glBindVertexArray(0);

		// Gear dynamic buffer
		memcpy(m_vertSta, m_vertDyn, sizeof(m_vertDyn));
		memcpy(m_indxSta, m_indxDyn, sizeof(m_indxDyn));
		//
		//
		glGenVertexArrays(1, &m_vaoSta);
		glBindVertexArray(m_vaoSta);
		//
		glGenBuffers(1, &m_vboSta);
		glBindBuffer(GL_ARRAY_BUFFER, m_vboSta);
		glBufferData(GL_ARRAY_BUFFER, sizeof(m_vertSta), &m_vertSta[0], GL_STATIC_DRAW);
		
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glPositionUV), (void*)0);
		
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(glPositionUV), (void*)(offsetof(glPositionUV, m_uv)));

		glGenBuffers(1, &m_iboSta);
		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_iboSta);
		glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(m_indxSta), &m_indxSta[0], GL_STATIC_DRAW);
		
		glBindVertexArray(0);

		glUseProgram(m_shaderHandle);

		m_colorLocation = glGetUniformLocation(m_shaderHandle, "color");
		m_projMtxLocation = glGetUniformLocation(m_shaderHandle, "ProjMtx");
		m_modMtxLocation = glGetUniformLocation(m_shaderHandle, "ModMtx");
		m_ptsizeLocation = glGetUniformLocation(m_shaderHandle, "ptsize");

		glUseProgram(0);
	}
};

void ndVehicleUI::RenderGageUI(const GLuint tex1, const ndFloat32 origin_x, const ndFloat32 origin_y, const ndFloat32 ptsize, ndFloat32 cparam, ndFloat32 minAngle, ndFloat32 maxAngle)
{
	if (m_vaoSta)
	{
		ndMatrix aprojm(ndGetIdentityMatrix());
		CreateOrthoViewMatrix(origin_x, origin_y, aprojm);
		
		minAngle *= -ndDegreeToRad;
		maxAngle *= -ndDegreeToRad;
		
		ndFloat32 angle = minAngle + (maxAngle - minAngle) * cparam;

		ndMatrix modm(ndRollMatrix(-angle));
		glVector4 color(GLfloat(1.0f), GLfloat(1.0f), GLfloat(1.0f), GLfloat(1.0f));

		glMatrix glModm(modm);
		glMatrix glAprojm(aprojm);

		glUniformMatrix4fv(m_projMtxLocation, 1, GL_FALSE, &glAprojm[0][0]);
		glUniformMatrix4fv(m_modMtxLocation, 1, GL_FALSE, &glModm[0][0]);
		glUniform1f(m_ptsizeLocation, GLfloat(ptsize));
		glUniform4fv(m_colorLocation, 1, &color[0]);

		glBindVertexArray(m_vaoSta);

		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);

		// This buffer is already in the opengl memory.
		// Don't need to bind it again.
		//glBindBuffer(GL_ARRAY_BUFFER, m_vboSta);

		if (tex1)
		{
			glBindTexture(GL_TEXTURE_2D, tex1);
		}

		// This buffer is already in the memory.
		// Don't need to bind it again.
		//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_iboSta);

		glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

		// Don't unbind this buffers from the opengl memory.
		//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		//glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
	}
};

void ndVehicleUI::RenderGearUI(const ndInt32 gearid, GLuint tex1, ndFloat32 origin_x, ndFloat32 origin_y, ndFloat32 ptsize)
{
	if (m_vaoDyn)
	{
		ndMatrix aprojm(ndGetIdentityMatrix());
		CreateOrthoViewMatrix(origin_x, origin_y, aprojm);

		ndMatrix origin(ndGetIdentityMatrix());
		origin[1][1] = -1.0f;
		origin.m_posit = ndVector(origin_x + ptsize * 1.9f, 50.0f, 0.0f, 1.0f);

		ndFloat32 uwith = 0.1f;
		ndFloat32 u0 = uwith * (ndFloat32)gearid;
		ndFloat32 u1 = u0 + uwith;
		ndFloat32 xy1 = 10.0f;

		glVector4 color;
		if (gearid == 0)
		{
			color = ndVector(1.0f, 0.5f, 0.0f, 1.0f);
		}
		else if (gearid == 1)
		{
			color = ndVector(1.0f, 1.0f, 0.0f, 1.0f);
		}
		else
		{
			color = ndVector(0.0f, 1.0f, 0.0f, 1.0f);
		}

		glMatrix glOrigin(origin);
		glMatrix glAprojm(aprojm);
		glUniformMatrix4fv(m_projMtxLocation, 1, GL_FALSE, &glAprojm[0][0]);
		glUniformMatrix4fv(m_modMtxLocation, 1, GL_FALSE, &glOrigin[0][0]);
		glUniform1f(m_ptsizeLocation, GLfloat(xy1));
		glUniform4fv(m_colorLocation, 1, &color[0]);

		glBindVertexArray(m_vaoDyn);

		glEnableVertexAttribArray(0);
		glEnableVertexAttribArray(1);

		//
		glBindBuffer(GL_ARRAY_BUFFER, m_vboDyn);

		m_vertDyn[0].m_uv.m_u = GLfloat(u0);
		m_vertDyn[1].m_uv.m_u = GLfloat(u0);

		m_vertDyn[2].m_uv.m_u = GLfloat(u1);
		m_vertDyn[3].m_uv.m_u = GLfloat(u1);

		// Bind and update the dynamic buffer uv data 
		glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(m_vertDyn), &m_vertDyn[0]);

		if (tex1)
		{
			glBindTexture(GL_TEXTURE_2D, tex1);
		}

		// This buffer is static, Don't need to bind it again.
		// The buffer is already bind in the opengl memory.
		//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_iboDyn);

		glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

		// Don't unbind this buffer to let's it in opengl memory.
		//glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);
	}
};

void ndVehicleUI::RenderHelp()
{
	ndVector color(1.0f, 1.0f, 0.0f, 0.0f);
	m_scene->Print(color, "Vehicle driving keyboard control");
	m_scene->Print(color, "change vehicle     : 'c'");
	m_scene->Print(color, "accelerator        : 'w'");
	m_scene->Print(color, "brakes             : 's'");
	m_scene->Print(color, "turn left          : 'a'");
	m_scene->Print(color, "turn right         : 'd'");
	m_scene->Print(color, "hand brakes        : 'space'");

	ImGui::Separator();
	m_scene->Print(color, "gear box");
	m_scene->Print(color, "ignition            : 'i'");
	m_scene->Print(color, "manual transmission : '?'");
	m_scene->Print(color, "neutral gear	    : 'n'");
	m_scene->Print(color, "forward gear up     : '>'");
	m_scene->Print(color, "forward gear down   : '<'");
	m_scene->Print(color, "reverse gear	    : 'r'");
	m_scene->Print(color, "parking gear	    : 'p'");
}

//void ndVehicleUI::RenderDash(ndDemoEntityManager* const scene, ndMultiBodyVehicle* const vehicle)
void ndVehicleUI::RenderUI()
{
	if (!m_vehicle)
	{
		return;
	}
	//ndFloat32 gageSize = 200.0f;
	//ndFloat32 y = (ndFloat32)m_scene->GetHeight() - (gageSize / 2.0f + 20.0f);
	
	// draw the tachometer
	//ndFloat32 x = gageSize / 2 + 20.0f;
	//ndFloat32 maxRpm = m_vehicle->m_configuration.m_engine.GetRedLineRadPerSec() * dRadPerSecToRpm;
	//maxRpm += 500.0f;

	//ndAssert(0);
	//ndFloat32 rpm = (m_vehicle->m_motor->GetRpm() / maxRpm) * 2.85f;
	//
	//glUseProgram(m_shaderHandle);
	//
	//glActiveTexture(GL_TEXTURE0);
	//
	//RenderGageUI(m_tachometer, -x, -y, gageSize * 0.5f, 0.0f, -180.0f, 90.0f);
	//
	//ndFloat32 s = gageSize * 0.7f;
	//RenderGageUI(m_redNeedle, -x, -y, s * 0.5f, rpm, -0.0f, 90.0f);
	//
	//x += gageSize;
	//RenderGageUI(m_odometer, -x, -y, gageSize * 0.5f, 0.0f, -180.0f, 90.0f);
	//
	//ndFloat32 speed = (m_vehicle->GetSpeed() / 100.0f) * 2.85f;
	//RenderGageUI(m_greenNeedle, -x, -y, s * 0.5f, ndAbs(speed), -0.0f, 90.0f);
	//
	//// draw the current gear
	//ndInt32 gearMap[8];
	//gearMap[sizeof(m_vehicle->m_configuration.m_transmission.m_forwardRatios) / sizeof(m_vehicle->m_configuration.m_transmission.m_forwardRatios[0]) + 0] = 1;
	//gearMap[sizeof(m_vehicle->m_configuration.m_transmission.m_forwardRatios) / sizeof(m_vehicle->m_configuration.m_transmission.m_forwardRatios[0]) + 1] = 0;
	//for (ndInt32 i = 0; i < m_vehicle->m_configuration.m_transmission.m_gearsCount; ++i)
	//{
	//	gearMap[i] = i + 2;
	//}
	//RenderGearUI(gearMap[m_vehicle->m_currentGear], m_gears, -x, -y, gageSize);
	//
	//glUseProgram(0);
}