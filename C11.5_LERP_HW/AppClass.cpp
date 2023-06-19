#include "AppClass.h"
void Application::InitVariables(void)
{
	////Change this to your name and email
	m_sProgrammer = "Michael Eaton - me3870@rit.edu";
	vector3 v3Position(0.0f, 0.0f, 30.0f);
	vector3 v3Target = ZERO_V3;
	vector3 v3Upward = AXIS_Y;
	m_pCameraMngr->SetPositionTargetAndUpward(v3Position, v3Target, v3Upward);
	//init the mesh
	m_pMesh = new MyMesh();
	//m_pMesh->GenerateCube(1.0f, C_WHITE);
	m_pMesh->GenerateCube(1.0f, C_BLACK);
}
void Application::Update(void)
{
	//Update the system so it knows how much time has passed since the last call
	m_pSystem->Update();

	//Is the arcball active?
	ArcBall();

	//Create a model matrix out of the arcball quaternion
	matrix4 m4Model = ToMatrix4(m_qArcBall);

	//Is the first person camera active?
	CameraRotation();

	//Update Entity Manager
	m_pEntityMngr->Update();

	//Add objects to render list
	m_pEntityMngr->AddEntityToRenderList(-1, true);
}
void Application::Display(void)
{
	// Clear the screen
	ClearScreen();

	matrix4 m4View = m_pCameraMngr->GetViewMatrix();
	matrix4 m4Projection = m_pCameraMngr->GetProjectionMatrix();


	static float timer = 0;
	static uint clock = m_pSystem->GenClock();
	timer += m_pSystem->GetDeltaTime(clock); 

	
	static std::vector<vector3> stops;
	static bool bInit = false;
	if (!bInit)
	{
		stops.push_back(vector3(-4.0f, -2.0f, 5.0f));
		stops.push_back(vector3(1.0f, -2.0f, 5.0f));
		stops.push_back(vector3(-3.0f, -1.0f, 3.0f));
		stops.push_back(vector3(2.0f, -1.0f, 3.0f));
		stops.push_back(vector3(-2.0f, 0.0f, 0.0f));
		stops.push_back(vector3(3.0f, 0.0f, 0.0f));
		stops.push_back(vector3(-1.0f, 1.0f, -3.0f));
		stops.push_back(vector3(4.0f, 1.0f, -3.0f));
		stops.push_back(vector3(0.0f, 2.0f, -5.0f));
		stops.push_back(vector3(5.0f, 2.0f, -5.0f));
		stops.push_back(vector3(1.0f, 3.0f, -5.0f));
		bInit = true;
	}
	for (uint i = 0; i < stops.size(); ++i)
	{
		m_pModelMngr->AddSphereToRenderList(glm::translate(stops[i]) * glm::scale(vector3(0.15f)), C_RED, RENDER_WIRE);
	}

	vector3 v3Start;
	vector3 v3End;
	static uint route = 0; 
	v3Start = stops[route]; 
	v3End = stops[(route + 1) % stops.size()]; 

	float fTimeBetweenStops = 2.0;
	float fPercentage = MapValue(timer, 0.0f, fTimeBetweenStops, 0.0f, 1.0f);

	vector3 v3CurrentPos = glm::lerp(v3Start, v3End, fPercentage);
	matrix4 m4Model = glm::translate(IDENTITY_M4, v3CurrentPos);

	if (fPercentage >= 1.0f)
	{
		route++;
		timer = m_pSystem->GetDeltaTime(clock);
		route %= stops.size();
	}

	m_pMesh->Render(m4Projection, m4View, m4Model);

	


	// draw a skybox
	m_pModelMngr->AddSkyboxToRenderList();

	//render list call
	m_uRenderCallCount = m_pModelMngr->Render();

	//clear the render list
	m_pModelMngr->ClearRenderList();

	//draw gui
	DrawGUI();
	
	//end the current frame (internally swaps the front and back buffers)
	m_pWindow->display();
}
void Application::Release(void)
{
	SafeDelete(m_pMesh);
	//release GUI
	ShutdownGUI();
}