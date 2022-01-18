//Precompiled Header [ALWAYS ON TOP IN CPP]
#include "stdafx.h"

//Includes
#include "App_Flocking.h"
#include "../SteeringAgent.h"
#include "TheFlock.h"

using namespace Elite;

//Destructor
App_Flocking::~App_Flocking()
{	
	SAFE_DELETE(m_pWander);
	
	SAFE_DELETE(m_pFlock);
	SAFE_DELETE(m_pAgentToEvade);
}

//Functions
void App_Flocking::Start()
{
	DEBUGRENDERER2D->GetActiveCamera()->SetZoom(55.0f);
	DEBUGRENDERER2D->GetActiveCamera()->SetCenter(Elite::Vector2(m_TrimWorldSize / 1.5f, m_TrimWorldSize / 2));

	// Agent information
	m_pWander = new Wander();
	m_pAgentToEvade = new SteeringAgent();
	m_pAgentToEvade->SetSteeringBehavior(m_pWander);
	m_pAgentToEvade->SetMaxLinearSpeed(15.f);
	m_pAgentToEvade->SetMass(1.f);
	m_pAgentToEvade->SetBodyColor(Color{ 1, 0, 0 });
	m_pAgentToEvade->SetAutoOrient(true);

	m_pFlock = new Flock(m_FlockSize, m_TrimWorldSize, m_pAgentToEvade, true);
}

void App_Flocking::Update(float deltaTime)
{
	//INPUT
	if (INPUTMANAGER->IsMouseButtonUp(InputMouseButton::eLeft) && m_VisualizeMouseTarget)
	{
		auto const mouseData = INPUTMANAGER->GetMouseData(InputType::eMouseButton, InputMouseButton::eLeft);
		m_MouseTarget.Position = DEBUGRENDERER2D->GetActiveCamera()->ConvertScreenToWorld({ static_cast<float>(mouseData.X), static_cast<float>(mouseData.Y) });
	}

	//Agent
	m_pAgentToEvade->Update(deltaTime);

	//The flock
	if (m_pFlock->CanTrimWorld())
	{
		m_TrimWorldSize = m_pFlock->GetWorldSize();
		m_pAgentToEvade->TrimToWorld(m_TrimWorldSize);
	}

	m_pFlock->UpdateAndRenderUI();
	m_pFlock->Update(deltaTime);

	if (m_UseMouseTarget)
		m_pFlock->SetSeekTarget(m_MouseTarget);

	//Debug Rendering
	if (m_pAgentToEvade->CanRenderBehavior() != m_pFlock->CanDebugRender())
	{
		m_pAgentToEvade->SetRenderBehavior(m_pFlock->CanDebugRender());
	}
}

void App_Flocking::Render(float deltaTime) const
{
	if (m_pFlock->CanTrimWorld())
	{
		std::vector<Elite::Vector2> points =
		{
			{ -m_TrimWorldSize,m_TrimWorldSize },
			{ m_TrimWorldSize,m_TrimWorldSize },
			{ m_TrimWorldSize,-m_TrimWorldSize },
			{-m_TrimWorldSize,-m_TrimWorldSize }
		};
		
		DEBUGRENDERER2D->DrawPolygon(&points[0], 4, { 1,0,0,1 }, 0.4f);
	}

	m_pFlock->Render(deltaTime);
	m_pAgentToEvade->Render(deltaTime);

	//Render Target
	if(m_VisualizeMouseTarget)
		DEBUGRENDERER2D->DrawSolidCircle(m_MouseTarget.Position, 0.3f, { 0.f,0.f }, { 1.f,0.f,0.f },-0.8f);
}
