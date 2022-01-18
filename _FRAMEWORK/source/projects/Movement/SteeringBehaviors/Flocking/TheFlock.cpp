#include "stdafx.h"
#include "TheFlock.h"

#include "../SteeringAgent.h"
#include "../Steering/SteeringBehaviors.h"
#include "../CombinedSteering/CombinedSteeringBehaviors.h"

//Constructor & Destructor
Flock::Flock(
	int flockSize /*= 50*/, 
	float worldSize /*= 100.f*/, 
	SteeringAgent* pAgentToEvade /*= nullptr*/, 
	bool trimWorld /*= false*/)

	: m_WorldSize{ worldSize }
	, m_FlockSize{ flockSize }
	, m_CanTrimWorld { trimWorld }
	, m_pAgentToEvade{pAgentToEvade}
	, m_NeighborhoodRadius{ 15 }
	, m_NrOfNeighbors{0}
	, m_CanDebugRender{false}
	, m_NrOfCellsX{ 25 }
	, m_UseSpacePartitioning{true}
{
	m_Agents.resize(m_FlockSize);
	m_Neighbors.resize(m_FlockSize);
	m_OldPositions.resize(m_FlockSize);

	// Initializing steering behaviors
	m_pSeparationBehavior = new Separation(this);
	m_pCohesionBehavior = new Cohesion(this);
	m_pVelMatchBehavior = new VelocityMatch(this);
	m_pSeekBehavior = new Seek();
	m_pWanderBehavior = new Wander();
	m_pEvadeBehavior = new Evade();

	m_pEvadeBehavior->SetEvadeRadius(20.f);

	m_pBlendedSteering = new BlendedSteering({ { m_pSeparationBehavior, 0.2f }, { m_pCohesionBehavior, 0.2f }, 
		{ m_pVelMatchBehavior, 0.2f }, { m_pSeekBehavior, 0.2f }, { m_pWanderBehavior, 0.2f } });

	m_pPrioritySteering = new PrioritySteering({ m_pEvadeBehavior, m_pBlendedSteering });

	// Initializing space partitioning
	m_pPartitionedSpace = new CellSpace(m_WorldSize, m_WorldSize, m_NrOfCellsX, m_NrOfCellsX, m_FlockSize);

	// Initializing all agents
	for (int i{}; i < m_FlockSize; ++i)
	{
		m_Agents[i] = new SteeringAgent();
		int spawnSize{ static_cast<int>(m_WorldSize) };
		int x{ rand() % (spawnSize * 2) };
		int y{ rand() % (spawnSize * 2) };
		Elite::Vector2 position = Elite::Vector2(static_cast<float>(x) - spawnSize, static_cast<float>(y) - spawnSize);
		m_Agents[i]->SetPosition(position);
		m_OldPositions[i] = position;

		m_Agents[i]->SetMaxLinearSpeed(15.f);
		m_Agents[i]->SetMass(1.f);
		m_Agents[i]->SetSteeringBehavior(m_pPrioritySteering);
		m_Agents[i]->SetAutoOrient(true);

		m_pPartitionedSpace->AddAgent(m_Agents[i]);
	}
}

Flock::~Flock()
{
	SAFE_DELETE(m_pSeparationBehavior);
	SAFE_DELETE(m_pCohesionBehavior);
	SAFE_DELETE(m_pVelMatchBehavior);
	SAFE_DELETE(m_pSeekBehavior);
	SAFE_DELETE(m_pWanderBehavior);

	SAFE_DELETE(m_pEvadeBehavior);

	SAFE_DELETE(m_pBlendedSteering);
	SAFE_DELETE(m_pPrioritySteering);

	SAFE_DELETE(m_pPartitionedSpace);

	for(auto pAgent: m_Agents)
	{
		SAFE_DELETE(pAgent);
	}
	m_Agents.clear();
}

void Flock::Update(float deltaT)
{
	TargetData evadeTarget;
	evadeTarget.LinearVelocity = m_pAgentToEvade->GetLinearVelocity();
	evadeTarget.Position = m_pAgentToEvade->GetPosition();
	m_pEvadeBehavior->SetTarget(evadeTarget);

	for (int i{}; i < m_FlockSize; ++i)
	{
		// Update agent
		RegisterNeighbors(m_Agents[i]);
		m_Agents[i]->Update(deltaT);
		if (m_CanTrimWorld)
		{
			m_Agents[i]->TrimToWorld(m_WorldSize);
		}

		// Update partitioning
		if (m_UseSpacePartitioning)
		{
			m_pPartitionedSpace->UpdateAgentCell(m_Agents[i], m_OldPositions[i]);
		}

		// Save old position before update
		if (m_UseSpacePartitioning)
		{
			m_OldPositions[i] = m_Agents[i]->GetPosition();
		}
	}
}

void Flock::Render(float deltaT)
{
	//RenderAgents
	for (int i{}; i < m_FlockSize; ++i)
	{
		m_Agents[i]->Render(deltaT);
	}

	//Debug Rendering	
	auto agent = m_Agents[0];
	if (agent->CanRenderBehavior() != m_CanDebugRender)
	{
		agent->SetRenderBehavior(m_CanDebugRender);
	}

	if (m_CanDebugRender)
	{
		DEBUGRENDERER2D->DrawCircle(agent->GetPosition(), m_NeighborhoodRadius, { 1,1,1 }, 0.4f);

		// Render SpacePartitioning
		if (m_UseSpacePartitioning)
		{
			m_pPartitionedSpace->RenderCells();
		}
	}
}

void Flock::UpdateAndRenderUI()
{
	//Setup
	int menuWidth = 235;
	int const width = DEBUGRENDERER2D->GetActiveCamera()->GetWidth();
	int const height = DEBUGRENDERER2D->GetActiveCamera()->GetHeight();
	bool windowActive = true;
	ImGui::SetNextWindowPos(ImVec2((float)width - menuWidth - 10, 10));
	ImGui::SetNextWindowSize(ImVec2((float)menuWidth, (float)height - 20));
	ImGui::Begin("Gameplay Programming", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
	ImGui::PushAllowKeyboardFocus(false);

	//Elements
	ImGui::Text("CONTROLS");
	ImGui::Indent();
	ImGui::Text("LMB: place target");
	ImGui::Text("RMB: move cam.");
	ImGui::Text("Scrollwheel: zoom cam.");
	ImGui::Unindent();

	ImGui::Spacing();
	ImGui::Separator();
	ImGui::Spacing();
	ImGui::Spacing();

	ImGui::Text("STATS");
	ImGui::Indent();
	ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
	ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
	ImGui::Unindent();

	ImGui::Spacing();
	ImGui::Spacing();
	ImGui::Checkbox("SpacePartitioning", &m_UseSpacePartitioning);

	ImGui::Spacing();
	ImGui::Spacing();
	if (m_CanTrimWorld)
	{
		ImGui::SliderFloat("Trim Size", &m_WorldSize, 0.f, 500.f, "%1.");
	}

	ImGui::Spacing();
	ImGui::Separator();
	ImGui::Spacing();

	ImGui::Text("Flocking");
	ImGui::Spacing();

	ImGui::Checkbox("Debug Rendering", &m_CanDebugRender);

	ImGui::SliderFloat("Separation", &m_pBlendedSteering->GetWeightedBehaviorsRef()[0].weight, 0.f, 1.f, "%.2");
	ImGui::SliderFloat("Cohesion", &m_pBlendedSteering->GetWeightedBehaviorsRef()[1].weight, 0.f, 1.f, "%.2");
	ImGui::SliderFloat("VelocityMatch", &m_pBlendedSteering->GetWeightedBehaviorsRef()[2].weight, 0.f, 1.f, "%.2");
	ImGui::SliderFloat("Seek", &m_pBlendedSteering->GetWeightedBehaviorsRef()[3].weight, 0.f, 1.f, "%.2");
	ImGui::SliderFloat("Wander", &m_pBlendedSteering->GetWeightedBehaviorsRef()[4].weight, 0.f, 1.f, "%.2");

	//End
	ImGui::PopAllowKeyboardFocus();
	ImGui::End();
}

void Flock::RegisterNeighbors(SteeringAgent* pAgent)
{
	m_NrOfNeighbors = 0;

	if (m_UseSpacePartitioning)	// Use space partitioning
	{
		m_pPartitionedSpace->RegisterNeighbors(pAgent, m_NeighborhoodRadius);
		m_Neighbors = m_pPartitionedSpace->GetNeighbors();
		m_NrOfNeighbors = m_pPartitionedSpace->GetNrOfNeighbors();

	}
	else // Don't use space partitioning
	{
		for (int i{}; i < m_FlockSize; ++i)
		{
			if (m_Agents[i] == pAgent)
			{
				continue;
			}
			float distanceToAgent = Elite::Distance(pAgent->GetPosition(), m_Agents[i]->GetPosition());
			if (distanceToAgent <= m_NeighborhoodRadius)
			{
				m_Neighbors[m_NrOfNeighbors] = m_Agents[i];
				++m_NrOfNeighbors;
			}
		}
	}

	//Debug Rendering: render neighborhood
	if (pAgent->CanRenderBehavior())
	{
		for (int i = 0; i < m_NrOfNeighbors; ++i)
		{
			const auto& neighbor{ m_Neighbors[i] };
			DEBUGRENDERER2D->DrawSolidCircle(neighbor->GetPosition(), neighbor->GetRadius(), { 0,0 }, { 0, 1, 0 }, 0.3f);
		}
	}
}

Elite::Vector2 Flock::GetAverageNeighborPos() const
{
	Elite::Vector2 averagePos{};
	if (m_NrOfNeighbors > 0)
	{
		for (int i{}; i < m_NrOfNeighbors; ++i)
		{
			averagePos += m_Neighbors[i]->GetPosition();
		}
		averagePos /= static_cast<float>(m_NrOfNeighbors);
	}
	
	return averagePos;
}

Elite::Vector2 Flock::GetAverageNeighborVelocity() const
{
	Elite::Vector2 averageVel{};
	if (m_NrOfNeighbors > 0)
	{
		for (int i{}; i < m_NrOfNeighbors; ++i)
		{
			averageVel += m_Neighbors[i]->GetLinearVelocity();
		}

		averageVel /= static_cast<float>(m_NrOfNeighbors);
	}

	return averageVel;
}

void Flock::SetSeekTarget(TargetData target)
{
	m_pSeekBehavior->SetTarget(target);
}

float* Flock::GetWeight(ISteeringBehavior* pBehavior) 
{
	if (m_pBlendedSteering)
	{
		auto& weightedBehaviors = m_pBlendedSteering->GetWeightedBehaviorsRef();
		auto it = find_if(weightedBehaviors.begin(),
			weightedBehaviors.end(),
			[pBehavior](BlendedSteering::WeightedBehavior el)
			{
				return el.pBehavior == pBehavior;
			}
		);

		if(it!= weightedBehaviors.end())
			return &it->weight;
	}

	return nullptr;
}
