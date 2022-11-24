#include "stdafx.h"
#include "Flock.h"

#include "../SteeringAgent.h"
#include "../Steering/SteeringBehaviors.h"
#include "../CombinedSteering/CombinedSteeringBehaviors.h"
#include "projects/Movement/SteeringBehaviors/SpacePartitioning/SpacePartitioning.h"

using namespace Elite;

//Constructor & Destructor
Flock::Flock(
	int flockSize /*= 50*/, 
	float worldSize /*= 100.f*/, 
	SteeringAgent* pAgentToEvade /*= nullptr*/, 
	bool trimWorld /*= false*/)

	: m_WorldSize{ worldSize }
	, m_FlockSize{ flockSize }
	, m_TrimWorld { trimWorld }
	, m_pAgentToEvade{pAgentToEvade}
	, m_NeighborhoodRadius{ 15 }
	, m_NrOfNeighbors{0}
{
	m_CellSpace = new CellSpace(worldSize, worldSize, 25, 25, flockSize);

	m_Agents.resize(m_FlockSize);
	m_pEvadeBehavior = new Evade();
	m_pWanderBehavior = new Wander();
	m_pSeekBehavior = new Seek();
	m_pPersuitPreyBehavior = new Persuit();
	m_pAlignmentBehavior = new Alignment(this);
	m_pCohesionBehavior = new Cohesion(this);
	m_pSeparationBehavior = new Separation(this);

	m_pBlendedSteering = new BlendedSteering({
		{m_pAlignmentBehavior, 0.5f },
		{m_pCohesionBehavior, 0.5f },
		{m_pSeparationBehavior, 0.5f },
		{m_pWanderBehavior, 0.5f },
		{m_pSeekBehavior, 0.0f }
		});

	m_pPrioritySteering = new PrioritySteering({ m_pEvadeBehavior, m_pBlendedSteering });

	// Initialize the agents.
	m_Agents.resize(flockSize);

#pragma region agent
	m_pAgentToEvade = new SteeringAgent();
	m_pAgentToEvade->SetSteeringBehavior(m_pPersuitPreyBehavior);
	m_pAgentToEvade->SetMaxLinearSpeed(17.f);
	m_pAgentToEvade->SetMass(0.7f);
	m_pAgentToEvade->SetAutoOrient(true);
	m_pAgentToEvade->SetBodyColor(Color{ 1.0f, 0.f, 0.f });

	Vector2 position = {
			static_cast<float>(rand() % static_cast<int>(m_WorldSize)),
			static_cast<float>(rand() % static_cast<int>(m_WorldSize))
	};
	m_pAgentToEvade->SetPosition(position);
#pragma endregion

#pragma region flock
	for (int agentIndex{ 0 }; agentIndex < flockSize; agentIndex++)
	{
		m_Agents[agentIndex] = new SteeringAgent();
		m_Agents[agentIndex]->SetSteeringBehavior(m_pPrioritySteering);
		m_Agents[agentIndex]->SetMaxLinearSpeed(15.f);
		m_Agents[agentIndex]->SetAutoOrient(true);
		m_Agents[agentIndex]->SetMass(0.3f);

		Vector2 position = {
			static_cast<float>(rand() % static_cast<int>(m_WorldSize)),
			static_cast<float>(rand() % static_cast<int>(m_WorldSize))
		};
		m_Agents[agentIndex]->SetPosition(position);

		//todo: only do this when checkbox gets checked
		// Add agent to the cellspace
		m_OldPositions.push_back(position);
		m_CellSpace->AddAgent(m_Agents[agentIndex]);
	}

	// Copy positions for m_Neighbors.
	m_Neighbors.resize(flockSize);
	m_Neighbors = m_Agents;
#pragma endregion

	const TargetData& targetData = {
		m_pAgentToEvade->GetPosition(),
		m_pAgentToEvade->GetRotation(),
		m_pAgentToEvade->GetLinearVelocity(),
		m_pAgentToEvade->GetAngularVelocity()
	};

	// Set the target data for the agents to know the chaser's position.
	m_pEvadeBehavior->SetTarget(targetData);

	// set prey
	m_pAgentPrey = m_Agents[0];
	m_pAgentToEvade->GetSteeringBehavior()->SetTarget({ m_pAgentPrey->GetPosition(), m_pAgentPrey->GetRotation(), m_pAgentPrey->GetLinearVelocity(), m_pAgentPrey->GetAngularVelocity() });
	m_Agents[0]->SetBodyColor({ 0,0,1 });
}

Flock::~Flock()
{
	SAFE_DELETE(m_pBlendedSteering);
	SAFE_DELETE(m_pPrioritySteering);
	SAFE_DELETE(m_pEvadeBehavior);
	SAFE_DELETE(m_pSeekBehavior);
	SAFE_DELETE(m_pWanderBehavior);
	SAFE_DELETE(m_pCohesionBehavior);
	SAFE_DELETE(m_pSeparationBehavior);
	SAFE_DELETE(m_pAlignmentBehavior);
	SAFE_DELETE(m_pPersuitPreyBehavior);

	SAFE_DELETE(m_pAgentToEvade);

	for(auto pAgent: m_Agents)
	{
		SAFE_DELETE(pAgent);
	}
	m_Agents.clear();

	SAFE_DELETE(m_CellSpace);
}

void Flock::Update(float deltaT)
{
	// Set seek target to mouse
	m_pBlendedSteering->GetWeightedBehaviorsRef()[4].pBehavior->SetTarget(m_SeekTargetPosition);

	const TargetData& targetData = {
		m_pAgentToEvade->GetPosition(),
		m_pAgentToEvade->GetRotation(),
		m_pAgentToEvade->GetLinearVelocity(),
		m_pAgentToEvade->GetAngularVelocity(),
	};

	// Make sure the evade behavior tracks the enemy agent.
	m_pEvadeBehavior->SetTarget(targetData);
	m_pEvadeBehavior->SetEvadeRadius(m_RunRadius);
	m_pAgentToEvade->Update(deltaT);

	for (int agentIndex{0}; agentIndex < m_Agents.size(); agentIndex++)
	{
		// Go over all agents and determine its neighbors => save in memory pool.
		if (m_UsingSpatialPartitioning)
		{
			m_CellSpace->UpdateAgentCell(m_Agents[agentIndex], m_OldPositions[agentIndex]);
			m_CellSpace->RegisterNeighbors(m_Agents[agentIndex], m_NeighborhoodRadius);

			// Update old positions
			m_OldPositions[agentIndex] = m_Agents[agentIndex]->GetPosition();

			m_Neighbors = m_CellSpace->GetNeighbors();
			m_NrOfNeighbors = m_CellSpace->GetNrOfNeighbors();
		}
		else
		{
			RegisterNeighbors(m_Agents[agentIndex]);
		}

		// Update the agent before checking old positions
		m_Agents[agentIndex]->Update(deltaT);	
	}

	if (m_TrimWorld)
	{
		for (SteeringAgent* agent : m_Agents)
		{
			agent->TrimToWorld(m_WorldSize);
		}

		m_pAgentToEvade->TrimToWorld(m_WorldSize);
	}

#pragma region enemy switch targets
	m_CurrentTime += deltaT;
	m_pAgentToEvade->GetSteeringBehavior()->SetTarget({
		m_pAgentPrey->GetPosition(),
		m_pAgentPrey->GetRotation(),
		m_pAgentPrey->GetLinearVelocity(),
		m_pAgentPrey->GetAngularVelocity()
	});

	if (m_CurrentTime >= m_TargetSwitchCooldown)
	{
		// Give old color back
		m_pAgentPrey->SetBodyColor({ 1,1,0, });
		m_pAgentPrey->SetRenderBehavior(false);

		m_pAgentPrey = m_Agents[rand() % m_Agents.size()];

		// Give new color to prey
		m_pAgentPrey->SetBodyColor({ 0,0,1, });
		m_pAgentPrey->SetRenderBehavior(true);

		m_CurrentTime = 0;
	}
#pragma endregion
}

void Flock::Render(float deltaT)
{
	for (SteeringAgent* agent : m_Agents)
	{
		if (agent->CanRenderBehavior())
		{
			DEBUGRENDERER2D->DrawCircle(agent->GetPosition(), m_NeighborhoodRadius, { 1,0,0 }, 0);
		}

		//agent->Render(deltaT);
	}

	if (m_CanRenderDebug)
	{
		DEBUGRENDERER2D->DrawCircle(m_pAgentToEvade->GetPosition(), m_RunRadius, { 1,0,0 }, 0);
	}

	m_pAgentToEvade->Render(deltaT);
	m_pAgentToEvade->SetRenderBehavior(m_CanRenderDebug);
	m_pAgentPrey->SetRenderBehavior(m_CanRenderDebug);

	if (m_UsingSpatialPartitioning && m_CanRenderDebug)
		m_CellSpace->RenderCells();
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
	ImGui::Separator();
	ImGui::Spacing();

	ImGui::Checkbox("Debug Rendering", &m_CanRenderDebug);
	ImGui::Checkbox("Trim World", &m_TrimWorld);
	ImGui::Checkbox("Spatial Partitioning", &m_UsingSpatialPartitioning);

	ImGui::Text("Flocking");
	ImGui::Spacing();

	ImGui::Text("Radius");
	ImGui::Spacing();
	ImGui::SliderFloat("Agent radius", &m_NeighborhoodRadius, 0.f, 20.f, "%.2");
	ImGui::SliderFloat("Flee radius", &m_RunRadius, 0.f, 100.f, "%.2");

	ImGui::Text("Behavior Weights");
	ImGui::Spacing();

	ImGui::SliderFloat("Alignment", &m_pBlendedSteering->GetWeightedBehaviorsRef()[0].weight, 0.f, 1.f, "%.2");
	ImGui::SliderFloat("Cohesion", &m_pBlendedSteering->GetWeightedBehaviorsRef()[1].weight, 0.f, 1.f, "%.2");
	ImGui::SliderFloat("Separation", &m_pBlendedSteering->GetWeightedBehaviorsRef()[2].weight, 0.f, 1.f, "%.2");
	ImGui::SliderFloat("Wander", &m_pBlendedSteering->GetWeightedBehaviorsRef()[3].weight, 0.f, 1.f, "%.2");
	ImGui::SliderFloat("Seek", &m_pBlendedSteering->GetWeightedBehaviorsRef()[4].weight, 0.f, 1.f, "%.2");

	//End
	ImGui::PopAllowKeyboardFocus();
	ImGui::End();
	
}

void Flock::RegisterNeighbors(SteeringAgent* pAgent)
{
	// Reset count to 0 for pool;
	m_NrOfNeighbors = 0;

	for (SteeringAgent* agent : m_Agents)
	{
		// Skip current agent.
		if (agent == pAgent) continue;

		if (Elite::DistanceSquared(agent->GetPosition(), pAgent->GetPosition()) <= m_NeighborhoodRadius * m_NeighborhoodRadius)
		{
			// Set found neighbor memory location in pool
			m_Neighbors[m_NrOfNeighbors++] = agent;
		}
	}
}

Elite::Vector2 Flock::GetAverageNeighborPos() const
{
	Vector2 positionsSum{};

	for (int neighborIndex{}; neighborIndex < m_NrOfNeighbors; neighborIndex++)
	{
		positionsSum += m_Neighbors[neighborIndex]->GetPosition();
	}

	return positionsSum / static_cast<float>(m_NrOfNeighbors);
}

Elite::Vector2 Flock::GetAverageNeighborVelocity() const
{
	Vector2 velocitiesSum{};

	for (int neighborIndex{}; neighborIndex < m_NrOfNeighbors; neighborIndex++)
	{
		velocitiesSum += m_Neighbors[neighborIndex]->GetLinearVelocity();
	}

	return velocitiesSum / static_cast<float>(m_NrOfNeighbors);
}

void Flock::SetTarget_Seek(TargetData target)
{
	m_SeekTargetPosition = target.Position;
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
