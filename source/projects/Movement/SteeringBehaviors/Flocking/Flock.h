#pragma once
#include "../SteeringHelpers.h"
#include "FlockingSteeringBehaviors.h"

class ISteeringBehavior;
class SteeringAgent;
class BlendedSteering;
class PrioritySteering;
class Pursuit;
class CellSpace;

class Flock final
{
public:
	Flock(
		int flockSize = 50, 
		float worldSize = 100.f, 
		SteeringAgent* pAgentToEvade = nullptr, 
		bool trimWorld = false);

	~Flock();

	void Update(float deltaT);
	void UpdateAndRenderUI() ;
	void Render(float deltaT);

	void RegisterNeighbors(SteeringAgent* pAgent);
	int GetNrOfNeighbors() const { return m_NrOfNeighbors; }
	const std::vector<SteeringAgent*>& GetNeighbors() const { return m_Neighbors; }

	Elite::Vector2 GetAverageNeighborPos() const;
	Elite::Vector2 GetAverageNeighborVelocity() const;

	float GetNeighborhoodRadius() { return m_NeighborhoodRadius; }

	void SetTarget_Seek(TargetData target);
	void SetWorldTrimSize(float size) { m_WorldSize = size; }


private:
	CellSpace* m_CellSpace{};
	std::vector<Elite::Vector2> m_OldPositions{};

	// Data members
	int m_FlockSize = 0;
	std::vector<SteeringAgent*> m_Agents;
	std::vector<SteeringAgent*> m_Neighbors;

	bool m_TrimWorld = false;
	bool m_CanRenderDebug = false;
	bool m_UsingSpatialPartitioning = true;

	float m_WorldSize = 0.f;

	float m_NeighborhoodRadius = 5.f;
	float m_RunRadius = 10.f;
	float m_TargetSwitchCooldown = 10.f;
	float m_CurrentTime = 0.f;

	int m_NrOfNeighbors = 0;

	// Seek target
	Elite::Vector2 m_SeekTargetPosition{};

	// enemy agent
	SteeringAgent* m_pAgentToEvade = nullptr;
	SteeringAgent* m_pAgentPrey = nullptr;
	
	//Steering Behaviors
	Seek* m_pSeekBehavior = nullptr;
	Persuit* m_pPersuitPreyBehavior = nullptr;
	Separation* m_pSeparationBehavior = nullptr;
	Cohesion* m_pCohesionBehavior = nullptr;
	Alignment* m_pAlignmentBehavior = nullptr;
	Wander* m_pWanderBehavior = nullptr;

	Evade* m_pEvadeBehavior = nullptr;
	BlendedSteering* m_pBlendedSteering = nullptr;
	PrioritySteering* m_pPrioritySteering = nullptr;

	float* GetWeight(ISteeringBehavior* pBehaviour);

private:
	Flock(const Flock& other);
	Flock& operator=(const Flock& other);
};