#include "stdafx.h"
#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"


//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	const Elite::Vector2 averageNeighborPos = m_pFlock->GetAverageNeighborPos();
	m_Target = averageNeighborPos;

	return Seek::CalculateSteering(deltaT, pAgent);
}

//*********************
//SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	Elite::Vector2 targetLocation{};
	for (int i{}; i < m_pFlock->GetNrOfNeighbors(); ++i) {
		Elite::Vector2 neighborPosition{ m_pFlock->GetNeighbors().at(i)->GetPosition() };
		Elite::Vector2 targetToAgent{ pAgent->GetPosition() - neighborPosition };
		targetLocation += (targetToAgent) / (targetToAgent).MagnitudeSquared();
	}
	targetLocation.Normalize();
	m_Target.Position = pAgent->GetPosition() + targetLocation;

	if (pAgent->CanRenderBehavior()) {
		DEBUGRENDERER2D->DrawPoint(m_Target.Position, 2.0f, { 1, 0, 0 });
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), m_Target.Position, 5.f, { 1, 0, 0 });
	}

	return Seek::CalculateSteering(deltaT, pAgent);
}

//*************************
//VELOCITY MATCH (FLOCKING)
SteeringOutput Alignment::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	const Elite::Vector2 averageNeighborVelocity = m_pFlock->GetAverageNeighborVelocity();
	m_Target = pAgent->GetPosition() + averageNeighborVelocity.GetNormalized();

	return Seek::CalculateSteering(deltaT, pAgent);
}