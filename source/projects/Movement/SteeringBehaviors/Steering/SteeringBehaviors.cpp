//Precompiled Header [ALWAYS ON TOP IN CPP]
#include "stdafx.h"

//Includes
#include "SteeringBehaviors.h"
#include "../SteeringAgent.h"
#include "../Obstacle.h"
#include "framework\EliteMath\EMatrix2x3.h"

//SEEK
//****
SteeringOutput Seek::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};

	steering.LinearVelocity = m_Target.Position - pAgent->GetPosition();
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	if (pAgent->CanRenderBehavior()) {
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5.f, { 0, 1, 0 });
	}

	return steering;
}

SteeringOutput Flee::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};

	const Elite::Vector2 toTarget = pAgent->GetPosition() - (m_Target).Position;
	const float distanceSquared = toTarget.MagnitudeSquared();

	if (distanceSquared > m_FleeRadius * m_FleeRadius)
	{
		steering.IsValid = false;
		return steering;
	}

	steering.LinearVelocity = -(m_Target.Position - pAgent->GetPosition());
	steering.LinearVelocity.Normalize();
	steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5, { 0, 1, 0 });
	}

	return steering;
}

SteeringOutput Arrive::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering{};

	const float distanceToTarget{ (m_Target.Position - pAgent->GetPosition()).Magnitude()};

	steering.LinearVelocity = m_Target.Position - pAgent->GetPosition();
	steering.LinearVelocity.Normalize();

	if (distanceToTarget <= m_SlowRadius)
	{
		steering.LinearVelocity *= pAgent->GetMaxLinearSpeed() * distanceToTarget / m_SlowRadius;

		if (pAgent->CanRenderBehavior())
		{
			DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5, { 1, 1, 0 });
		}
	}
	else
	{
		steering.LinearVelocity *= pAgent->GetMaxLinearSpeed();

		if (pAgent->CanRenderBehavior())
		{
			DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), steering.LinearVelocity, 5, { 0, 1, 0 });
		}
	}

	return steering;
}

SteeringOutput Face::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	// Disable this to avoid issues
	pAgent->SetAutoOrient(false);

	SteeringOutput steering{};

	// Get the rotation of the target point based on unit circle.
	const float targetRotation = atan2f(
		m_Target.Position.y - pAgent->GetPosition().y,
		m_Target.Position.x - pAgent->GetPosition().x
	);

	// Vector target to agent
	const Elite::Vector2 targetAgent{m_Target.Position - pAgent->GetPosition()};

	// Forward vector agent
	const Elite::Vector2 pAgentForward{
		cosf(pAgent->GetRotation()),
		sinf(pAgent->GetRotation())
	};

	// Determine what side to rotate through cross product
	const float direction = Elite::Cross(pAgentForward, targetAgent);

	// Stop rotating if within error margin
	if (std::abs(pAgent->GetRotation() - targetRotation) < 0.2f)
	{
		steering.AngularVelocity = 0.f;
	}
	else
	{
		// Steer correct side
		steering.AngularVelocity = Elite::Clamp(direction, -1.f, 1.f) * pAgent->GetMaxAngularSpeed();
	}

	std::cout << Elite::ToDegrees(pAgent->GetRotation()) << std::endl;

	// Debug draw calls.
	if (pAgent->CanRenderBehavior())
	{
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), pAgentForward.GetNormalized(), 10, { 0,0,1 });
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), targetAgent.GetNormalized(), 10, { 0,1,1 });
	}

	return steering;
}

SteeringOutput Wander::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	SteeringOutput steering = {};

	//Circle in front of the target
	const Elite::Vector2 circleCenter{ pAgent->GetPosition() + pAgent->GetDirection() * m_OffsetDistance };

	//Get a random angle between -45 and +45 degrees
	const float randomAngleRads = (fmod(rand(), 2 * m_MaxAngleChange)) - m_MaxAngleChange;
	m_WanderAngle += randomAngleRads;
	const Elite::Vector2 wanderAngleVector{ cos(m_WanderAngle), sin(m_WanderAngle) };
	//Get target
	Elite::Vector2 target = m_Radius * wanderAngleVector;
	target += circleCenter;

	m_Target = target;

	if (pAgent->CanRenderBehavior()) {
		DEBUGRENDERER2D->DrawCircle(circleCenter, m_Radius, { 0, 0, 1 }, 0.f);
		DEBUGRENDERER2D->DrawSegment(pAgent->GetPosition(), circleCenter, { 1, 0, 1 });
		DEBUGRENDERER2D->DrawDirection(pAgent->GetPosition(), wanderAngleVector, 5.f, { 0, 1, 0 });
		DEBUGRENDERER2D->DrawPoint(target, 5.0f, { 0, 1, 0 });
	}

	return Seek::CalculateSteering(deltaT, pAgent);
}

SteeringOutput Persuit::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	//check where the actor will be in the future
	//factor based on how far they are from each other
	const float futurePositionFactor{ 4.f * Elite::Distance(pAgent->GetPosition(), m_Target.Position) / pAgent->GetMaxLinearSpeed() };
	const Elite::Vector2 futurePosition{ m_Target.Position + (futurePositionFactor * m_Target.LinearVelocity.GetNormalized()) };

	if (pAgent->CanRenderBehavior()) {
		DEBUGRENDERER2D->DrawSegment(pAgent->GetPosition(), futurePosition, { 0, 0, 1 });
		DEBUGRENDERER2D->DrawSegment(m_Target.Position, futurePosition, { 0, 0, 1 });
	}

	m_Target = futurePosition;

	return Seek::CalculateSteering(deltaT, pAgent);
}

SteeringOutput Evade::CalculateSteering(float deltaT, SteeringAgent* pAgent)
{
	if (Elite::DistanceSquared(m_Target.Position, pAgent->GetPosition()) <= m_EvadeRadius * m_EvadeRadius)
	{
		const float futurePositionFactor{ Elite::Distance(pAgent->GetPosition(), m_Target.Position) / pAgent->GetMaxLinearSpeed() };
		const Elite::Vector2 futurePosition{ m_Target.Position + futurePositionFactor * m_Target.LinearVelocity.GetNormalized() };

		if (pAgent->CanRenderBehavior())
		{
			DEBUGRENDERER2D->DrawSegment(pAgent->GetPosition(), futurePosition, { 0.f, 0.f, 1.f }, 0);
			DEBUGRENDERER2D->DrawSegment(m_Target.Position, futurePosition, { 1, 0, 0 });
			DEBUGRENDERER2D->DrawSolidCircle(pAgent->GetPosition(), 2.f, { 1,1 }, { 1,0,0 });
		}

		m_Target = futurePosition;

		return Flee::CalculateSteering(deltaT, pAgent);
	}

	SteeringOutput steer = {};
	steer.IsValid = false;

	return steer;
}
