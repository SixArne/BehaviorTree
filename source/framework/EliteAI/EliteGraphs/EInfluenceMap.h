#pragma once
#include "EIGraph.h"
#include "EGraphNodeTypes.h"
#include "EGraphConnectionTypes.h"

namespace Elite
{
	template<class T_GraphType>
	class InfluenceMap final : public T_GraphType
	{
	public:
		InfluenceMap(bool isDirectional): T_GraphType(isDirectional) {}
		void InitializeBuffer() { m_InfluenceDoubleBuffer = std::vector<float>(m_Nodes.size()); }
		void PropagateInfluence(float deltaTime);

		void SetInfluenceAtPosition(Elite::Vector2 pos, float influence);

		void Render() const {}
		void SetNodeColorsBasedOnInfluence();

		float GetMomentum() const { return m_Momentum; }
		void SetMomentum(float momentum) { m_Momentum = momentum; }

		float GetDecay() const { return m_Decay; }
		void SetDecay(float decay) { m_Decay = decay; }

		float GetPropagationInterval() const { return m_PropagationInterval; }
		void SetPropagationInterval(float propagationInterval) { m_PropagationInterval = propagationInterval; }

	protected:
		virtual void OnGraphModified(bool nrOfNodesChanged, bool nrOfConnectionsChanged) override;

	private:
		Elite::Color m_NegativeColor{ 1.f, 0.2f, 0.f};
		Elite::Color m_NeutralColor{ 0.f, 0.f, 0.f };
		Elite::Color m_PositiveColor{ 0.f, 0.2f, 1.f};

		float m_MaxAbsInfluence = 100.f;

		float m_Momentum = 0.8f; // a higher momentum means a higher tendency to retain the current influence
		float m_Decay = 0.1f; // determines the decay in influence over distance

		float m_PropagationInterval = .05f; //in Seconds
		float m_TimeSinceLastPropagation = 0.0f;

		std::vector<float> m_InfluenceDoubleBuffer;
	};

	template <class T_GraphType>
	void InfluenceMap<T_GraphType>::PropagateInfluence(float deltaTime)
	{
		/// If an event were to happen we would trigger a node and set the influence of it
		/// this will then be written to the connections, the next time something happens 
		/// this function will properly blend everything and then the AI
		/// can head towards the nodes or get away from them.

		m_TimeSinceLastPropagation += deltaTime;

		if (m_TimeSinceLastPropagation >= m_PropagationInterval)
		{
			for (const auto& node : m_Nodes) 
			{
				auto connections = m_Connections.at(node->GetIndex());
				float highestInfluence{0};

				for (const auto& conn : connections)
				{
					InfluenceNode* neighborNode{ m_Nodes[conn->GetTo()] };
					float newInfluence = neighborNode->GetInfluence() * expf(-conn->GetCost() * m_Decay);

					if (std::abs(newInfluence) > std::abs(highestInfluence))
					{
						highestInfluence = newInfluence;
					}
				}


				float influence = Lerp(node->GetInfluence(), highestInfluence, m_Momentum);
				influence = Elite::Clamp(influence, -m_MaxAbsInfluence, m_MaxAbsInfluence);

				m_InfluenceDoubleBuffer[node->GetIndex()] = influence;
			}

			for (auto& node : m_Nodes)
			{
				node->SetInfluence(m_InfluenceDoubleBuffer[node->GetIndex()]);
			}

			m_TimeSinceLastPropagation = 0.f;
		}
	}

	template <class T_GraphType>
	inline void InfluenceMap<T_GraphType>::SetInfluenceAtPosition(Elite::Vector2 pos, float influence)
	{
		auto idx = GetNodeIdxAtWorldPos(pos);
		if (IsNodeValid(idx))
			GetNode(idx)->SetInfluence(influence);
	}

	template<class T_GraphType>
	inline void InfluenceMap<T_GraphType>::SetNodeColorsBasedOnInfluence()
	{
		const float half = .5f;

		for (auto& pNode : m_Nodes)
		{
			Color nodeColor{};
			float influence = pNode->GetInfluence();
			float relativeInfluence = abs(influence) / m_MaxAbsInfluence;

			if (influence < 0)
			{
				nodeColor = Elite::Color{
				Lerp(m_NeutralColor.r, m_NegativeColor.r, relativeInfluence),
				Lerp(m_NeutralColor.g, m_NegativeColor.g, relativeInfluence),
				Lerp(m_NeutralColor.b, m_NegativeColor.b, relativeInfluence)
				};
			}
			else
			{
				nodeColor = Elite::Color{
				Lerp(m_NeutralColor.r, m_PositiveColor.r, relativeInfluence),
				Lerp(m_NeutralColor.g, m_PositiveColor.g, relativeInfluence),
				Lerp(m_NeutralColor.b, m_PositiveColor.b, relativeInfluence)
				};
			}

			pNode->SetColor(nodeColor);
		}
	}

	template<class T_GraphType>
	inline void InfluenceMap<T_GraphType>::OnGraphModified(bool nrOfNodesChanged, bool nrOfConnectionsChanged)
	{
		InitializeBuffer();
	}
}