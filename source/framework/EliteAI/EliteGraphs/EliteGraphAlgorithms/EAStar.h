#pragma once
#include "framework/EliteAI/EliteNavigation/ENavigation.h"

namespace Elite
{
	template <class T_NodeType, class T_ConnectionType>
	class AStar
	{
	public:
		AStar(IGraph<T_NodeType, T_ConnectionType>* pGraph, Heuristic hFunction);

		// stores the optimal connection to a node and its total costs related to the start and end node of the path
		struct NodeRecord
		{
			T_NodeType* pNode = nullptr;
			T_ConnectionType* pConnection = nullptr;
			float costSoFar = 0.f; // accumulated g-costs of all the connections leading up to this one
			float estimatedTotalCost = 0.f; // f-cost (= costSoFar + h-cost)

			bool operator==(const NodeRecord& other) const
			{
				return pNode == other.pNode
					&& pConnection == other.pConnection
					&& costSoFar == other.costSoFar
					&& estimatedTotalCost == other.estimatedTotalCost;
			};

			bool operator<(const NodeRecord& other) const
			{
				return estimatedTotalCost < other.estimatedTotalCost;
			};
		};

		std::vector<T_NodeType*> FindPath(T_NodeType* pStartNode, T_NodeType* pDestinationNode);

	private:
		float GetHeuristicCost(T_NodeType* pStartNode, T_NodeType* pEndNode) const;

		IGraph<T_NodeType, T_ConnectionType>* m_pGraph;
		Heuristic m_HeuristicFunction;
	};

	template <class T_NodeType, class T_ConnectionType>
	AStar<T_NodeType, T_ConnectionType>::AStar(IGraph<T_NodeType, T_ConnectionType>* pGraph, Heuristic hFunction)
		: m_pGraph(pGraph)
		, m_HeuristicFunction(hFunction)
	{
	}

	template <class T_NodeType, class T_ConnectionType>
	std::vector<T_NodeType*> AStar<T_NodeType, T_ConnectionType>::FindPath(T_NodeType* pStartNode, T_NodeType* pGoalNode)
	{
#pragma region Old
		std::vector<T_NodeType*> path{};
		std::vector<NodeRecord> openList{};
		std::vector<NodeRecord> closedList{};

		NodeRecord currentRecord{};
		currentRecord.pNode = pStartNode;
		currentRecord.pConnection = nullptr;
		currentRecord.estimatedTotalCost = GetHeuristicCost(currentRecord.pNode, pGoalNode);
		openList.push_back(currentRecord);
	
		while (!openList.empty())
		{
			// Get the node with the smallest distance.
			currentRecord = *std::min_element(openList.begin(), openList.end());

			// If smallest distance node is goal we found it.
			if (currentRecord.pNode == pGoalNode)
			{
				closedList.push_back(currentRecord);
				break;
			}

			auto& connections = m_pGraph->GetNodeConnections(currentRecord.pNode);

			// Calculate total weight of node (f cost)
			for (auto& conn : connections)
			{
				const auto totalCostSoFar = currentRecord.costSoFar + conn->GetCost();
				const auto estimatedTotalCost = totalCostSoFar + GetHeuristicCost(m_pGraph->GetNode(conn->GetTo()), pGoalNode);

				auto findNode = [conn, this](NodeRecord nr) {return nr.pNode == m_pGraph->GetNode(conn->GetTo()); }; // searchLambda
				auto itFoundClosedNode = std::find_if(closedList.begin(), closedList.end(), findNode);

				// Node was found
				if (itFoundClosedNode != closedList.end())
				{
					// dereference iterator to get data.
					NodeRecord closedRecord = *itFoundClosedNode;

					if (closedRecord.costSoFar <= totalCostSoFar)
						continue;
					else
						closedList.erase(std::remove(closedList.begin(), closedList.end(), closedRecord));
				}

				auto& itFoundOpenNode = std::find_if(openList.begin(), openList.end(), findNode);
				if (itFoundOpenNode != openList.end())
				{
					NodeRecord openRecord = *itFoundOpenNode;

					if (openRecord.costSoFar <= totalCostSoFar)
						continue;
					else
						openList.erase(std::remove(openList.begin(), openList.end(), openRecord));
				}

				NodeRecord newNode{};
				newNode.pNode = m_pGraph->GetNode(conn->GetTo());
				newNode.costSoFar = totalCostSoFar;
				newNode.estimatedTotalCost = estimatedTotalCost;
				newNode.pConnection = conn;

				// push new node on top of open nodes
				openList.push_back(newNode);
			}

			openList.erase(std::remove(openList.begin(), openList.end(), currentRecord));
			closedList.push_back(currentRecord);
		}

		if (currentRecord.pNode != pGoalNode)
			return path;

		while (currentRecord.pNode != pStartNode)
		{
			int previousNodeId = currentRecord.pConnection->GetFrom();
			path.push_back(currentRecord.pNode);

			
			auto find = [this, previousNodeId](NodeRecord nr) {return nr.pNode == m_pGraph->GetNode(previousNodeId); };
			auto itClosedRecord = std::find_if(closedList.begin(), closedList.end(), find);

			if (itClosedRecord != closedList.end())
				currentRecord = *itClosedRecord;
		}

		path.push_back(pStartNode);
		std::reverse(path.begin(), path.end());

		return path;
#pragma endregion


	}

	template <class T_NodeType, class T_ConnectionType>
	float Elite::AStar<T_NodeType, T_ConnectionType>::GetHeuristicCost(T_NodeType* pStartNode, T_NodeType* pEndNode) const
	{
		Vector2 toDestination = m_pGraph->GetNodePos(pEndNode) - m_pGraph->GetNodePos(pStartNode);
		return m_HeuristicFunction(abs(toDestination.x), abs(toDestination.y));
	}
}
