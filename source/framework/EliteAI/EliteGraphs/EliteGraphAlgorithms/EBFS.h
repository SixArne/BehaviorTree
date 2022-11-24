#pragma once

#define NOT !

namespace Elite 
{
	template <class T_NodeType, class T_ConnectionType>
	class BFS
	{
	public:
		BFS(IGraph<T_NodeType, T_ConnectionType>* pGraph);

		std::vector<T_NodeType*> FindPath(T_NodeType* pStartNode, T_NodeType* pDestinationNode);
	private:
		IGraph<T_NodeType, T_ConnectionType>* m_pGraph;
	};

	template <class T_NodeType, class T_ConnectionType>
	BFS<T_NodeType, T_ConnectionType>::BFS(IGraph<T_NodeType, T_ConnectionType>* pGraph)
		: m_pGraph(pGraph)
	{

	}

	template <class T_NodeType, class T_ConnectionType>
	std::vector<T_NodeType*> BFS<T_NodeType, T_ConnectionType>::FindPath(T_NodeType* pStartNode, T_NodeType* pDestinationNode)
	{
		std::queue<T_NodeType*> openList{};
		std::map<T_NodeType*, T_NodeType*> closedList{}; // convert this to unordered if possible

		openList.push(pStartNode);

		while (NOT openList.empty())
		{
			T_NodeType* pCurrentNode{ openList.front() };
			openList.pop();

			if (pCurrentNode == pDestinationNode)
				break;

			auto& connections = m_pGraph->GetNodeConnections(pCurrentNode);
			for (auto& conn : connections)
			{
				T_NodeType* pNextNode{ m_pGraph->GetNode(conn->GetTo()) };

				// prevents loop triggers when its not in there yet
				if (closedList.find(pNextNode) == closedList.end())
				{
					openList.push(pNextNode);
					closedList[pNextNode] = pCurrentNode;
				}
			}
		}

		if (closedList.find(pDestinationNode) == closedList.end())
		{
			return std::vector<T_NodeType*>();
		}

		// backtracking
		std::vector<T_NodeType*> path{};

		T_NodeType* pCurrentNode{pDestinationNode};
		while (pCurrentNode != pStartNode)
		{
			path.push_back(pCurrentNode);
			pCurrentNode = closedList[pCurrentNode];
		}

		path.push_back(pStartNode);
		std::reverse(path.begin(), path.end());

		return path;
	}
}
