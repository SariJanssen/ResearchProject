#pragma once

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
		std::queue<T_NodeType*> openList; // Frontier - Expanding Edge
		std::map<T_NodeType*, T_NodeType*> closedList; // Already checked nodes

		openList.push(pStartNode); // Kickstarting the loop

		while (!openList.empty())
		{
			T_NodeType* pCurrentNode = openList.front();
			openList.pop();

			if (pCurrentNode == pDestinationNode)
			{
				break;
			}

			// Loop over all connections of the current node
			for (auto con : m_pGraph->GetNodeConnections(pCurrentNode))
			{
				T_NodeType* pNextNode = m_pGraph->GetNode(con->GetTo());
				if (closedList.find(pNextNode) == closedList.end()) // next node is not part of closed list
				{
					openList.push(pNextNode);
					closedList[pNextNode] = pCurrentNode;
				}
			}
		}

		// We found the goal node, track back from goal node to start node to create path
		vector<T_NodeType*> path;
		T_NodeType* pNode = pDestinationNode; // Start tracking back from end node
		while (pNode != pStartNode)
		{
			path.push_back(pNode);
			pNode = closedList[pNode];
		}

		path.push_back(pStartNode); // Start node is not part of path, manually add it to the path

		std::reverse(path.begin(), path.end());

		return path;
	}
}

