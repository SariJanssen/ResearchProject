#pragma once
#include <stack>

namespace Elite
{
	enum class Eulerianity
	{
		notEulerian,
		semiEulerian,
		eulerian,
	};

	std::string EulerianityToString(Eulerianity e)
	{
		switch (e)
		{
		case Elite::Eulerianity::notEulerian:
			return "not eulerian";
			break;
		case Elite::Eulerianity::semiEulerian:
			return "semi eulerian";
			break;
		case Elite::Eulerianity::eulerian:
			return "eulerian";
			break;
		default:
			return "error";
			break;
		}
	}

	template <class T_NodeType, class T_ConnectionType>
	class EulerianPath
	{
	public:

		EulerianPath(IGraph<T_NodeType, T_ConnectionType>* pGraph);

		Eulerianity IsEulerian() const;
		vector<T_NodeType*> FindPath(Eulerianity& eulerianity) const;

	private:
		void VisitAllNodesDFS(int startIdx, vector<bool>& visited) const;
		bool IsConnected() const;

		IGraph<T_NodeType, T_ConnectionType>* m_pGraph;
	};

	template<class T_NodeType, class T_ConnectionType>
	inline EulerianPath<T_NodeType, T_ConnectionType>::EulerianPath(IGraph<T_NodeType, T_ConnectionType>* pGraph)
		: m_pGraph(pGraph)
	{
	}

	template<class T_NodeType, class T_ConnectionType>
	inline Eulerianity EulerianPath<T_NodeType, T_ConnectionType>::IsEulerian() const
	{
		// If the graph is not connected, there can be no Eulerian Trail
		if (IsConnected() == false)
		{
			return Eulerianity::notEulerian;
		}

		// Count nodes with odd degree 
		auto activeNodes = m_pGraph->GetAllActiveNodes();
		int oddCount = 0;
		for (auto node : activeNodes)
		{
			auto connections = m_pGraph->GetNodeConnections(node);
			if (connections.size() & 1) // check if uneven
			{
				++oddCount;
			}
		}

		// A connected graph with more than 2 nodes with an odd degree (an odd amount of connections) is not Eulerian
		if (oddCount > 2)
		{
			return Eulerianity::notEulerian;
		}

		// A connected graph with exactly 2 nodes with an odd degree is Semi-Eulerian (unless there are only 2 nodes)
		// An Euler trail can be made, but only starting and ending in these 2 nodes
		else if (oddCount == 2)
		{
			return Eulerianity::semiEulerian;
		}

		// A connected graph with no odd nodes is Eulerian
		else
		{
			return Eulerianity::eulerian;
		}
	}

	template<class T_NodeType, class T_ConnectionType>
	inline vector<T_NodeType*> EulerianPath<T_NodeType, T_ConnectionType>::FindPath(Eulerianity& eulerianity) const
	{
		//Start with an empty stack and an empty path
		
		auto graphCopy = m_pGraph->Clone();
		auto path = vector<T_NodeType*>();

		stack<int> myStack{};
		int nrOfNodes = graphCopy->GetNrOfNodes();

		T_NodeType* pCurrentNode{ nullptr };

		auto& activeNodes{ graphCopy->GetAllActiveNodes() };

		switch (eulerianity)
		{
		case Elite::Eulerianity::notEulerian: // no eulerian circuit exists
			return path;
		case Elite::Eulerianity::semiEulerian: // if there are exactly 2 vertices having an odd degree, choose one of them
		{
			for (T_NodeType* node : activeNodes)
			{
				auto connections = m_pGraph->GetNodeConnections(node);
				if (connections.size() & 1) // check if uneven
				{
					pCurrentNode = node;
					break;
				}
			}
		}
			break;
		case Elite::Eulerianity::eulerian: // all vertices have even degree, choose any of them
			pCurrentNode = activeNodes[0];
			break;
		default:
			return path;
		}

		if (pCurrentNode == nullptr)
		{
			return path;
		}

		do
		{
			if (graphCopy->GetNodeConnections(pCurrentNode).size() == 0) // current node has no neighbors
			{
				path.push_back(m_pGraph->GetNode(pCurrentNode->GetIndex()));
				pCurrentNode = m_pGraph->GetNode(myStack.top());
				myStack.pop();
			}
			else // current node does have neigbor
			{
				myStack.push(pCurrentNode->GetIndex());
				int indexNext = graphCopy->GetNodeConnections(pCurrentNode->GetIndex()).front()->GetTo(); // get index of any next neighboring node
				T_NodeType* pNextNode = graphCopy->GetNode(indexNext);
				graphCopy->RemoveConnection(pCurrentNode->GetIndex(), pNextNode->GetIndex());
				pCurrentNode = pNextNode;
			}
		} while (graphCopy->GetNodeConnections(pCurrentNode).size() != 0 || !myStack.empty());
		// continue while there ARE connections OR the stack IS NOT empty

		path.push_back(m_pGraph->GetNode(pCurrentNode->GetIndex()));
		std::reverse(path.begin(), path.end()); // path obtained is in reverse order
		
		return path;
	}

	template<class T_NodeType, class T_ConnectionType>
	inline void EulerianPath<T_NodeType, T_ConnectionType>::VisitAllNodesDFS(int startIdx, vector<bool>& visited) const
	{
		// mark the visited node
		visited[startIdx] = true;

		// recursively visit any valid connected nodes that were not visited before
		for (T_ConnectionType* pConnection : m_pGraph->GetNodeConnections(startIdx))
		{
			if (visited[pConnection->GetTo()] == false)
			{
				VisitAllNodesDFS(pConnection->GetTo(), visited);
			}
		}

	}

	template<class T_NodeType, class T_ConnectionType>
	inline bool EulerianPath<T_NodeType, T_ConnectionType>::IsConnected() const
	{
		auto activeNodes = m_pGraph->GetAllActiveNodes();
		vector<bool> visited(m_pGraph->GetNrOfNodes(), false); // niet active nodes want we gaan dit ook gebruiken voor de index van de nodes

		// find a valid starting node that has connections
		int connectedIdx = invalid_node_index; // == -1
		for (auto node : activeNodes)
		{
			auto connections = m_pGraph->GetNodeConnections(node);
			if (connections.size() != 0)
			{
				connectedIdx = node->GetIndex(); // autocomplete werkt niet zo goed door template
				break;
			}
		}
		
		// if no valid node could be found, return false
		if (connectedIdx == invalid_node_index)
		{
			return false;
		}

		// start a depth-first-search traversal from the node that has at least one connection
		VisitAllNodesDFS(connectedIdx, visited);

		// if a node was never visited, this graph is not connected
		for (auto node : activeNodes)
		{
			if (visited[node->GetIndex()] == false)
			{
				return false;
			}
		}

		return true;
	}

}