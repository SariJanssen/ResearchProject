#include "stdafx.h"
#include "ENavGraph.h"
#include "framework\EliteAI\EliteGraphs\EliteGraphAlgorithms\EAStar.h"

using namespace Elite;

Elite::NavGraph::NavGraph(const Polygon& contourMesh, float playerRadius = 1.0f) :
	Graph2D(false),
	m_pNavMeshPolygon(nullptr)
{
	//Create the navigation mesh (polygon of navigatable area= Contour - Static Shapes)
	m_pNavMeshPolygon = new Polygon(contourMesh); // Create copy on heap

	//Get all shapes from all static rigidbodies with NavigationCollider flag
	auto vShapes = PHYSICSWORLD->GetAllStaticShapesInWorld(PhysicsFlags::NavigationCollider);

	//Store all children
	for (auto shape : vShapes)
	{
		shape.ExpandShape(playerRadius);
		m_pNavMeshPolygon->AddChild(shape);
	}

	//Triangulate
	m_pNavMeshPolygon->Triangulate();

	//Create the actual graph (nodes & connections) from the navigation mesh
	CreateNavigationGraph();
}

Elite::NavGraph::~NavGraph()
{
	delete m_pNavMeshPolygon; 
	m_pNavMeshPolygon = nullptr;
}

int Elite::NavGraph::GetNodeIdxFromLineIdx(int lineIdx) const
{
	auto nodeIt = std::find_if(m_Nodes.begin(), m_Nodes.end(), [lineIdx](const NavGraphNode* n) { return n->GetLineIndex() == lineIdx; });
	if (nodeIt != m_Nodes.end())
	{
		return (*nodeIt)->GetIndex();
	}

	return invalid_node_index;
}

Elite::Polygon* Elite::NavGraph::GetNavMeshPolygon() const
{
	return m_pNavMeshPolygon;
}

void Elite::NavGraph::CreateNavigationGraph()
{
	//1. Go over all the edges of the navigationmesh and create nodes
	//Graph2D graph{false};

	for (Elite::Line* line : m_pNavMeshPolygon->GetLines())
	{
		if (m_pNavMeshPolygon->GetTrianglesFromLineIndex(line->index).size() > 1)
		{
			float x = (line->p1.x + line->p2.x) / 2.f;
			float y = (line->p1.y + line->p2.y) / 2.f;
			Vector2 midPoint = { x, y };
			NavGraphNode* pNode = new NavGraphNode{GetNextFreeNodeIndex(), line->index, midPoint};
			AddNode(pNode);
		}
	}

	//2. Create connections now that every node is created
	std::vector<int> nodeIndexes{};
	nodeIndexes.resize(3);
	for (Elite::Triangle* pTriangle : m_pNavMeshPolygon->GetTriangles())
	{
		nodeIndexes.clear();

		for (int lineIdx : pTriangle->metaData.IndexLines)
		{
			for (auto node : GetAllNodes())
			{
				if (node->GetLineIndex() == lineIdx)
				{
					nodeIndexes.push_back(node->GetIndex());
				}
			}
		}

		switch (nodeIndexes.size())
		{
		case 2:
		{
			GraphConnection2D* pConnection = new GraphConnection2D{ nodeIndexes[0], nodeIndexes[1] };
			AddConnection(pConnection);
			break;
		}
		case 3:
		{
			GraphConnection2D* pConnection = new GraphConnection2D{ nodeIndexes[0], nodeIndexes[1] };
			AddConnection(pConnection);
			pConnection = new GraphConnection2D{ nodeIndexes[1], nodeIndexes[2] };
			AddConnection(pConnection);
			pConnection = new GraphConnection2D{ nodeIndexes[2], nodeIndexes[0] };
			AddConnection(pConnection);
			break;
		}
		default:
			break;
		}
	}

	//3. Set the connections cost to the actual distance
	SetConnectionCostsToDistance();
}

