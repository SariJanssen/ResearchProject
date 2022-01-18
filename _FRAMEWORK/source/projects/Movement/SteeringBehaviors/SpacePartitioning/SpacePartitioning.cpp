#include "stdafx.h"
#include "SpacePartitioning.h"
#include "projects\Movement\SteeringBehaviors\SteeringAgent.h"

// --- Cell ---
// ------------
Cell::Cell(float left, float bottom, float width, float height)
{
	boundingBox.bottomLeft = { left, bottom };
	boundingBox.width = width;
	boundingBox.height = height;
}

std::vector<Elite::Vector2> Cell::GetRectPoints() const
{
	auto left = boundingBox.bottomLeft.x;
	auto bottom = boundingBox.bottomLeft.y;
	auto width = boundingBox.width;
	auto height = boundingBox.height;

	std::vector<Elite::Vector2> rectPoints =
	{
		{ left , bottom  },
		{ left , bottom + height  },
		{ left + width , bottom + height },
		{ left + width , bottom  },
	};

	return rectPoints;
}

// --- Partitioned Space ---
// -------------------------
CellSpace::CellSpace(float width, float height, int rows, int cols, int maxEntities)
	: m_SpaceWidth(width)
	, m_SpaceHeight(height)
	, m_NrOfRows(rows)
	, m_NrOfCols(cols)
	, m_Neighbors(maxEntities)
	, m_NrOfNeighbors(0)
	, m_CellHeight{ height * 2 / rows }
	, m_CellWidth{ width * 2 / cols }
{
	for (int y{}; y < rows; ++y)
	{
		for (int x{}; x < cols; ++x)
		{
			m_Cells.push_back(Cell{ -width + (x * m_CellWidth), -height + (y * m_CellHeight) , m_CellWidth, m_CellHeight });
		}
	}
}

void CellSpace::AddAgent(SteeringAgent* agent)
{
	int position = PositionToIndex(agent->GetPosition());
	m_Cells[position].agents.push_back(agent);
}

void CellSpace::UpdateAgentCell(SteeringAgent* agent, Elite::Vector2 oldPos)
{
	int position = PositionToIndex(agent->GetPosition());
	int oldPosition = PositionToIndex(oldPos);

	if (position != oldPosition)
	{
		m_Cells[oldPosition].agents.remove(agent);
		m_Cells[position].agents.push_back(agent);
	}
}

void CellSpace::RegisterNeighbors(SteeringAgent* agent, float queryRadius)
{
	Elite::Vector2 position{ agent->GetPosition() };
	int agentIndex{ PositionToIndex(position) };

	// Get indexes from surrounding positions
	Elite::Vector2 upPos{ position.x, ClampY(position.y - queryRadius) };
	Elite::Vector2 downPos{ position.x, ClampY(position.y + queryRadius) };
	Elite::Vector2 rightPos{ ClampX(position.x + queryRadius), position.y};
	Elite::Vector2 leftPos{ ClampX(position.x - queryRadius), position.y};

	int upIndex{ PositionToIndex(upPos) };
	int downIndex{ PositionToIndex(downPos) };
	int rightIndex{ PositionToIndex(rightPos) };
	int leftIndex{ PositionToIndex(leftPos) };

	int upLeft{ upIndex - (agentIndex - leftIndex) };

	// Get index shift up and left
	int xIndex{ rightIndex - leftIndex };
	int yIndex{ (downIndex - upIndex) / m_NrOfRows };

	m_NrOfNeighbors = 0;
	for (int y{}; y <= yIndex; ++y)
	{
		for (int x{}; x <= xIndex; ++x)
		{
			int index{ upLeft + x + (y * m_NrOfRows) };

			for (SteeringAgent* pAgent : m_Cells[index].agents) // Are inside boudingbox of query
			{
				if (pAgent == agent)
				{
					continue;
				}

				// Check if inside actual query
				float distanceToAgent = Elite::Distance(position, pAgent->GetPosition());
				if (distanceToAgent <= queryRadius)
				{
					m_Neighbors[m_NrOfNeighbors] = pAgent;
					++m_NrOfNeighbors;
				}
			}

			// Debug Rendering: render currently active query boundingbox
			if (agent->CanRenderBehavior())
			{
				auto pol = Elite::Polygon(m_Cells[index].GetRectPoints());
				DEBUGRENDERER2D->DrawPolygon(&pol, { 1,1,0 }, 0.35f);
			}
		}
	}
}

void CellSpace::RenderCells() const
{
	for (Cell cell : m_Cells)
	{
		auto pol = Elite::Polygon(cell.GetRectPoints());
		DEBUGRENDERER2D->DrawPolygon(&pol, { 1,0,0 }, 0.36f);

		Elite::Vector2 loc{ cell.GetRectPoints()[1] };
		std::string amount{ std::to_string(cell.agents.size()) };
		DEBUGRENDERER2D->DrawString(loc, amount.c_str(), 0.36f);
	}
}

int CellSpace::PositionToIndex(const Elite::Vector2 pos) const
{
	for (size_t i = 0; i < m_Cells.size(); ++i)
	{
		float left{ m_Cells[i].boundingBox.bottomLeft.x };
		float right{ left + m_Cells[i].boundingBox.width };
		float bottom{ m_Cells[i].boundingBox.bottomLeft.y };
		float top{ bottom + m_Cells[i].boundingBox.height };

		if ((pos.x > left || Elite::AreEqual(pos.x, left)) // pos.x >= left
			&& (pos.x < right || Elite::AreEqual(pos.x, right))) // pos.x <= right
		{
			if ((pos.y > bottom || Elite::AreEqual(pos.y, bottom))  // pos.y >= bottom
				&& (pos.y < top || Elite::AreEqual(pos.y, top))) // pos.y <= top
			{
				return i;
			}
		}
	}
	assert(false && "Position not found on grid!");
	return -1;
}

float CellSpace::ClampY(float y)
{
	if (y >= m_SpaceHeight)
	{
		return m_SpaceHeight;
	}
	else if (y <= -m_SpaceHeight)
	{
		return -m_SpaceHeight;
	}

	return y;
}

float CellSpace::ClampX(float x)
{
	if (x >= m_SpaceWidth)
	{
		return m_SpaceWidth;
	}
	else if (x <= -m_SpaceWidth)
	{
		return -m_SpaceWidth;
	}

	return x;
}
