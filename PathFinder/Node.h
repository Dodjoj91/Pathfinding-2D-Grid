#pragma once
#include "BaseEnums.h"


namespace AI
{
	class Node
	{
	public:
		struct NodeComparator
		{
			bool operator()(const Node* a, const Node* b) const
			{
				if (a->GetTotalCost() != b->GetTotalCost())
				{
					return a->GetTotalCost() < b->GetTotalCost();
				}
				else
				{
					if (a->GetHCost() != b->GetHCost()) { return a->GetHCost() < b->GetHCost(); }
					else { return a->GetMapIndex() < b->GetMapIndex(); } //If there is a tie we will compare them with their unique map index
				}
			}

		};


		Node()
		{
			m_column = -1;
			m_row = -1;
			m_mapIndex = -1;

			m_gCost = 0u;
			m_hCost = 0u;
			m_totalCost = 0u;
			m_areaCost = 0u;

			m_isInsideNodeSet = false;
			m_isVisited = false;
			m_pathType = EPathType::Path;
			m_parentNode = nullptr;
		}

		//Get Functions
		inline int GetRow() const { return m_row; }
		inline int GetColumn() const { return m_column; }
		inline int GetMapIndex() const { return m_mapIndex; }
		inline unsigned int GetGCost() const { return m_gCost; }
		inline unsigned int GetHCost() const { return m_hCost; }
		inline unsigned int GetTotalCost() const { return m_totalCost; }

		inline bool GetIsInsideNodeSet() const { return m_isInsideNodeSet; }
		inline bool GetIsObstacle() const { return m_pathType == EPathType::Obstacle; }
		inline bool GetIsVisited() const { return m_isVisited; }
		inline bool GetIsHCalculated() const { return m_hCost > 0; }

		inline EPathType GetPathType() const { return m_pathType; }
		const inline Node* GetParent() const { return m_parentNode; }

		//Set Functions
		inline void SetGCost(const unsigned int newGCost) { m_gCost = newGCost; }
		inline void SetHCost(const unsigned int newHCost) { m_hCost = newHCost; }
		inline void SetAreaCost(const unsigned int newAreaCost) { m_areaCost = newAreaCost; }
		inline void SetRealMapIndex(const int newMapIndex) { m_mapIndex = newMapIndex; }
		inline void SetColumn(const int newColumn) { m_column = newColumn; }
		inline void SetRow(const int newRow) { m_row = newRow; }
		inline void SetVisited() { m_isVisited = true; }
		inline void SetInsideNodeSet(const bool isInside) { m_isInsideNodeSet = isInside; }

		void SetPathType(const int pathValue)
		{
			if (pathValue >= 0 && pathValue < static_cast<int>(EPathType::Count))
			{
				m_pathType = static_cast<EPathType>(pathValue);
			}
		}

		inline void SetParent(Node* const nodeToParent)
		{
			m_parentNode = nodeToParent;
		}

		inline void CalculateTotalCost() { m_totalCost = m_gCost + m_hCost + m_areaCost; }

	private:
		unsigned int m_gCost;
		unsigned int m_hCost;
		unsigned int m_areaCost;
		unsigned int m_totalCost;

		int m_mapIndex;
		int m_column;
		int m_row;

		bool m_isInsideNodeSet;
		bool m_isVisited;
		EPathType m_pathType;
		Node* m_parentNode;

	};
}

