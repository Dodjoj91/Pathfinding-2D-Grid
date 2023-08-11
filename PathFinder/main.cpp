#include <string>
#include <random>

#include "Macros.h"
#include "PathFinder.h"


void StartFindPath();
void WritePath(std::vector<int>& Map, std::pair<int, int>& MapDimensions, std::vector<int>& OutPath);

int main()
{
	StartFindPath();
}

void StartFindPath()
{
	std::vector<int> Map = { 1, 1, 1, 1,
							 1, 1, 0, 1,
							 0, 1, 1, 1,
							 0, 1, 1, 0,
							 0, 1, 1, 1,
							 1, 1, 0, 1,
							 1, 1, 1, 1 };

	std::pair<int, int> MapDimensions = { 4, 7 };
	std::vector<int> OutPath;
	
	AI::PathFinder::SetOptions(EPathBuilderType::AStar, EHeuristicType::Euclidean);
	
	if (AI::PathFinder::FindPath({ 0, 0 }, { 2, 6 }, Map, MapDimensions, OutPath))
	{
		if (OutPath.size() > 0)
		{
			WritePath(Map, MapDimensions, OutPath);
		}
	}
}


void WritePath(std::vector<int>& Map, std::pair<int, int>& MapDimensions, std::vector<int>& OutPath)
{
	int indexCounter = 0;

	for (int y = 0; y < MapDimensions.second; y++)
	{
		for (int x = 0; x < MapDimensions.first; x++)
		{
			auto it = std::find(OutPath.begin(), OutPath.end(), indexCounter);

			std::string visitString = "";

			if (OutPath.end() != it)
			{
				visitString = "[P]";
			}
			else
			{
				if (Map[indexCounter] == 0) { visitString = "[X]"; }
				else { visitString = "[O]"; }
			}

			std::cout << visitString;
			indexCounter++;
		}

		std::cout << std::endl;
	}
}