# Pathfinding-2D-Grid

	Author: Daniel Jonsson


	Introduction:

	Hello!
	This is a Pathfinder with focus on using the AStar algorithm.

	I have some files that is included to this pathfinding:

	PathFinder.h - Header file, which includes all the functions for finding the shortest path
	PathFinder.cpp - Source file, for the pathfinding algorithm
	BaseEnums.h - Different enums regarding pathfinding
	Macros.h - Macro for logging errors
	Node.h - The node class that have all the settings during the main pathfinding

	I have set it under the namespace AI, so to call pathfinding functions we need to access that first ex: AI::PathFinder::FindPath().


	We have two public functions that can be used:

	------------------------------------------------------------------------------------------------------------------

	Function 1: SetOptions(const EPathBuilderType pathType, const EHeuristicType heuristicType)

	This will take in two arguments, my thoughts about this was to set different builder algorithms for ex: Breadth First, Dijkstras & AStar.
	But as it is now only the AStar is implemented and it is defaulted to that.

	AStar:
	It will calculate values based on the distance from start to the target and also count each step it takes to every neighbor it encounter, making it efficient
	to reach the goal fast.

	The heuristic type we can use are two different versions: Euclidean & Manhattan.

	Euclidean:
	Will calculate start to the goal with and raises the power by value of 2. Making the cost quite high on each neigbour, but it got a more smoother direction
	towards the goal by not following corners.

	Manhattan:
	Takes the absolute value from start to the goal, making it to walk towards corners. It is cheaper to calculate but we do not get that smooth transition
	by going directly to the target.

	------------------------------------------------------------------------------------------------------------------

	Function 2: FindPath(std::pair<int, int> Start, std::pair<int, int> Target, const std::vector<int>& Map, std::pair<int, int> MapDimensions, std::vector<int>& OutPath);

	This is the main function that will get you the path,


	Example usage:

	std::vector<int> Map = { 1, 1, 1, 1, 1, 
	                         1, 1, 1, 1, 1, 
	                         1, 1, 1, 1, 1 };
                    
	std::vector<int> OutPath;

	std::pair<int, int> Start = { 0, 0 };
	std::pair<int, int> Target = { 3, 2 };

	std::pair<int, int> MapDimensions = { 5, 3 };


	AI::PathFinder::SetOptions(EPathBuilderType::AStar, EHeuristicType::Manhattan);

	if (AI::PathFinder::FindPath(Start, Target, Map, MapDimensions, OutPath))
	{
		if (OutPath.size() > 0)
		{
			for (const int pathIndex : OutPath)
			{
				LOG(ELogType::Info, std::string("PATH: " + std::to_string(pathIndex)));
			}
		}
	}

	We use a full open Map with no obstacles.
	We also set the builder options.

	We navigate from:
	Start: 0, 0
	Target: 3, 2

	We create the size of the map which it will use to create all the nodes, that we set to: 5, 3.
	After building it will return the OutPath and we can then LOG it to the console showing the final path taken.

------------------------------------------------------------------------------------------------------------------
