/*

Compiling in Linux
	~~~~~~~~~~~~~~~~~~
	You will need a modern C++ compiler, so update yours!
	To compile use:

	g++-(gcc version number) YourSource.cpp -o YourProgName -lX11 -lGL -lpthread -lpng -lstdc++fs -std=c++17
	
	ex.: g++-9 main.cpp -o path -lX11 -lGL -lpthread -lpng -lstdc++fs -std=c++17

	On some Linux configurations, the frame rate is locked to the refresh
	rate of the monitor. This engine tries to unlock it but may not be
	able to, in which case try launching your program like this:

	vblank_mode=0 ./YourProgName

	But the normal way is:

	./YourProgName

*/


#include <iostream>
#include <string>
#include <algorithm>

#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"

using namespace std;

#define NodePixelSize 6

class Path : public olc::PixelGameEngine
{
public:
	Path()
	{
		sAppName = "Path";
	}

private:

	struct sNode
	{
		bool bObstacle = false;			// Is the node an obstruction?
		bool bVisited = false;			// Have we searched this node before?
		float fGlobalGoal;				// Distance to goal so far
		float fLocalGoal;				// Distance to goal if we took the alternative route
		int x;							// Nodes position in 2D space
		int y;
		vector<sNode*> vecNeighbours;	// Connections to neighbours
		sNode* parent;					// Node connecting to this node that offers shortest parent
	};

	sNode *nodes = nullptr;
	int nMapWidth = 16;
	int nMapHeight = 16;

	sNode *nodeStart = nullptr;
	sNode *nodeEnd = nullptr;
	

protected:
	virtual bool OnUserCreate()
	{
		
		// Create a 2D array of nodes - this is for convenience of rendering and construction
		// and is not required for the algorithm to work - the nodes could be placed anywhere
		// in any space, in multiple dimensions...
		nodes = new sNode[nMapWidth * nMapHeight];
		for (int x = 0; x < nMapWidth; x++)
			for (int y = 0; y < nMapHeight; y++)
			{
				nodes[y * nMapWidth + x].x = x; // ...because we give each node its own coordinates
				nodes[y * nMapWidth + x].y = y;
				nodes[y * nMapWidth + x].bObstacle = false;
				nodes[y * nMapWidth + x].parent = nullptr;
				nodes[y * nMapWidth + x].bVisited = false;
			}

		// Create connections - in this case nodes are on a regular grid
		for (int x = 0; x < nMapWidth; x++)
			for (int y = 0; y < nMapHeight; y++)
			{
				if(y>0)
					nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y - 1) * nMapWidth + (x + 0)]);
				if(y<nMapHeight-1)
					nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y + 1) * nMapWidth + (x + 0)]);
				if (x>0)
					nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y + 0) * nMapWidth + (x - 1)]);
				if(x<nMapWidth-1)
					nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y + 0) * nMapWidth + (x + 1)]);

				// We can also connect diagonally
				//if (y>0 && x>0)
				//	nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y - 1) * nMapWidth + (x - 1)]);
				//if (y<nMapHeight-1 && x>0)
				//	nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y + 1) * nMapWidth + (x - 1)]);
				//if (y>0 && x<nMapWidth-1)
				//	nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y - 1) * nMapWidth + (x + 1)]);
				//if (y<nMapHeight - 1 && x<nMapWidth-1)
				//	nodes[y*nMapWidth + x].vecNeighbours.push_back(&nodes[(y + 1) * nMapWidth + (x + 1)]);
				
			}

		// Manually positio the start and end markers so they are not nullptr
		nodeStart = &nodes[(nMapHeight / 2) * nMapWidth + 1];
		nodeEnd = &nodes[(nMapHeight / 2) * nMapWidth + nMapWidth-2];
		
		return true;
	}

	bool Solve_AStar()
	{
		
		// Reset Navigation Graph - default all node states
		for (int x = 0; x < nMapWidth; x++)
			for (int y = 0; y < nMapHeight; y++)
			{
				nodes[y*nMapWidth + x].bVisited = false;
				nodes[y*nMapWidth + x].fGlobalGoal = INFINITY;
				nodes[y*nMapWidth + x].fLocalGoal = INFINITY;
				nodes[y*nMapWidth + x].parent = nullptr;	// No parents
			}

		auto distance = [](sNode* a, sNode* b) // For convenience
		{
			return sqrtf((a->x - b->x)*(a->x - b->x) + (a->y - b->y)*(a->y - b->y));
		};

		auto heuristic = [distance](sNode* a, sNode* b) // So we can experiment with heuristic
		{
			return distance(a, b);
		};

		// Setup starting conditions
		sNode *nodeCurrent = nodeStart;
		nodeStart->fLocalGoal = 0.0f;
		nodeStart->fGlobalGoal = heuristic(nodeStart, nodeEnd);

		// Add start node to not tested list - this will ensure it gets tested.
		// As the algorithm progresses, newly discovered nodes get added to this
		// list, and will themselves be tested later
		list<sNode*> listNotTestedNodes;
		listNotTestedNodes.push_back(nodeStart);

		// if the not tested list contains nodes, there may be better paths
		// which have not yet been explored. However, we will also stop 
		// searching when we reach the target - there may well be better
		// paths but this one will do - it wont be the longest.
		while (!listNotTestedNodes.empty() && nodeCurrent != nodeEnd)// Find absolutely shortest path // && nodeCurrent != nodeEnd)
		{
			// Sort Untested nodes by global goal, so lowest is first
			listNotTestedNodes.sort([](const sNode* lhs, const sNode* rhs){ return lhs->fGlobalGoal < rhs->fGlobalGoal; } );
			
			// Front of listNotTestedNodes is potentially the lowest distance node. Our
			// list may also contain nodes that have been visited, so ditch these...
			while(!listNotTestedNodes.empty() && listNotTestedNodes.front()->bVisited)
				listNotTestedNodes.pop_front();

			// ...or abort because there are no valid nodes left to test
			if (listNotTestedNodes.empty())
				break;

			nodeCurrent = listNotTestedNodes.front();
			nodeCurrent->bVisited = true; // We only explore a node once
			
					
			// Check each of this node's neighbours...
			for (auto nodeNeighbour : nodeCurrent->vecNeighbours)
			{
				// ... and only if the neighbour is not visited and is 
				// not an obstacle, add it to NotTested List
				if (!nodeNeighbour->bVisited && nodeNeighbour->bObstacle == 0)
					listNotTestedNodes.push_back(nodeNeighbour);

				// Calculate the neighbours potential lowest parent distance
				float fPossiblyLowerGoal = nodeCurrent->fLocalGoal + distance(nodeCurrent, nodeNeighbour);

				// If choosing to path through this node is a lower distance than what 
				// the neighbour currently has set, update the neighbour to use this node
				// as the path source, and set its distance scores as necessary
				if (fPossiblyLowerGoal < nodeNeighbour->fLocalGoal)
				{
					nodeNeighbour->parent = nodeCurrent;
					nodeNeighbour->fLocalGoal = fPossiblyLowerGoal;

					// The best path length to the neighbour being tested has changed, so
					// update the neighbour's score. The heuristic is used to globally bias
					// the path algorithm, so it knows if its getting better or worse. At some
					// point the algo will realise this path is worse and abandon it, and then go
					// and search along the next best path.
					nodeNeighbour->fGlobalGoal = nodeNeighbour->fLocalGoal + heuristic(nodeNeighbour, nodeEnd);
				}
			}	
		}

		return true;
	}

	virtual bool OnUserUpdate(float fElapsedTime) 
	{
		int nNodeSize = 9;
		int nNodeBorder = 2;

		// Use integer division to nicely get cursor position in node space
		int nSelectedNodeX = GetMouseX() / nNodeSize;
		int nSelectedNodeY = GetMouseY() / nNodeSize;

		if (GetMouse(0).bReleased) // Use mouse to draw maze, shift and ctrl to place start and end
		{
			if(nSelectedNodeX >=0 && nSelectedNodeX < nMapWidth)
				if (nSelectedNodeY >= 0 && nSelectedNodeY < nMapHeight)
				{
					if (GetKey(olc::Key::SHIFT).bHeld)

						nodeStart = &nodes[nSelectedNodeY * nMapWidth + nSelectedNodeX];
					else if (GetKey(olc::Key::CTRL).bHeld)
						nodeEnd = &nodes[nSelectedNodeY * nMapWidth + nSelectedNodeX];
					else
						nodes[nSelectedNodeY * nMapWidth + nSelectedNodeX].bObstacle = !nodes[nSelectedNodeY * nMapWidth + nSelectedNodeX].bObstacle;

					Solve_AStar(); // Solve in "real-time" gives a nice effect
				}
		}

		// Draw Connections First - lines from this nodes position to its
		// connected neighbour node positions
		FillRect(0, 0, ScreenWidth(), ScreenHeight(), olc::BLACK);
		for (int x = 0; x < nMapWidth; x++)
			for (int y = 0; y < nMapHeight; y++)
			{
				for (auto n : nodes[y*nMapWidth + x].vecNeighbours)
				{
					DrawLine(x*nNodeSize + nNodeSize / 2, y*nNodeSize + nNodeSize / 2,
						n->x*nNodeSize + nNodeSize / 2, n->y*nNodeSize + nNodeSize / 2, olc::DARK_BLUE);
				}
			}

		// Draw Nodes on top
		for (int x = 0; x < nMapWidth; x++)
			for (int y = 0; y < nMapHeight; y++)
			{
	
				FillRect(x*nNodeSize + nNodeBorder, y*nNodeSize + nNodeBorder, 
					NodePixelSize, NodePixelSize, 
					nodes[y * nMapWidth + x].bObstacle ? olc::GREY : olc::BLUE);

				if (nodes[y * nMapWidth + x].bVisited)
					FillRect(x*nNodeSize + nNodeBorder, y*nNodeSize + nNodeBorder, NodePixelSize, NodePixelSize, olc::DARK_CYAN);

				if(&nodes[y * nMapWidth + x] == nodeStart)
					FillRect(x*nNodeSize + nNodeBorder, y*nNodeSize + nNodeBorder, NodePixelSize, NodePixelSize, olc::GREEN);

				if(&nodes[y * nMapWidth + x] == nodeEnd)
					FillRect(x*nNodeSize + nNodeBorder, y*nNodeSize + nNodeBorder, NodePixelSize, NodePixelSize, olc::RED);						
				
			}


		// Draw Path by starting ath the end, and following the parent node trail
		// back to the start - the start node will not have a parent path to follow
		if (nodeEnd != nullptr)
		{
			sNode *p = nodeEnd;
			while (p->parent != nullptr)
			{
				DrawLine(p->x*nNodeSize + nNodeSize / 2, p->y*nNodeSize + nNodeSize / 2,
					p->parent->x*nNodeSize + nNodeSize / 2, p->parent->y*nNodeSize + nNodeSize / 2, olc::YELLOW);
				
				// Set next node to this node's parent
				p = p->parent;
			}
		}

		return true;
	}

};

int main()
{	
	Path path;
	if(path.Construct(160, 160, NodePixelSize, NodePixelSize))
		path.Start();
	return 0;
}