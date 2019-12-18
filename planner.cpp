#include <iostream>
#include <random>
#include <vector>
#include "Objects.h"
#include "planner.h"

void main() {
	RRT_Planner planner;
	Node newNode;
	SquareObject object(90, 50 ,50);
	if(planner.solveQuery(1000,1,2,2,98,98,object))
	{	
		std::cout << std::endl << "SUCCESS" << std::endl;
	}
	system("PAUSE");
}