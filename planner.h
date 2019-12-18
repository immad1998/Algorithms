#pragma once
#include <iostream>
#include <random>
#include <vector>
#include "Objects.h"
#define MAX_MAP_SIZE 100
#define MIN_MAP_SIZE 0

struct Node																				
{
	int index;
	int predecessor_index;
	double x_config;
	double y_config;
};

struct Tree																				
{
	std::string name;
	std::vector<Node> node;
	int num_nodes;
};

enum Status {ADVANCE, REACHED , TRAPPED};

class RRT_Planner
{
private:
	Tree Tstart;
	std::uniform_real_distribution<> uni_rand_gen;

	double object_length;
	double object_width;
	Node start_node;
	Node goal_node;

	bool sample_goal;
public:

	std::vector<Node> path;
	RRT_Planner() : sample_goal(false), uni_rand_gen(MIN_MAP_SIZE, MAX_MAP_SIZE) {};


	

	bool isConfigValid(Node& r_node, SquareObject object)		//  Checks if configuration is not colliding with any 
	{																							//	obstacle.
		/*if((r_node.x_config < 5 ||															
		r_node.x_config > 95) &&
		(r_node.y_config )*/
		if (r_node.x_config > MAX_MAP_SIZE ||
			r_node.x_config < MIN_MAP_SIZE ||
			r_node.y_config > MAX_MAP_SIZE ||
			r_node.y_config < MIN_MAP_SIZE)
		{
			std::cout << "The Start or Goal Node is out of the given area\n" << std::endl;
			return false;
		}

		if ((r_node.x_config < (object.getX_cood() - (object.getLength() / 2)) ||
			r_node.x_config >(object.getX_cood() + (object.getLength() / 2))) ||
			(r_node.y_config < (object.getY_cood() - (object.getLength() / 2)) ||
				r_node.y_config >(object.getY_cood() + (object.getLength() / 2))))
			return true;
		else
			return false;
	}

	void getRandomConfig(Node& r_node, SquareObject object)				// Gets a random node in the grid.
	{
		std::random_device rd;
		std::mt19937 gen(rd());
		std::uniform_real_distribution<> dis(0, 10);

		if (dis(gen) <= 1)
			sample_goal = true;
		else
			sample_goal = false;

		if (sample_goal)
		{
			r_node.x_config = goal_node.x_config;
			r_node.y_config = goal_node.y_config;
		}
		else
		{
			std::random_device rd;
			std::mt19937 gen(rd());

			bool is_valid = false;

			while (!(is_valid))
			{
				r_node.x_config = uni_rand_gen(gen);
				//std::cout << "X.Configuration : " << r_node.x_config << std::endl;
				r_node.y_config = uni_rand_gen(gen);
				//std::cout << "Y.Configuration : " << r_node.y_config << std::endl;


				is_valid = isConfigValid(r_node, object);
				if (is_valid)
					break;
				//std::cout << "invalid configuration\n" << std::endl;
			}
			//std::cout << "The configuration is valid\n" << std::endl;
		}
	}

	bool setStartGoalConfig(double start_x_config, double start_y_config, double goal_x_config, double goal_y_config, SquareObject object)
	{
		Node start_node_;
		start_node_.index = 0;
		start_node_.x_config = start_x_config;
		start_node_.y_config = start_y_config;
		start_node_.predecessor_index = 0;
		if (!isConfigValid(start_node_, object))
		{
			std::cout << "The Start Node configuration is invalid.\n" << std::endl;
			return false;
		}

		Node goal_node_;
		goal_node_.index = 0;
		goal_node_.x_config = start_x_config;
		goal_node_.y_config = start_y_config;
		goal_node_.predecessor_index = 0;
		if (!isConfigValid(goal_node_, object))
		{
			std::cout << "The Goal Node configuration is invalid.\n" << std::endl;
			return false;
		}

		start_node = start_node_;
		goal_node = goal_node_;

		Tstart.node.push_back(start_node);
		Tstart.num_nodes = 1;
		std::cout << "The Start and Goal node configuration is set.\n" << std::endl;

	}

	void findNearestNeighbour(Tree Tcurrent, Node r_rand, Node& nearest_node)
	{
		int nindex = -1;
		double best_dist = 100000;
		double node_dist = 0;
		double sum_sqr = 0;
		double x_distance = 0;
		double y_distance = 0;

		for (int i = 0; i < Tcurrent.num_nodes; i++)
		{
			x_distance = r_rand.x_config - Tcurrent.node[i].x_config;
			y_distance = r_rand.y_config - Tcurrent.node[i].y_config;

			sum_sqr = (x_distance*x_distance) + (y_distance*y_distance);

			node_dist = sqrt(sum_sqr);

			if (node_dist < best_dist)
			{
				best_dist = node_dist;
				nindex = i;
			}
		}

		if (nindex < 0)
		{
			std::cout << "There is no nearest neighbour.\n" << std::endl;
		}
		else
		{
			std::cout << "Nearest Neighbour found.\n" << std::endl;
			nearest_node.index = Tcurrent.node[nindex].index;
			nearest_node.predecessor_index = Tcurrent.node[nindex].predecessor_index;
			nearest_node.x_config = Tcurrent.node[nindex].x_config;
			nearest_node.y_config = Tcurrent.node[nindex].y_config;
		}
	}

	Status generateRandomNode(Node near_node, Node r_node, Node& new_node, double max_step_size, SquareObject object) // uses generates the new node in accordance with the step provided
	{
		Status status = TRAPPED;
		double x_distance;
		double y_distance;
		double sum_square = 0.0;
		double dist_norm = 0.0;

		x_distance = r_node.x_config - near_node.x_config;
		y_distance = r_node.y_config - near_node.y_config;
		sum_square = (x_distance*x_distance) + (y_distance*y_distance);
		dist_norm = sqrt(sum_square);

		double x_direction;
		double y_direction;
		double x_increment;
		double y_increment;
		double sum_square_incr = 0;
		double dist_norm_incr = 0;

		x_direction = x_distance / dist_norm;
		y_direction = y_distance / dist_norm;
		x_increment = max_step_size * x_direction;
		y_increment = max_step_size * y_direction;
		sum_square_incr = (x_increment*x_increment) + (y_increment*y_increment);
		dist_norm_incr = sqrt(sum_square_incr);

		double x_temp;
		double y_temp;

		x_temp = near_node.x_config + x_increment;
		y_temp = near_node.y_config + y_increment;

		Node temp_node;
		temp_node.x_config = x_temp;
		temp_node.y_config = y_temp;

		if (!isConfigValid(temp_node, object))
		{
			status = TRAPPED;
			return status;
		}
		else if (dist_norm_incr < dist_norm)
		{
			status = ADVANCE;
			new_node.x_config = x_temp;
			new_node.y_config = y_temp;
		}
		else
		{
			new_node.x_config = r_node.x_config;
			new_node.y_config = r_node.y_config;
			status = REACHED;
		}
		return status;
	}

	void addConfigToTree(Tree& tree, Node near_node, Node new_node)  //keeps track of the number of node and predecessor index etc.
	{
		new_node.index = tree.node.size();
		new_node.predecessor_index = near_node.index;

		tree.node.push_back(new_node);
		tree.num_nodes = tree.node.size();
	}

	Status extendTree(Tree& tree, Node rand_node, Node near_node, Node new_node, double max_step_size, SquareObject object) //adds node to the tree
	{
		Status tree_state;
		findNearestNeighbour(tree, rand_node, near_node);
		tree_state = generateRandomNode(near_node, rand_node, new_node, max_step_size, object);

		if (tree_state != TRAPPED)
			addConfigToTree(tree, near_node, new_node);
		
		return tree_state;
	}

	bool solveQuery(int max_iter, double max_step_size, double start_Xconfig, double start_Yconfig, double goal_Xconfig, double goal_Yconfig, SquareObject object) //main cheez
	{
		if (!setStartGoalConfig(start_Xconfig, start_Yconfig, goal_Xconfig, goal_Yconfig, object))
			throw 0;

		Node rand;
		Node near;
		Node new_n;

		near.index = 0;
		near.predecessor_index = -1;
		new_n.index = 0;
		near.predecessor_index = -1;


		Status returned;
		bool path_found = false;

		for (int i = 0; i < max_iter; i++)
		{
			std::cout << "Iteration number: " << i << std::endl;
			getRandomConfig(rand, object);
			returned = extendTree(Tstart, rand, near, new_n, max_step_size, object);

			if (returned == REACHED)
			{
				if (rand.x_config - goal_node.x_config < 0.1 &&
					rand.y_config - goal_node.y_config < 0.1)
				{
					path_found = true;					//YAAR KAHIN MASLA HO TOU CALL ME ON WHATSAAP :)
				}

				if (path_found)
				{
					printf("Path Found: goal to start!!");
					Node start_connect = Tstart.node.back();
					getSolutionPath(Tstart, start_connect);

					Tstart.node.clear();
					Tstart.num_nodes = 0;

					return true;
				}
			} 
		}

		Tstart.node.clear();
		Tstart.num_nodes = 0;

		printf("NO PATH FOUND!!");

		return false;
	}

	void getSolutionPath(Tree &tree, Node node_connect)
	{
		std::vector<Node> solution_path;
		//std::vector <double> path_config(2);
		Node node_in;
		Node current = node_connect;

		while (current.index != 0)
		{
			//path_config[0] = current.x_config;
			//path_config[1] = current.y_config;
			node_in.x_config = current.x_config;
			node_in.y_config = current.y_config;
			solution_path.insert(solution_path.begin(), node_in);
			current = tree.node[current.predecessor_index];
		}

		node_in.x_config = current.x_config;
		node_in.y_config = current.y_config;
		//path_config[0] = current.x_config;
		//path_config[1] = current.y_config;
		solution_path.insert(solution_path.begin(), node_in);

		path = solution_path;

	}
};