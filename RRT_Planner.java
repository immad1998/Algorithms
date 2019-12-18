package oopProject;
import java.io.*;
import java.util.*;

//import oopa.SquareObject;

//import java.io.ObjectOutputStream;


public class RRT_Planner {
	
	
	public final double MAX_MAP_SIZE=100;
	public final double MIN_MAP_SIZE=0;
	
	Tree Tstart = new Tree();
	Random uni_rand_gen= new Random();
	
	double object_length;
	double object_width;
	Node start_node=new Node();
	Node goal_node=new Node();
	
	boolean sample_goal;
	
		Vector<Node> path=new Vector<Node>();
		
		//ArrayList<Tree> Tstart=new ArrayList<Tree>(100000);
	
	RRT_Planner(){
		sample_goal=false;
		uni_rand_gen.nextDouble();// * (MAX_MAP_SIZE - -MIN_MAP_SIZE)+ MIN_MAP_SIZE;
	}
	
	public boolean isConfigValid(Node r_node, SquareObject object) {
		
		if (r_node.x_config > MAX_MAP_SIZE ||
				r_node.x_config < MIN_MAP_SIZE ||
				r_node.y_config > MAX_MAP_SIZE ||
				r_node.y_config < MIN_MAP_SIZE)
			{
				System.out.print("The Start or Goal Node is out of the given area\n");
				return false;
			}
		
		if ((r_node.x_config < (object.getX_cood() - (object.getLength() / 2)) ||
				r_node.x_config >(object.getX_cood() + (object.getLength() / 2))) ||
				(r_node.y_config < (object.getY_cood() - (object.getLength() / 2)) ||
				r_node.y_config >(object.getY_cood() + (object.getLength() / 2))))
				{
					return true;
				}
		else 		
			return false;
		
		
	}
	
	
	void getRandomConfig(Node r_node, SquareObject object)				// Gets a random node in the grid.
	{
		double max=10;
		double min=0,val=1;
		Random rd = new Random();
		double dis = (min + (max - min) * rd.nextDouble());
		System.out.println("dis valus is  "+dis);
		if (dis <= 1)
			{
		//	System.out.println("****   HERE3   *****");
			sample_goal = true;
			}
		else
			{
		//	System.out.println("****   HERE4   *****");
			sample_goal = false;}

		if (sample_goal)
		{
			//System.out.println("****   HERE5   *****");
			r_node.x_config = goal_node.x_config;
			r_node.y_config = goal_node.y_config;
		}
		else
		{
			
		//	System.out.println("****   HERE6   *****");
			boolean is_valid = false;

			while (!(is_valid))
			{
				//System.out.println("****   HERE7   *****");
				r_node.x_config = (rd.nextDouble()*100);
				System.out.println("X.Configuration : " + r_node.x_config );
				r_node.y_config = (rd.nextDouble()*100);
				System.out.println("Y.Configuration : " + r_node.y_config );
				

				is_valid = isConfigValid(r_node, object);
				//System.out.println("value ofis_valid " + is_valid);
				if (is_valid) {
				//	System.out.println("****   HERE8   *****");
					
					break;}
				//std::cout << "invalid configuration\n" << std::endl;
			}
			//std::cout << "The configuration is valid\n" << std::endl;
		}
	}
	
	
	public boolean setStartGoalConfig(double start_x_config, double start_y_config, double goal_x_config, double goal_y_config, SquareObject object) {
		Node start_node_ = new Node();
		start_node_.index = 0;
		start_node_.x_config = start_x_config;
		start_node_.y_config = start_y_config;
		start_node_.predecessor_index = 0;
		if (!isConfigValid(start_node_, object))
		{
			System.out.print("The Start Node configuration is invalid.\n" ); 
			return false;
		}
		Node goal_node_ = new Node();
		goal_node_.index = 0;
		goal_node_.x_config = start_x_config;
		goal_node_.y_config = start_y_config;
		goal_node_.predecessor_index = 0;
		if (!isConfigValid(goal_node_, object))
		{
			System.out.print( "The Goal Node configuration is invalid.\n" );
			return false;
		}
		start_node = start_node_;
		goal_node = goal_node_;

		Tstart.node.add(start_node);
		Tstart.num_nodes = 1;
		System.out.print("The Start and Goal node configuration is set.\n"); 
		return true;

	}
	
	void findNearestNeighbour(Tree Tcurrent, Node r_rand, Node nearest_node)
	{
		int nindex = -1;
		double best_dist = 100000;
		double node_dist = 0;
		double sum_sqr = 0;
		double x_distance = 0;
		double y_distance = 0;
		
		//Object[] Tcurrent= T_current.toArray();
		
		for (int i = 0; i < Tcurrent.num_nodes; i++)
		{
			//System.out.println("****   HERE9   *****");
			x_distance = r_rand.x_config - Tcurrent.node.get(i).x_config;
			y_distance = r_rand.y_config - Tcurrent.node.get(i).y_config;

			sum_sqr = (x_distance*x_distance) + (y_distance*y_distance);

			node_dist = Math.sqrt(sum_sqr);

			if (node_dist < best_dist)
			{
			//	System.out.println("****   HERE10   *****");
				best_dist = node_dist;
				nindex = i;
			}
		}

		if (nindex < 0)
		{
			//System.out.println("****   HERE11   *****");
			System.out.print("\"There is no nearest neighbour.\\n\" ");
		}
		else
		{
		//	System.out.println("****   HERE12   *****");
			System.out.print("Nearest Neighbour found.\n");
			nearest_node.index = Tcurrent.node.get(nindex).index;
			nearest_node.predecessor_index = Tcurrent.node.get(nindex).predecessor_index;
			nearest_node.x_config = Tcurrent.node.get(nindex).x_config;
			nearest_node.y_config = Tcurrent.node.get(nindex).y_config;
		}
	}
	
	Status generateRandomNode(Node near_node, Node r_node, Node new_node, double max_step_size, SquareObject object) // uses generates the new node in accordance with the step provided
	{
		Status status = Status.TRAPPED;
		double x_distance;
		double y_distance;
		double sum_square = 0.0;
		double dist_norm = 0.0;

		x_distance = r_node.x_config - near_node.x_config;
		y_distance = r_node.y_config - near_node.y_config;
		sum_square = (x_distance*x_distance) + (y_distance*y_distance);
		dist_norm = Math.sqrt(sum_square);

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
		dist_norm_incr = Math.sqrt(sum_square_incr);
		double x_temp;
		double y_temp;

		x_temp = near_node.x_config + x_increment;
		y_temp = near_node.y_config + y_increment;
		
		Node temp_node= new Node();
		temp_node.x_config = x_temp;
		temp_node.y_config = y_temp;

		if (!isConfigValid(temp_node, object))
		{
			status = Status.TRAPPED;
			return status;
		}
		else if (dist_norm_incr < dist_norm)
		{
			//System.out.println("****   HERE  OR *****");
			status = Status.ADVANCE;
			new_node.x_config = x_temp;
			new_node.y_config = y_temp;
		}
		else
		{
			//System.out.println("****   HERE17   *****");
			new_node.x_config = r_node.x_config;
			new_node.y_config = r_node.y_config;
			status = Status.REACHED;
		}
		return status;
	}
	
	void addConfigToTree(Tree tree, Node near_node, Node new_node)  //keeps track of the number of node and predecessor index etc.
	{
		new_node.index = tree.node.size();
		new_node.predecessor_index = near_node.index;

		tree.node.add(new_node);
		tree.num_nodes = tree.node.size();
	}
	
	Status extendTree(Tree tree, Node rand_node, Node near_node, Node new_node, double max_step_size, SquareObject object) //adds node to the tree
	{
		Status tree_state;
		findNearestNeighbour(tree, rand_node, near_node);
		tree_state = generateRandomNode(near_node, rand_node, new_node, max_step_size, object);

		if (tree_state != Status.TRAPPED)
			addConfigToTree(tree, near_node, new_node);

		return tree_state;
	}
	
	boolean solveQuery(int max_iter, double max_step_size, double start_Xconfig, double start_Yconfig, double goal_Xconfig, double goal_Yconfig, SquareObject object) //main cheez
	{
		while (!setStartGoalConfig(start_Xconfig, start_Yconfig, goal_Xconfig, goal_Yconfig, object)) {
			
			//System.out.println("****   HERE   *****");
		}
	
		Node rand= new Node();
		Node near= new Node();
		Node new_n= new Node();

		near.index = 0;
		near.predecessor_index = -1;
		new_n.index = 0;
		near.predecessor_index = -1;


		Status returned;
		boolean path_found = false;

		for (int i = 0; i < max_iter; i++)
		{
			System.out.println("Iteration number: "+i); 
			//System.out.println("****   HERE2   *****");
			getRandomConfig(rand, object);
			returned = extendTree(Tstart, rand, near, new_n, max_step_size, object);
			
			if (returned == Status.REACHED)
			{
				if (rand.x_config - goal_node.x_config < 0.10 &&
					rand.y_config - goal_node.y_config < 0.10)
				{
					path_found = true;					
				//	System.out.println("****   HERE   *****");
					
				}
				if (path_found)
				{
					System.out.printf("Path Found: goal to start!!");
					Node start_connect =new Node();
					start_connect=Tstart.node.get(Tstart.node.size()-1);
					//getSolutionPath(Tstart, start_connect);

					Tstart.node.clear();
					Tstart.num_nodes = 0;

					return true;
				}
			}
		}
		//System.out.println("****   HERE2   *****");
		
		Tstart.node.clear();
		Tstart.num_nodes = 0;

		System.out.printf("NO PATH FOUND!!");
		
		return false;
		
		
	}
	
	
	
	/*void getSolutionPath(Tree tree, Node node_connect)
	{

		Vector<Node> solution_path=new Vector<Node>();

		
		Node node_in=new Node();
		Node current = new Node();
		current=node_connect;
		
		while (current.index != 0)
		{
			
			node_in.x_config = current.x_config;
			node_in.y_config = current.y_config;
			solution_path.add(0,node_in);
			Node[] tNode= tree.node.toArray(new Node [tree.node.size()]);   
			current = tNode[current.predecessor_index];
		}

		node_in.x_config = current.x_config;
		node_in.y_config = current.y_config;
		//path_config[0] = current.x_config;
		//path_config[1] = current.y_config;
		solution_path.add(0,node_in);

		path = solution_path;

	}
	*/
	
}


class SquareObject {
	private double x_cood;
	private double y_cood;
	private double length;
	
	SquareObject(double len, double xCood, double yCood){
		x_cood=xCood;
		y_cood=yCood;
		length=len;
	}
	
	public double getX_cood()
	{
		return x_cood;
	}

	public double getY_cood()
	{
		return y_cood;
	}

	public double getLength()
	{
		return length;
	}
}





