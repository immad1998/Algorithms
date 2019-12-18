package oopProject;

//import oopa.SquareObject;

public class planner {

	public static void main(String[] args) {
		RRT_Planner planner= new RRT_Planner();
		//Node newNode=new Node();
		SquareObject object= new SquareObject(90, 50, 50);
		if (planner.solveQuery(2000, 1, 2, 2, 98, 98, object))
		{
			System.out.println("SUCCESS" );
		}
		
	}
	
	
}
