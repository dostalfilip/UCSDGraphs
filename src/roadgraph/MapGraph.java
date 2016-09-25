/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */

public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	List<GeographicPoint> locations = new ArrayList<GeographicPoint>();
	List<Edge> edges = new ArrayList<Edge>();
	HashSet set = new HashSet();
	
	//declaration of variables
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return set.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		HashSet setReturn = new HashSet(set);
		
		return setReturn;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2	
		return edges.size();
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
		if(locations.add(location) && set.add(location)){ 
		//System.out.println("Proceded " + location);
		return 	true;
		}
	return false;	
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 2
		edges.add(new Edge(from,to,roadName,roadType,length));
		
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 2
	
		//Initialization
		List<GeographicPoint> path = new ArrayList<GeographicPoint>();
		
		HashMap<GeographicPoint,GeographicPoint> visited = new HashMap<GeographicPoint,GeographicPoint>();
		visited.put(start,start);
		
		LinkedBlockingQueue queue = new LinkedBlockingQueue(getNumVertices());
		queue.add(start);
		
		List<GeographicPoint> tempGeographicPoint = new ArrayList<GeographicPoint>();
		
		GeographicPoint current = new GeographicPoint(0,0);
		GeographicPoint pathtemp = new GeographicPoint(0,0);
		
		HashMap<String, GeographicPoint> startGoal = new HashMap<String, GeographicPoint>();
		startGoal.put("start", start);
		startGoal.put("goal", goal);
		
		//Begin of BFS
		while(!queue.isEmpty()){

			current = (GeographicPoint) queue.poll(); 	// take GPoint from queue
			nodeSearched.accept(current); // visualization
			
			if(current.equals(startGoal.get("goal"))){
				System.out.println("You got a GOAL POINT.");				
				//return path code
				path.add(current); // = GOAL
				pathtemp = current;// temp geographic location
					// searching in visited edges
				
				while(!(pathtemp.equals(startGoal.get("start")))){
					path.add(visited.get(pathtemp));
					pathtemp = visited.get(pathtemp);
				}
				 
				 System.out.println("RETURN PATH DONE.");
				 Collections.reverse(path);
				return path;
			}
			
			//searchEdge returns Geographicpoint of all neighbors around now checking the allready visited GoographicPoint
			
			tempGeographicPoint=searchEdge(current);
			for(GeographicPoint temp : tempGeographicPoint){
				if(!(visited.containsKey(temp))){
						visited.put(temp,current); //(next,back(root))
						queue.add(temp);
					}
				}
		}
		// if you get here, there is no path	
		return null; 
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
			GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		
		//Initialization
		
				List<GeographicPoint> path = new ArrayList<GeographicPoint>();
				
				HashMap<GeographicPoint,GeographicPoint> visited = new HashMap<GeographicPoint,GeographicPoint>();
				visited.put(start,start);
				
				HashMap<GeographicPoint,GeographicPoint> visitedfinal = new HashMap<GeographicPoint,GeographicPoint>();
				visited.put(start,start);
				//zde uklada pouze nejlepsi varianty
				
				Comparator<PriorityGeoPoint> queueComparator  = new PPComparator(); // heureka ono to vazne funguje
				
				PriorityQueue<PriorityGeoPoint> queue = new PriorityQueue < PriorityGeoPoint > (getNumVertices(),queueComparator);
				queue.add(new PriorityGeoPoint(start,0));
				
				//PriorityGeoPoint			
				PriorityGeoPoint current = new PriorityGeoPoint(new GeographicPoint(0,0));
				GeographicPoint currentGP = new GeographicPoint(0,0);
				
				double distance = 0;
				
				
				//Begin of dijkstra
				while(!queue.isEmpty()){
									

					current = queue.poll(); 	// take GPoint from queue
					currentGP = current.getPoint();
					
					visitedfinal.put(currentGP,visited.get(currentGP)); // prekopiruje hodnoty z visited
								
					distance =   distance + current.getPriority();					
					
					nodeSearched.accept(currentGP); // visualization

					
				//	System.out.println();
				//	System.out.println("Nove kolo, aktuální distance:" + current );
				//	System.out.println("Aktuální node:" + currentGP );
					
				//	for(PriorityGeoPoint j : queue){
				//		System.out.println(j + " pozice: " + j.getPoint());
				//	}
					
					
					
					
					
					if(currentGP.equals(goal)){
						System.out.println("You got a GOAL POINT.");				
						//return path code
						path.add(currentGP); // = GOAL
							// searching in visited edges
						
						while(!(currentGP.equals(start))){
							path.add(visitedfinal.get(currentGP));
							currentGP = visitedfinal.get(currentGP);
						}
						System.out.println("Celkove prohledanych node: "+ visited.size());
						System.out.println("RETURN PATH DONE.");
						Collections.reverse(path);
						 		 
						System.out.println("Trasa k cíli: ");
						for(GeographicPoint g : path){
								System.out.println(g);
							}
						return path;
					}

					for(GeographicPoint temp : searchEdge(currentGP)){
						if(!(visitedfinal.containsKey(temp))){
							
								visited.put(temp,currentGP); //(next,back(root))
								queue.add(new PriorityGeoPoint(temp,distance + currentGP.distance(temp)));
							}
						}
				}
				// if you get here, there is no path	
				return null; 
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{	
				//Initialization
						List<GeographicPoint> path = new ArrayList<GeographicPoint>();					
						HashMap<GeographicPoint,GeographicPoint> visited = new HashMap<GeographicPoint,GeographicPoint>();
						visited.put(start,start);						
						HashMap<GeographicPoint,GeographicPoint> visitedfinal = new HashMap<GeographicPoint,GeographicPoint>();
						visited.put(start,start);
						Comparator<PriorityGeoPoint> queueComparator  = new PPComparator(); // heureka ono to vazne funguje
						PriorityQueue<PriorityGeoPoint> queue = new PriorityQueue < PriorityGeoPoint > (getNumVertices(),queueComparator);
						queue.add(new PriorityGeoPoint(start,0));		
						PriorityGeoPoint current = new PriorityGeoPoint(new GeographicPoint(0,0));
						GeographicPoint currentGP = new GeographicPoint(0,0);
						double distance = 0;
						
						//Begin of aStarSearch
						while(!queue.isEmpty()){

							current = queue.poll(); 	// take GPoint from queue
							currentGP = current.getPoint();
							
							visitedfinal.put(currentGP,visited.get(currentGP)); // prekopiruje hodnoty z visited
							distance = current.getPriority();	
							
							nodeSearched.accept(currentGP); // visualization
							
							if(currentGP.equals(goal)){
								System.out.println("You got a GOAL POINT.");				
								//return path code
								path.add(currentGP); // = GOAL
									// searching in visited edges
								
								while(!(currentGP.equals(start))){
									path.add(visitedfinal.get(currentGP));
									currentGP = visitedfinal.get(currentGP);
								}
								System.out.println("Celkove prohledanych node: "+ visited.size());
								System.out.println("RETURN PATH DONE.");
								Collections.reverse(path);

								System.out.println("Trasa k cíli: ");
								for(GeographicPoint g : path){
									System.out.println(g);
									}
								return path;
							}
							for(GeographicPoint temp : searchEdge(currentGP)){
								if(!(visitedfinal.containsKey(temp))){
										visited.put(temp,currentGP); //(next,back(root))
										queue.add(new PriorityGeoPoint(temp,distance + currentGP.distance(temp) + temp.distance(goal)));
									}
								}
						}
						// if you get here, there is no path	
						return null; 
	}
	
	
	
	
	
	
	
	private List<GeographicPoint> searchEdge(GeographicPoint current){
		List<GeographicPoint> nextGeographicPoint = new ArrayList<GeographicPoint>();

		for (Edge temp : edges){
			if(current.equals(temp.getFrom()) ){
				nextGeographicPoint.add(temp.getTo());
					}
			}
		return nextGeographicPoint;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		System.out.println("DONE.");
		
		
		GeographicPoint start = new GeographicPoint(7.0, 3.0);
		GeographicPoint goal = new GeographicPoint(4.0, -1.0);

		
		//theMap.bfs(start,goal);
		//theMap.dijkstra(start,goal);
		//theMap.aStarSearch(start,goal);
		
		System.out.println("KONEC");
	
		
	}
}
