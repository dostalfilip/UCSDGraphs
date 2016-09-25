package roadgraph;

import java.util.Comparator;

import geography.GeographicPoint;

public class PriorityGeoPoint {
	
	private double priority; // priority = distance from start
	private GeographicPoint point;
	

	
	public PriorityGeoPoint(GeographicPoint point) {
		this.setPoint(point);
		this.priority = -1;
		// TODO Auto-generated constructor stub
	}
	
	public PriorityGeoPoint(GeographicPoint point, double priority) {
		this.setPoint(point);
		this.priority = priority;
		// TODO Auto-generated constructor stub
	}
	
	
	

	public double getPriority() {
		return priority;
	}
	
	public void setPriority(double priority) {
		this.priority = priority;
	}
	


	public GeographicPoint getPoint() {
		return point;
	}

	public void setPoint(GeographicPoint point) {
		this.point = point;
	}

	
    public String toString()
    {
    	return Double.toString(getPriority());
    }
	

}
