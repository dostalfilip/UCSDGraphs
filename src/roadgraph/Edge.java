package roadgraph;

import geography.GeographicPoint;

public class Edge {
	private GeographicPoint from;
	private GeographicPoint to;
	private String roadName;
	private String roadType;
	private double length;
	
	
	public Edge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length){
		setFrom(from);
		setTo(to);
		setRoadName(roadName);
		setRoadType(roadType);
		setLength(length);
	}


	public GeographicPoint getFrom() {
		return from;
	}


	public void setFrom(GeographicPoint from) {
		this.from = from;
	}


	public GeographicPoint getTo() {
		return to;
	}


	public void setTo(GeographicPoint to) {
		this.to = to;
	}


	public String getRoadName() {
		return roadName;
	}


	public void setRoadName(String roadName) {
		this.roadName = roadName;
	}


	public String getRoadType() {
		return roadType;
	}


	public void setRoadType(String roadType) {
		this.roadType = roadType;
	}


	public double getLength() {
		return length;
	}


	public void setLength(double length) {
		this.length = length;
	}

	//OVERWRITE
	
	public String toString(){
		String s = "Edge summary:\n";
		s+="Border1: " + getFrom();
		s+=" Border2: " + getTo();
		s+="\nRoadmName: " + getRoadName();
		s+="RoadmType: " + getRoadType();
		s+="RoadLengh: " + getLength();
		s+="\n \n";
		return s;
	}
	


	

}
