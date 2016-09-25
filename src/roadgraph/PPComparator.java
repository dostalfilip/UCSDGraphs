package roadgraph;

import java.util.Comparator;

public class PPComparator implements Comparator<PriorityGeoPoint> {

	@Override
	public int compare(PriorityGeoPoint a1, PriorityGeoPoint a2) {
		if(a1.getPriority() > a2.getPriority()){
		return 1;
		}
		else if(a1.getPriority() < a2.getPriority()){
			return -1;
			}
		return 0;
	}

}
