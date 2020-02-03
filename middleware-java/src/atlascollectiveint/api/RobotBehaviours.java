package atlascollectiveint.api;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import atlassharedclasses.*;

public class RobotBehaviours {
	private static List<Point> translateRegionToCoordsList(Region r) {
		List<Point> coords = new ArrayList<Point>();
		
		double left = r.left();
		double right = r.right();
		double top = r.top();
		double bottom = r.bottom();
		
		double step = 5;
		
		for (double y = bottom; y < top; y+=2*step) {
			coords.add(new Point(left,y));
			coords.add(new Point(right,y));
			coords.add(new Point(right,y+step));
			coords.add(new Point(left,y+step));
		}
		Collections.reverse(coords);
		return coords;
	}
	
	private static String pointListToPolyString(List<Point> coords) {
		StringBuilder b = new StringBuilder();
		for (Point c : coords) 
			b.append(c.toStringBareCSV());
		return b.toString();
	}
		
	public static void setSweepRegion(String robotName, Region r) {
		List<Point> coords = translateRegionToCoordsList(r);
		String polyUpdate = "polygon=" + pointListToPolyString(coords);
		// TODO: translate the region encoding into MOOS variable update
		// need to get a reference to the ActiveMQ producer here from somewhere
		CollectiveIntActiveMQProducer.sendMOOSUpdate(robotName, "UP_LOITER=" + polyUpdate);
	}
	
	public static void setSweepAroundPoint(String robotName, Point p, double radius) {
		// TODO: construct a region then use setSweepRegion
		Region r = Region.sqaureAroundPoint(p, radius);
		
		
	}
}