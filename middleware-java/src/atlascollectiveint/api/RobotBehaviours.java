package atlascollectiveint.api;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import atlascollectiveintgenerator.CollectiveIntActiveMQProducer;
import atlassharedclasses.*;

public class RobotBehaviours {
	private static CollectiveIntActiveMQProducer prod;
	
	private static List<Point> translateRegionToCoordsList(Region r, double stepSize) {
		List<Point> coords = new ArrayList<Point>();
		double left = r.left();
		double right = r.right();
		double top = r.top();
		double bottom = r.bottom();
		
		for (double y = bottom; y < top; y+=2*stepSize) {
			coords.add(new Point(left,y));
			coords.add(new Point(right,y));
			coords.add(new Point(right,y+stepSize));
			coords.add(new Point(left,y+stepSize));
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
		
	public static void setSweepRegion(String robotName, Region r, double stepSize) {
		Double endTime = 1000000.0;

		List<Point> coords = translateRegionToCoordsList(r, stepSize);
		String polyUpdate = "polygon=" + pointListToPolyString(coords);
		// TODO: translate the region encoding into MOOS variable update
		// need to get a reference to the ActiveMQ producer here from somewhere
		prod.sendMOOSUpdate(robotName, (endTime.toString() + "|UP_LOITER=" + polyUpdate));
	}
	
	public static void setSweepAroundPoint(String robotName, Point p, double size, double stepSize) {
		// TODO: construct a region then use setSweepRegion
		Region r = Region.squareAroundPoint(p, size);
		setSweepRegion(robotName, r, stepSize);
	}
	
	public static void setProducer(CollectiveIntActiveMQProducer producer) {
		prod = producer;
	}
}
