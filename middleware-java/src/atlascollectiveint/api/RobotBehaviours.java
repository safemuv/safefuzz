package atlascollectiveint.api;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;

import atlascollectiveintgenerator.CollectiveIntActiveMQProducer;
import atlassharedclasses.*;

public class RobotBehaviours {
	private static boolean DEBUG_POLYGON_COORDS = false;
	
	private static HashMap<String,CollectiveIntActiveMQProducer> producers = new HashMap<String,CollectiveIntActiveMQProducer>();
	
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
		String coordsJoined = coords.stream()
				.map(p -> p.toStringBareCSV())
				.collect(Collectors.joining(":"));
		return coordsJoined;
	}
	
	private static CollectiveIntActiveMQProducer getProducerFor(String robotName) {
		return producers.get(robotName);
	}
		
	public static void setSweepRegion(String robotName, Region r, double stepSize) {
		// TODO: do something about the end time here
		// maybe a message format encoding for MOOS which doesn't require it
		Double endTime = 1000000.0;

		List<Point> coords = translateRegionToCoordsList(r, stepSize);
		String polyUpdate = "polygon=" + pointListToPolyString(coords);
		if (DEBUG_POLYGON_COORDS) {
			System.out.println("coords=" + coords.toString());
			System.out.println("polyUpdate" + polyUpdate);
		}
		
		CollectiveIntActiveMQProducer prod = getProducerFor(robotName);
		prod.sendMOOSUpdate(endTime, "UP_LOITER", polyUpdate);
	}
	
	public static void setSweepAroundPoint(String robotName, Point p, double size, double stepSize) {
		// TODO: construct a region then use setSweepRegion
		Region r = Region.squareAroundPoint(p, size);
		setSweepRegion(robotName, r, stepSize);
	}
	
	public static void registerNewProducer(String communityName, CollectiveIntActiveMQProducer producer) {
		producers.put(communityName, producer);
	}
}
