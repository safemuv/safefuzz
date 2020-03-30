package atlascollectiveint.api;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import atlascollectiveintgenerator.CollectiveInt;
import atlascollectiveintgenerator.CollectiveIntActiveMQProducer;
import atlassharedclasses.*;

public class API {
	// TODO: to add to the API - speed change requests from the CI
	
	private static boolean DEBUG_POLYGON_COORDS = false;
	
	private static HashMap<String,CollectiveIntActiveMQProducer> producers = new HashMap<String,CollectiveIntActiveMQProducer>();
	private static CollectiveInt ci;
	
	private static List<Point> translateRegionToCoordsList(Region r, double stepSize) {
		List<Point> coords = new ArrayList<Point>();
		double left = r.left();
		double right = r.right();
		double top = r.top();
		double bottom = r.bottom();
		
		for (double y = bottom; y < top; y+=2*stepSize) {
			coords.add(new Point(left,y));
			coords.add(new Point(right,y));
			// TODO: check we're not exceeding region boundary here - is 2X incorrect?
			coords.add(new Point(right,y+stepSize));
			coords.add(new Point(left,y+stepSize));
		}
		Collections.reverse(coords);
		return coords;
	}
	
	private static CollectiveIntActiveMQProducer getProducerFor(String robotName) {
		return producers.get(robotName);
	}
		
	public static void setPatrolAroundRegion(String robotName, Region r, double stepSize) {
		List<Point> coords = translateRegionToCoordsList(r, stepSize);
		if (DEBUG_POLYGON_COORDS) {
			System.out.println("coords=" + coords.toString());
		}
		BehaviourCommand cmd = new SetCoordinates(coords);
		CollectiveIntActiveMQProducer prod = getProducerFor(robotName);
		prod.send(cmd, robotName);
	}
	
	public static void setSweepAroundPoint(String robotName, Point p, double size, double stepSize) {
		Region r = Region.squareAroundPoint(p, size);
		setPatrolAroundRegion(robotName, r, stepSize);
	}
	
	public static void registerNewProducer(String communityName, CollectiveIntActiveMQProducer producer) {
		producers.put(communityName, producer);
	}
	
	public static void setCIReference(CollectiveInt ciRef) {
		ci = ciRef;
	}

	public static void registerTimer(String timerName, Timer timer) {
		ci.registerTimer(timerName, timer);
	}
}
