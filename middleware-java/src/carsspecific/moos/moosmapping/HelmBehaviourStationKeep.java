package carsspecific.moos.moosmapping;

import atlassharedclasses.Point;

public class HelmBehaviourStationKeep extends MOOSBehaviour {
	public HelmBehaviourStationKeep(MOOSProcess parent, Point stationPoint) {
		super("BHV_StationKeep", parent);
		String pointString = stationPoint.getX() + "," + stationPoint.getY();
		
		// TODO: pull out these radii properties to the constructor
		setProperty("name", "station-keep");
		//setProperty("pwt", 100);
		setProperty("condition", "STATION_KEEP=true");
		setProperty("updates", "UP_STATIONKEEP");
		setProperty("station_pt", pointString);
		setProperty("inner_radius", 5);
		setProperty("outer_radius", 10);
		setProperty("outer_speed", 1.0);
		setProperty("transit_speed", 1.3);
		setProperty("swing_time", 7);
		setProperty("hibernation_radius", 25);
		setProperty("visual_hints", "vertex_size=0, edge_color=blue");
	}
}
