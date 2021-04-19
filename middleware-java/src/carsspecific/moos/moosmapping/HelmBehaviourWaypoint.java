package carsspecific.moos.moosmapping;

import atlassharedclasses.Point;

public class HelmBehaviourWaypoint extends MOOSBehaviour {
	public HelmBehaviourWaypoint(MOOSProcess parent, String vehicleName, Point startPos, double speed, double radius, double nm_radius) {
		super("BHV_Waypoint", parent);
		setProperty("name", "waypt");
		setProperty("condition", "(DEPLOY=true) and ((CONSTHEADING=false) and (STATION_KEEP=false))");
		setProperty("updates", "UP_WAYPOINT");
		// TODO: this will automatically end the mission when station_keep activated
		// maybe add STATION_KEEP=false in condition
		setProperty("endflag", "WAYPOINT_COMPLETE=true");
		setProperty("speed", speed);
		setProperty("radius", radius);
		setProperty("nm_radius", nm_radius);
		setProperty("points", startPos.toStringBareCSV());
		setProperty("repeat", 1);
		setProperty("lead", 8);
	}
}
