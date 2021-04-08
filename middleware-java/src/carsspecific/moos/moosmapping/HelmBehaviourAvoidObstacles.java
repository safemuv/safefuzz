package carsspecific.moos.moosmapping;

public class HelmBehaviourAvoidObstacles extends MOOSBehaviour {
	private static final int DEFAULT_PRIORITY_FOR_OBSTACLE_AVOIDANCE = 500;
		
	public HelmBehaviourAvoidObstacles(MOOSProcess parent, String vehicleName, int priority) {
		super("BHV_AvoidObstacle", parent);
		setProperty("name", "avd_obstacles_");
		setProperty("pwt", priority);
		setProperty("condition", "(DEPLOY = true) and (CONSTHEADING = false)");
		setProperty("templating", "spawn");
		setProperty("updates", "OBSTACLE_ALERT");
		setProperty("allowable_ttc", 5);
		setProperty("buffer_dist", 7);
		setProperty("pwt_outer_dist", 20);
		setProperty("pwt_inner_dist", 10);
		setProperty("completed_dist", 25);
	}
	
	public HelmBehaviourAvoidObstacles(MOOSProcess parent, String vehicleName) {
		this(parent, vehicleName, DEFAULT_PRIORITY_FOR_OBSTACLE_AVOIDANCE);
	}
}
