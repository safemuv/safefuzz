package carsspecific.moos.moosmapping.user;

import carsspecific.moos.moosmapping.HelmBehaviourAvoidance;
import carsspecific.moos.moosmapping.MOOSProcess;

public class HelmBehaviourAvoidance_V19 extends HelmBehaviourAvoidance {
	private static final int DEFAULT_PRIORITY_FOR_AVOIDANCE = 200;
	
	public HelmBehaviourAvoidance_V19(MOOSProcess parent, String vehicleName, int priority) {
		super(parent, vehicleName, priority);
		setName("BHV_AvdColregsV19");
		setProperty("giveway_bow_dist", 10);
		setProperty("use_refinery", true);
	}
	
	public HelmBehaviourAvoidance_V19(MOOSProcess parent, String vehicleName) {
		this(parent, vehicleName, DEFAULT_PRIORITY_FOR_AVOIDANCE);
	}
}
