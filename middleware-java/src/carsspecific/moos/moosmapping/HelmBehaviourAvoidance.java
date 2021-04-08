package carsspecific.moos.moosmapping;

public class HelmBehaviourAvoidance extends MOOSBehaviour {
	private static final int DEFAULT_PRIORITY_FOR_AVOIDANCE = 200;
	
	public static enum AvoidanceBehaviourVersion {
		STANDARD,
		V19
	}
	
	public HelmBehaviourAvoidance(MOOSProcess parent, String vehicleName, int priority, AvoidanceBehaviourVersion ver) {
		// Use the ternary operator to determine the superclass call based upon the avoidance version selected
		super((ver == AvoidanceBehaviourVersion.V19 ? "BHV_AvdColregsV19" : "BHV_AvoidCollision"), parent);
		setProperty("name", "avdcollision_");
		setProperty("pwt", priority);
		setProperty("condition", "(AVOID=true) and (CONSTHEADING=false)");
		setProperty("updates", "CONTACT_INFO");
		setProperty("endflag", "CONTACT_RESOLVED = $[CONTACT]");
		setProperty("templating", "spawn");
		setProperty("contact", "to-be-set");
		setProperty("on_no_contact_ok", true);
		setProperty("extrapolate", true);
		setProperty("decay", "30,60");
		setProperty("pwt_outer_dist", 50);
		setProperty("pwt_inner_dist", 20);
		setProperty("completed_dist", 75);
		setProperty("min_util_cpa_dist", 3);
		setProperty("max_util_cpa_dist", 7);
		setProperty("pwt_grade", "linear");
		setProperty("bearing_line_config", "white:0, green:0.65, yellow:0.8, red:1.0");
		
		// Set additional properties if using V19 for it
		if (ver == AvoidanceBehaviourVersion.V19) {
			setProperty("giveway_bow_dist", 10);
			setProperty("use_refinery", true);
		}
	}
		
	public HelmBehaviourAvoidance(MOOSProcess parent, String vehicleName, int priority) {
		this(parent, vehicleName, priority, AvoidanceBehaviourVersion.STANDARD);
	}
	
	public HelmBehaviourAvoidance(MOOSProcess parent, String vehicleName) {
		this(parent, vehicleName, DEFAULT_PRIORITY_FOR_AVOIDANCE, AvoidanceBehaviourVersion.STANDARD);
	}
	
	public HelmBehaviourAvoidance(MOOSProcess parent, String vehicleName, AvoidanceBehaviourVersion ver) {
		this(parent, vehicleName, DEFAULT_PRIORITY_FOR_AVOIDANCE, ver);
	}
}
