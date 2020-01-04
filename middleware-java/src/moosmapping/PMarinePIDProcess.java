package moosmapping;

public class PMarinePIDProcess extends MOOSProcess {
	public PMarinePIDProcess(MOOSCommunity parent) {
		super("pMarinePID", parent);
		
		setProperty("VERBOSE", true);
		setProperty("DEPTH_CONTROL", false);
		setProperty("ACTIVE_START", true);
		
		setProperty("YAW_PID_KP", 0.4);
		setProperty("YAW_PID_KD", 0.1);
		setProperty("YAW_PID_KI", 0.0);
		setProperty("YAW_PID_INTEGRAL_LIMIT", 0.07);
		
		setProperty("SPEED_PID_KP", 0.4);
		setProperty("SPEED_PID_KD", 0.1);
		setProperty("SPEED_PID_KI", 0.0);
		setProperty("SPEED_PID_INTEGRAL_LIMIT", 0.07);
		
		setProperty("MAXRUDDER", 100);
		setProperty("MAXTHRUST", 100);
		setProperty("SPEED_FACTOR", 20);
	}
}
