package moosmapping;

public class HelmBehaviourConstantDepth extends MOOSBehaviour {
	public HelmBehaviourConstantDepth(MOOSProcess parent) {
		super("BHV_ConstantHeading", parent);
		setProperty("name", "constdepth");
		setProperty("pwt", 100);
		setProperty("condition", "MODE==CONSTDEPTH");
		setProperty("updates", "UP_DEPTH");
		setProperty("depth", 0.0);
		setProperty("duration", "no-time-limit");
	}
}
