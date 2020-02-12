package atlasdsl.faults;
import atlasdsl.*;
import middleware.core.ATLASCore;

public class MotionFault extends ComponentImpact {
	private Robot affectedRobot;
	private String affectedProperty;
	private String newValue;
	
	public MotionFault(Robot r, String affectedProperty, String newValue) {
		this.affectedRobot = r;
		this.affectedProperty = affectedProperty;
		this.newValue = newValue;
	}
	
	public Object applyImpact(Object orig) {
		return orig;
	}

	public void immediateEffects(ATLASCore core) {
		// TODO: this assumes a MOOS effect with the same name
		// as the main effect
		
		// Somehow need some translation here
		core.sendToCARS(affectedRobot, affectedProperty, newValue);
	}
}
