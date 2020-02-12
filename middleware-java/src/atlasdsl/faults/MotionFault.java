package atlasdsl.faults;
import atlasdsl.*;
import middleware.core.ATLASCore;

public class MotionFault extends ComponentImpact {
	private Robot affectedRobot;
	
	public Object applyImpact(Object orig) {
		return orig;
	}

	public void immediateEffects(ATLASCore core) {
		// TODO: this assumes a MOOS effect
		// how to get a handle on the producer here?
		
		
	}
}
