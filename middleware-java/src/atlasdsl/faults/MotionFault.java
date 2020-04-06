package atlasdsl.faults;
import atlasdsl.*;
import middleware.core.ATLASCore;

public class MotionFault extends ComponentImpact {
	private String affectedProperty;
	private String newValue;
	
	public MotionFault(Component c, String affectedProperty, String newValue) {
		super(c);
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
		// TODO: sendTOCARS should be using generic Component here - not a robot - lookup host robot?
		core.sendToCARS((Robot)affectedComponent, affectedProperty, newValue);
	}
}
