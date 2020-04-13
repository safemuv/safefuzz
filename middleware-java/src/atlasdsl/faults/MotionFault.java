package atlasdsl.faults;
import atlasdsl.*;
import middleware.core.ATLASCore;

public class MotionFault extends ComponentImpact {
	private String affectedProperty;
	private String newValue;
	// This is to store the preserved value
	private String originalValue;
	
	public MotionFault(Component c, String affectedProperty, String newValue) throws InvalidComponentType {
		super(c);
		if (c instanceof MotionSource) {
			this.affectedProperty = affectedProperty;
			this.newValue = newValue;
		} else {
			throw new InvalidComponentType(c, c.getClass(), MotionSource.class);
		}
	}
	
	public Object applyImpact(Object orig) {
		return orig;
	}

	public void immediateEffects(ATLASCore core) {
		// TODO: this assumes a MOOS effect with the same name
		// as the main effect
		// Need MOOS translation here
		
		// TODO: store the original value here
		Robot relevantVehicle = (Robot)affectedComponent.getParent();
		core.sendToCARS(relevantVehicle, affectedProperty, newValue);
	}
	
	public void completionEffect(ATLASCore core) {
		Robot relevantVehicle = (Robot)affectedComponent.getParent();
		core.sendToCARS(relevantVehicle, affectedProperty, originalValue);
	}
}
