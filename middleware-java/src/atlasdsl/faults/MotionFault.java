package atlasdsl.faults;

import atlasdsl.*;
import middleware.core.ATLASCore;

public class MotionFault extends ComponentImpact {
	private String affectedProperty;
	private String newValue;
	// TODO: Need to store the original value - this requires the middleware to know
	// from MOOS the original value!
	private String originalValue = "speed=1.0";
	
	public MotionFault(Component c, String affectedProperty, String newValue) throws InvalidComponentType {
		super(c);
		if (c instanceof MotionSource) {
			this.affectedProperty = affectedProperty;
			this.newValue = newValue;
		} else {
			throw new InvalidComponentType(c, c.getClass(), MotionSource.class);
		}
	}
	
	public String getNewValue() {
		return newValue;
	}
	
	public Object applyImpact(Object orig) {
		return orig;
	}
	
	// Custom implementation for a heading fault
	public void headingImmediateEffect(Robot relevantVehicle, ATLASCore core) {	
		core.sendToCARS(relevantVehicle, "CONSTHEADING", "true");
		core.sendToCARS(relevantVehicle, "LOITER", "false");
	}
	
	public void headingCompletionEffect(Robot relevantVehicle, ATLASCore core) {	
		core.sendToCARS(relevantVehicle, "CONSTHEADING", "false");
		core.sendToCARS(relevantVehicle, "LOITER", "true");
	}

	public void immediateEffects(ATLASCore core) {
		// TODO: this assumes a MOOS effect with the same name
		// as the main effect
		// Need MOOS translation here
		
		Robot relevantVehicle = (Robot)affectedComponent.getParent();
		if (affectedProperty.contains("HEADING")) {
			headingImmediateEffect(relevantVehicle, core);
		}
		
		// TODO: store the original value here
		core.sendToCARS(relevantVehicle, affectedProperty, newValue);
	}
	
	public void completionEffects(ATLASCore core) {
		Robot relevantVehicle = (Robot)affectedComponent.getParent();
		if (affectedProperty.contains("HEADING")) {
			headingCompletionEffect(relevantVehicle, core);
		}
		
		System.out.println("Motion fault completion effect");
		core.sendToCARS(relevantVehicle, affectedProperty, originalValue);
	}

	public void _overrideSpeed(double speedOverride) {
		newValue = "speed=" + Double.toString(speedOverride);
		System.out.println("WARNING: using temporary hack to override speed only for experiments: " + newValue);
	}
}
