package atlasdsl.faults;

import java.util.Optional;

import atlasdsl.*;
import middleware.core.ATLASCore;

public class MotionFault extends ComponentImpact {
	private String affectedProperty;
	private String newValue;
	private String carsVar;
	
	// TODO: Need to store the original value - this requires the middleware to know
	// from MOOS the original value!
	private String originalValue = "1.0";
	
	// TODO: push this into MOOS translation layer
	public String translatePropertyToCARSVar(String property) {
		String carsName = "UNKNOWN";
		if (property.equals("heading")) {
			carsName = "UP_HEADING";
		}
		
		if (property.equals("speed")) {
			carsName = "UP_LOITER";
		}
		return carsName;
	}
	
	public MotionFault(Component c, String affectedProperty, String newValue) throws InvalidComponentType {
		super(c);
		if (c instanceof MotionSource) {
			this.affectedProperty = affectedProperty;
			this.newValue = newValue;
			this.carsVar = translatePropertyToCARSVar(affectedProperty);
		} else {
			throw new InvalidComponentType(c, c.getClass(), MotionSource.class);
		}
	}
	
	public String getNewValue() {
		return newValue;
	}
	
	public Object applyImpact(Object orig, Optional<String> additionalData) {
		return orig;
	}
	
	public void headingImmediateEffect(Robot relevantVehicle, ATLASCore core) {	
		core.sendToCARS(relevantVehicle, "CONSTHEADING", "true");
		core.sendToCARS(relevantVehicle, "LOITER", "false");
	}
	
	public void headingCompletionEffect(Robot relevantVehicle, ATLASCore core) {	
		core.sendToCARS(relevantVehicle, "CONSTHEADING", "false");
		core.sendToCARS(relevantVehicle, "LOITER", "true");
	}

	public void immediateEffects(ATLASCore core, Optional<String> additionalData) {	
		Robot relevantVehicle = (Robot)affectedComponent.getParent();
		
		if (affectedProperty.contains("heading")) {
			headingImmediateEffect(relevantVehicle, core);
		}

		String changedValue = newValue;
		if (additionalData.isPresent()) {
			changedValue = additionalData.get();
		}
		
		String carsUpdate = affectedProperty + "=" + changedValue;
		core.sendToCARS(relevantVehicle, carsVar, carsUpdate);
	}
	
	public void completionEffects(ATLASCore core, Optional<String> additionalData) {
		Robot relevantVehicle = (Robot)affectedComponent.getParent();
		
		if (affectedProperty.contains("heading")) {
			headingCompletionEffect(relevantVehicle, core);
		}
		
		System.out.println("Motion fault completion effect");
		String carsUpdate = affectedProperty + "=" + originalValue;
		core.sendToCARS(relevantVehicle, carsVar, carsUpdate);
	}
}
