package middleware.core;

import atlasdsl.Mission;
import middleware.core.MOOSVariableUpdate;

public class MOOSATLASCore extends ATLASCore {
	private int MOOS_QUEUE_CAPACITY = 100;
	
	public MOOSATLASCore(Mission mission) {
		super(mission);
		carsIncoming = new MOOSEventQueue(MOOS_QUEUE_CAPACITY);
	}
	
	// TODO: This has to be auto-generated to produce a decoders for the specific messages
	// once the test code below is working
	public void handleCARSEvent(CARSEvent e) {
		System.out.println("DEBUG: handleCARSEvent called");
		// Need to first check it's a MOOS event
		if (e instanceof MOOSVariableUpdate) {
			MOOSVariableUpdate mup = (MOOSVariableUpdate)e;
			String eventValue = mup.getValue();
			if (mup.keyStartMatches("NODE_REPORT")) {
			// Extract the x,y coordinates from the contents
			System.out.println("debug - NODE_REPORT found for vehicle " + mup.getVehicleName() + ": " + mup.getValue());
			} 
			
			if (mup.keyStartMatches("UHZ_DETECTION_REPORT")) {
				
			}
		} else {
			// TODO: log a warning of an incompatible message
			System.out.println("DEBUG: handleCARSEvent got incompatible object of class " + e.getClass());
		}
	}
}
 