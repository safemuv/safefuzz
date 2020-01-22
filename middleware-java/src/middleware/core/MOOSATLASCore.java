package middleware.core;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

import atlasdsl.*;
import middleware.core.MOOSVariableUpdate;
import middleware.gui.GUITest;

public class MOOSATLASCore extends ATLASCore {
	private int MOOS_QUEUE_CAPACITY = 100;
	private Pattern nodeReportScanner;
	private GUITest gui;
	
	public MOOSATLASCore(Mission mission) {
		super(mission);
		carsIncoming = new MOOSEventQueue(MOOS_QUEUE_CAPACITY);
		setupForCARSEvents();
		gui = new GUITest(mission);
	}
	
	public void setupForCARSEvents() {
		// TODO: move these scanners out into general utility library?
		//nodeReportScanner = Pattern.compile("X=[+-]?([0-9]*[.])?[0-9]+,Y=[+-]?([0-9]*[.])?[0-9]+");
		nodeReportScanner = Pattern.compile("X=(.+),Y=(.+)(,?.*)");
	}
	
	// TODO: This has to be auto-generated to produce a decoders for the specific messages
	// once the test code below is working
	public void handleCARSEvent(CARSEvent e) {
		System.out.println("DEBUG: handleCARSEvent called");
		// Need to first check it's a MOOS event
		if (e instanceof MOOSVariableUpdate) {
			MOOSVariableUpdate mup = (MOOSVariableUpdate)e;
			if (mup.keyStartMatches("NODE_REPORT")) {
				// Extract the x,y coordinates from the contents via regex
				String val = mup.getValue();
				Matcher m = nodeReportScanner.matcher(val);
				if (m.find()) {
					System.out.println("0=" + m.group(1) + ",1=" + m.group(2));
					// TODO: exception checks for malformed value here
					double x = Double.valueOf(m.group(1));
					double y = Double.valueOf(m.group(2));
					// Find the corresponding robot object and update its location parameters
					Robot r = mission.getRobot(mup.getVehicleName());
					if (r != null) {
						r.setPointComponentProperty("location", new Point(x,y));
					} else {
						System.out.println("Robot " + mup.getVehicleName() + " not found");
					}
				}
			}
			
			if (mup.keyStartMatches("UHZ_DETECTION_REPORT")) {
				
			}
		} else {
			// TODO: log a warning of an incompatible message
			System.out.println("DEBUG: handleCARSEvent got incompatible object of class " + e.getClass());
		}
	}
}
 