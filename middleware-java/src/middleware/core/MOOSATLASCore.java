package middleware.core;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

import javax.jms.JMSException;

import atlasdsl.*;
import middleware.core.MOOSVariableUpdate;
import middleware.gui.GUITest;

public class MOOSATLASCore extends ATLASCore {
	private int MOOS_QUEUE_CAPACITY = 100;
	private Pattern nodeReportScanner;
	private Pattern detectionScanner;
	
	private ActiveMQProducer outputToCI;
	
	private GUITest gui;
	
	public MOOSATLASCore(Mission mission) {
		super(mission);
		outputToCI = new ActiveMQProducer("MIDDLEWARE-TO_CI", ActiveMQProducer.QueueOrTopic.TOPIC);
		carsIncoming = new MOOSEventQueue(MOOS_QUEUE_CAPACITY);
		setupForCARSEvents();
		gui = new GUITest(mission);
	}
	
	public void setupForCARSEvents() {
		// TODO: move these scanners out into general utility library?
		nodeReportScanner = Pattern.compile("NAME=([^,]+),X=([^,]+),Y=([^,]+)");
		detectionScanner = Pattern.compile("x=([^,]+),y=([^,]+),label=([^,]+),vname=([^,+])");
	}
	
	public void afterAction() {
		gui.updateGUI();
	}
	
	// TODO: This has to be auto-generated to produce decoders for the specific messages
	// once the test code below is working
	public void handleCARSEvent(CARSEvent e) {
		// Need to first check it's a MOOS event
		if (e instanceof MOOSVariableUpdate) {
			MOOSVariableUpdate mup = (MOOSVariableUpdate)e;
			
			// Handle NODE_REPORT events
			if (mup.keyStartMatches("NODE_REPORT")) {
				// Extract the x,y coordinates from the contents via regex
				String val = mup.getValue();
				Matcher m = nodeReportScanner.matcher(val);
				if (m.find()) {
					// TODO: exception checks for malformed value here
					String entityName = m.group(1);
					double x = Double.parseDouble(m.group(2));
					double y = Double.parseDouble(m.group(3));
					// Find the corresponding robot object and update its location parameters
					Robot r = mission.getRobot(entityName);
					if (r != null) {
						r.setPointComponentProperty("location", new Point(x,y));
					} else {
						Computer c = mission.getComputer(entityName);
						if (c != null) {
							System.out.println("Robot or computer " + mup.getVehicleName() + " not found in middleware internal state");
						}
					}
				}
			}
			
			// Handle sonar detection report events
			if (mup.keyStartMatches("UHZ_DETECTION_REPORT")) {
				System.out.println("SENSOR DETECTION REPORT");
				String val = mup.getValue();
				Matcher m = detectionScanner.matcher(val);
				if (m.find()) {
					double x = Double.parseDouble(m.group(1));
					double y = Double.parseDouble(m.group(2));
					Integer label = Integer.parseInt(m.group(3));
					String vname = m.group(4);
					Robot r = mission.getRobot(vname);
					EnvironmentalObject eo = mission.getEnvironmentalObject(label);
					if (r == null) {
						System.out.println("DEBUG: robot not found for detection report: " + vname + "full update: " + val);	
					} else {
						if (eo == null) {
							System.out.println("DEBUG: detected object not registered in environment " + label + "full update " + val);	
						} else {
							System.out.println("INFO: sending out sensor detection");
							SensorDetection d = new SensorDetection(new Point(x,y),r,eo);
							try {
								outputToCI.sendMessage(d.toString());
							} catch (JMSException e1) {
								// TODO Auto-generated catch block
								e1.printStackTrace();
							} 
						}	
					}
				}
			}
		} else {
			// TODO: log a warning of an incompatible message
			System.out.println("DEBUG: handleCARSEvent got incompatible object of class " + e.getClass());
		}
	}
}
 