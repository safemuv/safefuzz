package middleware.core;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import javax.jms.JMSException;

import com.fasterxml.jackson.core.JsonProcessingException;

import activemq.portmapping.PortMappings;

import atlasdsl.*;
import atlassharedclasses.*;

public class MOOSEventQueue extends ATLASEventQueue<MOOSEvent> {

	private static final boolean DEBUG_PRINT_DESERIALISED_MSGS = false;
	
	private static final long serialVersionUID = 1L;
	private Pattern nodeReportScanner;
	private Pattern detectionScanner;
	private Mission mission;
	private List<ActiveMQConsumer> activeConsumers = new ArrayList<ActiveMQConsumer>();
	private ActiveMQProducer outputToCI;

	public MOOSEventQueue(ATLASCore core, Mission mission, int queueCapacity) {
		super(core,queueCapacity, '.');
		this.mission = mission;
		nodeReportScanner = Pattern.compile("NAME=([^,]+),X=([^,]+),Y=([^,]+)");
		detectionScanner = Pattern.compile("x=([^,]+),y=([^,]+),label=([^,]+),vname=([^,]+)");
		
		atlasOMapper = new ATLASObjectMapper();
		outputToCI = core.getCIProducer();
	}	
	
	public void run() {
		super.run();
	}
	
	public void registerAfter() {
		
	}

	public void setup() {
		// Setup the ActiveMQ connection for the simulation side
		
		// Setup MOOS-side interface for the middleware
		for (Robot r : mission.getAllRobots()) {
			// TODO: either this class will be custom-generated, or 
			// various generated hooks with be added at code generation time
			// TODO: change to ActiveMQRobotConsumer?
			String robotName = r.getName();
			ActiveMQConsumer consumer = new ActiveMQConsumer(robotName, (PortMappings.portForMOOSWatch(robotName)), this);
			activeConsumers.add(consumer);
			startThread(consumer, false);
		}
		
		for (Computer c : mission.getAllComputers()) {
			// TODO: either this class will be custom-generated, or 
			// various generated hooks with be added at code generation time
			// TODO: change to ActiveMQRobotConsumer?
			ActiveMQConsumer consumer = new ActiveMQConsumer(c.getName(), (PortMappings.portForMOOSWatch(c.getName())), this);
			activeConsumers.add(consumer);
			startThread(consumer, false);
		}
	}
	
	// TODO: This has to be auto-generated to produce decoders for the specific messages
	// once the test code below is working
	public void handleEvent(MOOSEvent e) {
		if (e instanceof MOOSVariableUpdate) {
			MOOSVariableUpdate mup = (MOOSVariableUpdate)e;
			
			if (mup.keyMatches("DB_UPTIME")) {
				Double time = Double.valueOf(mup.getValue());
				try {
					// Update the core's internal time 
					core.updateTime(time);
					// If no causality exception, create a time update notification
					// and send it on to the CI
					ATLASTimeUpdate tup = new ATLASTimeUpdate(time);
					String msg = atlasOMapper.serialise(tup);
					outputToCI.sendMessage(msg);
					
					
				} catch (CausalityException e1) {
					// If there is a causality error, it may come from merely 
					// node reports being received out of order from different
					// vehicles. It is not necessarily fatal.
					// Log it anyway
					System.out.println("Causality exception - time diff " + e1.timeDiff());
				} catch (JsonProcessingException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				} catch (JMSException e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
				}
			}
			
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
					
					// TODO: for now, assume the position sensor updates are provided directly 
					// by base position
					GPSPositionReading gps = new GPSPositionReading(x,y, entityName);
					
					try {
						String msg = atlasOMapper.serialise(gps);
						
						if (DEBUG_PRINT_DESERIALISED_MSGS) {
							System.out.println("DEBUG: serialised message " + msg);
						}
						outputToCI.sendMessage(msg);	
					} catch (JsonProcessingException e1) {
						// TODO Auto-generated catch block
						e1.printStackTrace();
					} catch (JMSException e1) {
						// TODO Auto-generated catch block
						e1.printStackTrace();
					}
					
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
				Matcher mtch = detectionScanner.matcher(val);
				if (mtch.find()) {
					double x = Double.parseDouble(mtch.group(1));
					double y = Double.parseDouble(mtch.group(2));
					Integer objectID = Integer.parseInt(mtch.group(3));
					String vname = mtch.group(4);
					Robot r = mission.getRobot(vname);
					Optional<EnvironmentalObject> eo = mission.getEnvironmentalObject(objectID);
					if (r == null) {
						System.out.println("DEBUG: robot not found for detection report: " + vname + "full update: " + val);	
					} else {
						if (!eo.isPresent()) {
							System.out.println("DEBUG: detected object not registered in environment " + objectID + "full update " + val);	
						} else {
							System.out.println("INFO: sending out sensor detection");
							Message m = new Message("SONAR_DETECTION_" + vname.toUpperCase());
							SensorDetection d = new SensorDetection(m, SensorType.SONAR);
							d.setField("location", new Point(x,y));
							d.setField("robotName", vname);
							d.setField("objectID", objectID);						
							core.notifySensorDetection(d);
							
							try {
								String msg = atlasOMapper.serialise(d);
								
								if (DEBUG_PRINT_DESERIALISED_MSGS) {
									System.out.println("DEBUG: serialised message " + msg);
								}
								outputToCI.sendMessage(msg);
							} catch (JMSException e1) {
								// TODO Auto-generated catch block
								e1.printStackTrace();
							} catch (JsonProcessingException e1) {
								// TODO Auto-generated catch block
								e1.printStackTrace();
							}
						}
					}
				}
			}
		} else {
			// TODO: log a warning of an incompatible message
			System.out.println("DEBUG: MOOSEventQueue.handleEvent got incompatible object of class " + e.getClass());
		}
	}
}
