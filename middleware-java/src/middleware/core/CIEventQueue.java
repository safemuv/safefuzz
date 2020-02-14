package middleware.core;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.stream.Collectors;
import javax.jms.JMSException;
import activemq.portmapping.PortMappings;

import atlasdsl.*;
import atlasdsl.faults.*;
import atlassharedclasses.*;

public class CIEventQueue extends ATLASEventQueue<CIEvent> {
	private static final long serialVersionUID = 1L;
	private Mission mission;
	private ATLASCore core;
	
	private HashMap<String,ActiveMQProducer> producers = new LinkedHashMap<String,ActiveMQProducer>();
	private HashMap<String,CIActiveMQConsumer> consumers = new LinkedHashMap<String,CIActiveMQConsumer>();

	public CIEventQueue(ATLASCore core, Mission mission, int capacity) {
		super(capacity, '@');
		this.mission = mission;
		this.core = core;
	}
	
	public void setup() {
		// Create the producers to send out converted updates to the relevant MOOSDB's
		for (Robot r : mission.getAllRobots()) {
			String name = r.getName();
			ActiveMQProducer p = new ActiveMQProducer(PortMappings.portForMOOSDB(name), ActiveMQProducer.QueueOrTopic.TOPIC);
			CIActiveMQConsumer c = new CIActiveMQConsumer(name, PortMappings.portForMiddlewareFromCI(name), this);
			producers.put(name, p);
			p.run();
			consumers.put(name,c);
			startThread(c, false);
		}
		for (Computer cp : mission.getAllComputers()) {
			String name = cp.getName();
			ActiveMQProducer p = new ActiveMQProducer(PortMappings.portForMOOSDB(name), ActiveMQProducer.QueueOrTopic.TOPIC);
			CIActiveMQConsumer c = new CIActiveMQConsumer(name, PortMappings.portForMiddlewareFromCI(name), this);
			producers.put(name, p);
			p.run();
			consumers.put(name, c);
		}
	}
	
	// TODO: this should be pushed into a MOOS-specific translation class?
	// This should operate upon a Message instead of a string value
	// serialise the message in some way
	private synchronized void sendMOOSUpdate(String robotName, String key, String value) {
		Double endTimeOfUpdate = 1000000.0;
		String msg = endTimeOfUpdate.toString() + "|" + key + "=" + value;
		ActiveMQProducer prod = producers.get(robotName); 
		try {
			prod.sendMessage(msg);
		} catch (JMSException e) {
			e.printStackTrace();
		}
	}
	
	// This is used by active faults to inject their immediate effects
	// upon the low-level CARS simulation
	public void sendToCARS(Robot r, String key, String value) {
		sendMOOSUpdate(r.getName(), key, value);
	}
	
	private static String pointListToPolyString(List<Point> coords) {
		String coordsJoined = coords.stream()
				.map(p -> p.toStringBareCSV())
				.collect(Collectors.joining(":"));
		return coordsJoined;
	}
	
	public void handleEvent(CIEvent event) {
		System.out.println("CIEventQueue.handleEvent - " + event.toString());
		// General procedure:
		// 1) log received event at the middleware side
		// 2) apply any relevant faults if they are registered in the middleware
		// 3) convert it into a simulator specific representation
		// 4) then send to MOOS producers to be relayed to MOOSDBs
		
		// TOOD: log the incoming event here
		
		BehaviourCommand ciCmd = event.getCommand();
		
		System.out.println("CIEvent behaviour command class = " + ciCmd.getClass().toString());
		
		// Dispatch types of CI event, convert it into a low-level simulator event
		// currently handle a BehaviourEvent
		if (ciCmd instanceof ActivateBehaviour) {
			ActivateBehaviour actCmd = (ActivateBehaviour)ciCmd;
			// TODO: Check for any fault impacting the command here
			System.out.println("CIEventQueue - ActivateBehaviour received");
		}
		
		if (ciCmd instanceof SetCoordinates) {
			SetCoordinates setCmd = (SetCoordinates)ciCmd;
			// put the faults that impact the coordinate processing here
			List<Point> coordinates = setCmd.getCoordinates();

			// Check for faults impacting the coordinates here
			List<FaultInstance> fs = core.activeFaultsOfClass(PointMessageChange.class);
			List<Point> modifiedCoords = coordinates;
			for (FaultInstance fi : fs) {
				Fault f = fi.getFault();
				modifiedCoords = (List<Point>)f.applyFaultToData(modifiedCoords);
			}
			
			// TODO: this contains MOOS-specific conversion here - push into the MOOS layer
			String polyUpdate = "polygon=" + pointListToPolyString(coordinates);
			String robotName = event.getRobotName();
			System.out.println("CIEventQueue - SetCoordinates received: vehicle " + robotName + " : " + polyUpdate);
			sendMOOSUpdate(robotName, "UP_LOITER", polyUpdate);
		}
	}
}
