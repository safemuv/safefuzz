package middleware.core;

import java.util.HashMap;
import java.util.LinkedHashMap;

import javax.jms.JMSException;

import activemq.portmapping.PortMappings;
import atlasdsl.Mission;
import atlasdsl.Robot;

public class CIEventQueue extends ATLASEventQueue<CIEvent> {
	private static final long serialVersionUID = 1L;
	private Mission mission;
	
	private HashMap<String,ActiveMQProducer> producers = new LinkedHashMap<String,ActiveMQProducer>();

	public CIEventQueue(ATLASCore core, Mission mission, int capacity) {
		super(capacity, '@');
		this.mission = mission;
	}

	public void setup() {
		for (Robot r : mission.getAllRobots()) {
			String name = r.getName();
			ActiveMQProducer p = new ActiveMQProducer(PortMappings.portForMOOSDB(name), ActiveMQProducer.QueueOrTopic.TOPIC);
			producers.put(name, p);
		}
	}
	
	// TODO: this should be pushed into a MOOS-specific translation class?
	// This should operate upon a Message instead of a string value
	// serialise the message in some way
	private void sendMOOSUpdate(String robotName, String key, String value) {
		Double endTimeOfUpdate = 1000000.0;
		String msg = endTimeOfUpdate.toString() + "|" + key + "=" + value;
		ActiveMQProducer prod = producers.get(robotName); 
		try {
			prod.sendMessage(msg);
		} catch (JMSException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public void handleEvent(CIEvent event) {
		// Dispatch types of CI event, convert it into a low-level simulator event
		// 1) log it at the middleware side
		// 2) apply any relevant faults if they are registered in the middleware!
		// 3) convert it into a simulator specific representation
		// 4) then send to producer to be related into the real 
		// It may be a new BehaviourCommand...
	}
}
