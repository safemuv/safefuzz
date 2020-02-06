package atlascollectiveintgenerator;

import atlasdsl.loader.*;
import atlascollectiveint.api.*;
import atlassharedclasses.*;
import java.util.HashMap;
import java.util.LinkedHashMap;

import activemq.portmapping.PortMappings;
import atlasdsl.*;

public class CollectiveInt {
	private Mission mission;
	private CollectiveIntActiveMQConsumer consumer;
	protected HashMap<String, CollectiveIntActiveMQProducer> moosProducers = new LinkedHashMap<String, CollectiveIntActiveMQProducer>();
	
	protected void handleMessage(ATLASSharedResult a) {
		System.out.println("CollectiveInt.handleMessage called");
	}
	
	public void startCI() {
		consumer.run();
	}

	public void init() {
		DSLLoader l = new StubDSLLoader();
		mission = l.loadMission();
		// TODO: fix, this port is hardcoded to just listen to Shoreside
		consumer = new CollectiveIntActiveMQConsumer(PortMappings.portForCI("shoreside"), mission, this);
		
		for (Robot r : mission.getAllRobots()) {
			String name = r.getName();
			// TODO: Currently the CI is sending directly to the MOOSDB's for the various elements
			CollectiveIntActiveMQProducer p = new CollectiveIntActiveMQProducer(PortMappings.portForMOOSWatch(name), mission);
			moosProducers.put(name, p);
			RobotBehaviours.registerNewProducer(name, p);
			p.setupConnection();
		}
		
		for (Computer c : mission.getAllComputers()) {
			String name = c.getName();
			// TODO: Currently the CI is sending directly to the MOOSDB's for the various elements
			CollectiveIntActiveMQProducer p = new CollectiveIntActiveMQProducer(PortMappings.portForMOOSWatch(name), mission);
			moosProducers.put(name, p);
			RobotBehaviours.registerNewProducer(name, p);
			p.setupConnection();
		}
	}
}
