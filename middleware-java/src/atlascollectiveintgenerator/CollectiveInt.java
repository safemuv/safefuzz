package atlascollectiveintgenerator;

import atlasdsl.loader.*;
import atlascollectiveint.api.*;
import atlassharedclasses.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;

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
		
		List<String> producerNames = new ArrayList<String>();
		
		for (Robot r : mission.getAllRobots()) {
			producerNames.add(r.getName());
		}
		
		for (Computer c : mission.getAllComputers()) {
			producerNames.add(c.getName());
		}
		
		for (String name : producerNames) {
			CollectiveIntActiveMQProducer p = new CollectiveIntActiveMQProducer(PortMappings.portForMiddlewareFromCI(name), mission);
			moosProducers.put(name, p);
			RobotBehaviours.registerNewProducer(name, p);
			p.setupConnection();
		}
	}
}
