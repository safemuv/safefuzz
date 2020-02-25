package atlascollectiveintgenerator;

import atlasdsl.loader.*;
import atlascollectiveint.api.*;
import atlassharedclasses.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Optional;

import activemq.portmapping.PortMappings;
import atlasdsl.*;

public class CollectiveInt {
	private Mission mission;
	
	protected double time = 0.0;
	
	private CollectiveIntActiveMQConsumer consumer;
	protected HashMap<String, CollectiveIntActiveMQProducer> moosProducers = new LinkedHashMap<String, CollectiveIntActiveMQProducer>();
	
	protected void handleMessage(ATLASSharedResult a) {
		if (a.getContentsClass() == ATLASTimeUpdate.class) {
			Optional<ATLASTimeUpdate> a_o = a.getATLASTimeUpdate();
			if (a_o.isPresent()) {
				double newTime = a_o.get().getTime();
				updateTime(newTime);
			}
		}
	}
	
	public void startCI() {
		System.out.print("startCI beginning...");
		consumer.run();
	}
	
	protected void updateTime(double newTime) {
		if (time < newTime) {
			time = newTime;
			// TODO: logging of the time update messages here
			System.out.println("time = " + time);
		}
	}

	public void init() {
		System.out.println();
		DSLLoader l = new StubDSLLoader();
		mission = l.loadMission();
		// TODO: fix, this port is hardcoded to just listen to shoreside
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