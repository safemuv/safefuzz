package atlascollectiveintgenerator;

import atlasdsl.loader.*;
import atlascollectiveint.api.*;
import atlassharedclasses.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;

import java.util.Map.Entry;
import java.util.Optional;

import activemq.portmapping.PortMappings;
import atlasdsl.*;

public class CollectiveInt {
	private Mission mission;
	
	protected double timeNow = 0.0;
	
	private CollectiveIntActiveMQConsumer consumer;
	protected HashMap<String, CollectiveIntActiveMQProducer> moosProducers = new LinkedHashMap<String, CollectiveIntActiveMQProducer>();
	protected HashMap<String, Timer> timers = new LinkedHashMap<String,Timer>();
	
	protected void handleMessage(ATLASSharedResult a) {
		
		if (a.getContentsClass() == ATLASTimeUpdate.class) {
			Optional<ATLASTimeUpdate> a_o = a.getATLASTimeUpdate();
			if (a_o.isPresent()) {
				double newTime = a_o.get().getTime();
				updateTime(newTime);
			}
		}
	}
	
	public void checkTimers() {
		// Use iterator instead of for as we may to change the collection
		// while iterating over it
		Iterator<Entry<String, Timer>> it = timers.entrySet().iterator();
	    while (it.hasNext()) {
	        Timer t = it.next().getValue();
			if (t.isReady(timeNow)) {
				t.performAction();
			}
	        
			if (t.shouldRemove(timeNow)) {
				it.remove();
			}
	    }
	}
	
	public void startCI() {
		System.out.print("startCI beginning...");
		consumer.run();
	}
	
	protected void updateTime(double newTime) {
		if (timeNow < newTime) {
			timeNow = newTime;
			//System.out.println("time = " + timeNow + "-" + timers.size() + " timers registered");
		}
	}

	public void init() {
		System.out.println();
		DSLLoader l = new StubDSLLoader();
		mission = l.loadMission();
		
		API.setCIReference(this);
		
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
			API.registerNewProducer(name, p);
			p.setupConnection();
		}
	}
	
	public void registerTimer(String timerName, Timer t) {
		timers.put(timerName, t);
	}
}