package atlascollectiveintgenerator;

import atlasdsl.loader.*;
import atlascollectiveint.api.*;
import atlassharedclasses.ATLASSharedResult;
import atlassharedclasses.SonarDetection;
import activemq.portmapping.PortMappings;
import atlasdsl.*;

public class CollectiveInt {
	private Mission mission;
	private CollectiveIntActiveMQConsumer consumer;
	protected CollectiveIntActiveMQProducer producer;
	
	protected void handleMessage(ATLASSharedResult a) {
		System.out.println("CollectiveInt.handleMessage called");
	}
	
	public void startCI() {
		DSLLoader l = new StubDSLLoader();
		mission = l.loadMission();
		// TODO: fix, this port is hardcoded to just send to Shoreside
		consumer = new CollectiveIntActiveMQConsumer(PortMappings.portForCI("shoreside"), mission, this);
		producer = new CollectiveIntActiveMQProducer(PortMappings.portForCIReverse("shoreside"), mission);
		RobotBehaviours.setProducer(producer);
		// Setup the interface over ActiveMQ
		producer.setupConnection();
		consumer.run();
		
	}

	public void init() {
		
	}
}
