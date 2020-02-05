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
		// TODO: Currently the CI is sending directly to the shoreside MOOSDB
		producer = new CollectiveIntActiveMQProducer("FAULTS-SIM-TO-ATLAS-targ_shoreside.moos", mission);
		RobotBehaviours.setProducer(producer);
		// Setup the interface over ActiveMQ
		producer.setupConnection();
		consumer.run();
		
	}

	public void init() {
		
	}
}
