package atlascollectiveintgenerator;

import atlasdsl.loader.*;
import middleware.core.SensorDetection;
import activemq.portmapping.PortMappings;
import atlasdsl.*;

public class CollectiveInt {
	private Mission mission;
	private CollectiveIntActiveMQConsumer consumer;
	
	void handleMessage(SensorDetection d) {
		System.out.println("CollectiveInt.handleMessage called");
	}
	
	public void startCI() {
		DSLLoader l = new StubDSLLoader();
		mission = l.loadMission();
		consumer = new CollectiveIntActiveMQConsumer(PortMappings.portForCI("shoreside"), mission, this);
		// Start the CI listening over ActiveMQ
		consumer.run();
	}
}
