package carsspecific.ros.carsqueue;

import atlasdsl.*;
import carsspecific.moos.carsqueue.MOOSEvent;
import middleware.core.*;

public class ROSEventQueue extends CARSLinkEventQueue<MOOSEvent> {

	private static final boolean DEBUG_PRINT_DESERIALISED_MSGS = false;
	private static final boolean ALWAYS_REQUEST_CLASSIFICATION = true;

	private static final long serialVersionUID = 1L;
	// TODO: shift into parent class
	private ActiveMQProducer outputToCI;
	// 

	public ROSEventQueue(ATLASCore core, Mission mission, int queueCapacity) {
		super(core, queueCapacity, '.');

	}

	public void run() {
		super.run();
	}

	public void registerAfter() {

	}

	public void setup() {
		// Set up the Websocket connection to ROS
	}

	public void handleEventSpecifically(MOOSEvent e) {

	}
}
