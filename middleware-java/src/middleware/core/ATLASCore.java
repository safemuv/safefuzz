package middleware.core;

import java.util.ArrayList;
import java.util.List;

import activemq.portmapping.PortMappings;
import atlasdsl.*;
import atlassharedclasses.ATLASObjectMapper;
import middleware.gui.GUITest;

// This code will be combined with the simulator-specific code
// during code generation
public abstract class ATLASCore {
	
	protected ATLASEventQueue carsIncoming;
	protected ATLASEventQueue fromCI;
	protected ATLASEventQueue fromFaultGen;
	
	protected ActiveMQProducer outputToCI;
	
	private final int CI_QUEUE_CAPACITY = 100;
	
	protected Mission mission;
	
	private GUITest gui;
	
	protected List<ATLASEventQueue> queues = new ArrayList<ATLASEventQueue>();
	
	public ATLASCore(Mission mission) {
		this.mission = mission;
		fromCI = new CIEventQueue(this, mission, CI_QUEUE_CAPACITY);
		queues.add(fromCI);
		gui = new GUITest(mission);
	}
    
    public void afterAction() {
    	
    }
    
    public ActiveMQProducer getCIProducer() {
    	return outputToCI;
    }
	
    public void runMiddleware()  {
		for (ATLASEventQueue q : queues) {
			// The GUI needs to be updated following every event
			q.registerAfterHook(() -> gui.updateGUI());
			q.setup();
		}
		
		for (ATLASEventQueue q : queues) {
			new Thread(q).start();
		}
    }
}