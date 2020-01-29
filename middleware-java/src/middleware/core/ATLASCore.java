package middleware.core;

import java.util.ArrayList;
import java.util.List;

import activemq.portmapping.PortMappings;
import atlasdsl.*;
import atlassharedclasses.ATLASObjectMapper;

// This code will be combined with the simulator-specific code
// during code generation
public abstract class ATLASCore {
	private List<ActiveMQConsumer> activeConsumers = new ArrayList<ActiveMQConsumer>();
	protected CARSEventQueue carsIncoming;
	protected Mission mission;
	protected ATLASObjectMapper atlasOMapper;
	
	private enum consumerSimEntity {
		ROBOT,
		SHORESIDE
	}
	
	public ATLASCore(Mission mission) {
		this.mission = mission;
		ATLASObjectMapper atlasOMapper = new ATLASObjectMapper();
	}
	
    public static void startThread(Runnable runnable, boolean daemon) {
        Thread brokerThread = new Thread(runnable);
        brokerThread.setDaemon(daemon);
        brokerThread.start();
    }
    
    public void setupAction(ActiveMQConsumer consumer, consumerSimEntity type) {
    	
    }
    
    public void afterAction() {
    	
    }
    
    // This method will have to be added to the middleware during code generation
    // TODO: fix: superclass should not have dependency upon MOOS
    public abstract void handleCARSEvent(CARSEvent e);
	
    public void runMiddleware() throws InterruptedException {
    	boolean continueLoop = true;
    	// Setup the ActiveMQ connection for the simulation side
    			
    			// Setup MOOS-side interface for the middleware
    			for (Robot r : mission.getAllRobots()) {
    				// TODO: either this class will be custom-generated, or 
    				// various generated hooks with be added at code generation time
    				// TODO: change to ActiveMQRobotConsumer?
    				String robotName = r.getName();
    				ActiveMQConsumer consumer = new ActiveMQConsumer(robotName, (PortMappings.portForMOOSWatch(robotName)), carsIncoming);
    				// This is a no-op, but will be redefined in a custom modified class
    				setupAction(consumer, consumerSimEntity.ROBOT);
    				activeConsumers.add(consumer);
    				startThread(consumer, false);
    			}
    			
    			for (Computer c : mission.getAllComputers()) {
    				// TODO: either this class will be custom-generated, or 
    				// various generated hooks with be added at code generation time
    				// TODO: change to ActiveMQRobotConsumer?
    				ActiveMQConsumer consumer = new ActiveMQConsumer(c.getName(), (PortMappings.portForMOOSWatch(c.getName())), carsIncoming);
    				// This is a no-op, but will be redefined in a custom modified class
    				setupAction(consumer, consumerSimEntity.SHORESIDE);
    				activeConsumers.add(consumer);
    				startThread(consumer, false);
    			}
    			
    			while (continueLoop) {
    				try {
    					// Check for MOOS events
    					CARSEvent e = (CARSEvent)carsIncoming.take();
    					if (e != null) {
    						handleCARSEvent(e);
    						// TODO: put logger calls here for the new event
    					}
    					
    					afterAction();
    				} catch (InterruptedException e) {
    					e.printStackTrace();
    				}	
		   			// TODO: On message from ActiveMQ for the collective intelligence side
    			    // Work out the command from the CI behaviour request
    				// TODO: On a fault generation message - FaultInstance
    			}
    }
}