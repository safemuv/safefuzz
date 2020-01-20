package middleware.core;

import java.util.ArrayList;
import java.util.List;

import atlasdsl.*;
import atlasdsl.loader.DSLLoader;
import atlasdsl.loader.StubDSLLoader;

// This code will be combined with the simulator-specific code
// during code generation
public class ATLASCore {
	private List<ActiveMQConsumer> activeConsumers = new ArrayList<ActiveMQConsumer>();
	private Mission mission;
	
	public ATLASCore(Mission mission) {
		this.mission = mission;
	}
	
    public static void startThread(Runnable runnable, boolean daemon) {
        Thread brokerThread = new Thread(runnable);
        brokerThread.setDaemon(daemon);
        brokerThread.start();
    }
    
    public void setupAction(ActiveMQConsumer consumer) {
    	
    }
	
    public void runMiddleware() {
    	// Start the ActiveMQ connection - via ActiveMQConsumer
    			
    			// Setup MOOS-side interface for the middleware
    			for (Robot r : mission.getAllRobots()) {
    				// TODO: either this class will be custom-generated, or 
    				// various generated hooks with be added at code generation time
    				// TODO: change to ActiveMQRobotConsumer?
    				ActiveMQConsumer consumer = new ActiveMQConsumer("MIDDLEWARE-from-" + r.getName());
    				// This is a no-op, but will be redefined in a custom modified class
    				setupAction(consumer);
    				
    				activeConsumers.add(consumer);
    				startThread(consumer, false);
    			}
    			
    			// The message consumers on the MOOS side need to do the following
    			
    			// Get from the MOOS translator a list of the specific variables to watch
    			// When a message is received from the simulation:
    			// 1) log the raw text from the simulator    			
    			// 2) decode the message 
    			// 3) use the appropriate translator to do the translation into MOOS commands
    			// 4) call the collective intelligence hooks if necessary
    			
    			// On message from ActiveMQ for the collective intelligence side
    			// Work out the command from the CI behaviour request
    			
    			// On a fault generation message - FaultInstance
    }
    
	public static void main(String [] args) {
		// On initialisation, read the DSL concrete syntax file and construct the appropriate ATLAS objects here
		// TODO: replace with a real loader once the concrete syntax is ready
		DSLLoader dslloader = new StubDSLLoader();
		
		Mission mission = dslloader.loadMission();
		ATLASCore core = new ATLASCore(mission);
		core.runMiddleware();
	}
}