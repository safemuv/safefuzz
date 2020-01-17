package middleware.core;

import atlasdsl.*;

// This code will be combined with the simulator-specific code
// during code generation
public class ATLASCore {
	public static void main(String [] args) {
		// On initialisation, read the DSL file and construct the appropriate ATLAS objects
		
		// Start the ActiveMQ connection - via ActiveMQLink
		// Get from the MOOS translator a list of the specific ports
		// When a message is received from the simulation:
		// 1) log the raw text from the simulator
		// 2) use the appropriate translator to do the translation into 
		// 3) call the collective intelligence hooks if necessary
		
		// On message from ActiveMQ for the collective intelligence side
		// Work out the command from the CI behaviour request
		
		// On a fault generation message - FaultInstance
	}
}
