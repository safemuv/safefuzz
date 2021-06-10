package fuzzingengine.spec;

import java.util.Optional;
import atlasdsl.Mission;

import fuzzingengine.*;
import fuzzingengine.operationparamsinfo.*;

public class GeneratedFuzzingSpec {

	public static FuzzingEngine createFuzzingEngine(Mission m, boolean loadCSV) {
	FuzzingEngine fe = new FuzzingEngine(m);
	FuzzingSimMapping simMapping = new FuzzingSimMapping();
	
	
	
	try {
	
	
	   OperationParameterSet ops1 = new OperationParameterSet("StringChangeRandom", "StringVariableChange");
 
 
	   			OperationParameter opp1 = new StringConstantOperationParameter("Nature", "RANDOMALPHA");
	   			
	   		
	   		
	   		
	   		ops1.addParameter(opp1);
	   			
	   		
	   		
 
	   			OperationParameter opp2 = new IntConstantOperationParameter("maxLen", 8);
	   		
	   		ops1.addParameter(opp2);
	   OperationParameterSet ops2 = new OperationParameterSet("Delay-3s", "DelayFuzzingOperation");
 
	   			
	   		
 
	   			OperationParameter opp3 = new DoubleConstantOperationParameter("delay", 3.0);
	   		
	   		
	   		ops2.addParameter(opp3);
	
	
				simMapping.addLaunchFilePath("/home/jharbin/catkin_ws/src/safemuv/safemuv_shared/launcher/shared_launcher.launch");
				simMapping.addLaunchFilePath("/home/jharbin/catkin_ws/src/safemuv/safemuv_launchers/launch/safemuv_launch_obstacle_publisher.launch");
	
	
		
		
		
		
	simMapping.setComponentFuzzingInfo("null", FuzzingSimMapping.FuzzingNature.NO_MODIFICATIONS, Optional.empty(), Optional.empty(), false);
		
				
		simMapping.addRecord("null", "/topic2", "/topic1", 
		FuzzingSimMapping.VariableDirection.OUTBOUND, Optional.of("/home/jharbin/catkin_ws/src/safemuv//ual"), Optional.of("std_msgs/String"),
		false, Optional.of(new TimeSpec(m,100.0,150.0)), Optional.of(1.0));
	
			simMapping.addOperationParameterSetForVariable("/topic2", ops1, "");
	
	

	} catch (InvalidTimeSpec its) {
			System.out.println("InvalidTimeSpec - " + its.getNature());
			its.printStackTrace();
	}
	
	fe.setSimMapping(simMapping);
	if (loadCSV) {
		fe.setupFromFuzzingFile("/home/jharbin//academic/atlas/atlas-middleware/fuzz-configs/ros-fuzztest.csv", m);
	}
	return fe;
	}
}