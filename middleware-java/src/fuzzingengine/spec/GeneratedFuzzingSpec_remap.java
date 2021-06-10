package fuzzingengine.spec;

import java.util.Optional;
import atlasdsl.Mission;

import fuzzingengine.*;
import fuzzingengine.operationparamsinfo.*;

public class GeneratedFuzzingSpec_remap {

	public static FuzzingEngine createFuzzingEngine(Mission m, boolean loadCSV) {
	FuzzingEngine fe = new FuzzingEngine(m);
	FuzzingSimMapping simMapping = new FuzzingSimMapping();
	
	
	
	try {
	
	
	   OperationParameterSet ops1 = new OperationParameterSet("JSONPointChange-RandomOffset-10-each-axis", "JSONPointChange");
 
 
	   			OperationParameter opp1 = new StringConstantOperationParameter("Nature", "RANDOMOFFSET");
	   			
	   		
	   		
	   		
	   		ops1.addParameter(opp1);
	   			
 
	   			OperationParameter opp2 = new DoubleRangeOperationParameter("X", 0.0, 10.0);
	   		
	   		
	   		
	   		ops1.addParameter(opp2);
	   			
 
	   			OperationParameter opp3 = new DoubleRangeOperationParameter("Y", 0.0, 10.0);
	   		
	   		
	   		
	   		ops1.addParameter(opp3);
	   			
 
	   			OperationParameter opp4 = new DoubleRangeOperationParameter("Z", 0.0, 10.0);
	   		
	   		
	   		
	   		ops1.addParameter(opp4);
	
	
				simMapping.addLaunchFilePath("/home/jharbin/catkin_ws/src/safemuv/safemuv_shared/launcher/shared_launcher.launch");
				simMapping.addLaunchFilePath("/home/jharbin/catkin_ws/src/safemuv/safemuv_launchers/launch/safemuv_launch_obstacle_publisher.launch");
	
	
		
		
		
		
	simMapping.setComponentFuzzingInfo("ual", FuzzingSimMapping.FuzzingNature.NO_MODIFICATIONS, Optional.empty(), Optional.empty(), false);
		
				
		simMapping.addRecord("ual", "/ual/velocity", "/ual/set_velocity", 
		FuzzingSimMapping.VariableDirection.OUTBOUND, Optional.of("/home/jharbin/catkin_ws/src/safemuv//ual"), Optional.of("geometry_msgs/TwistStamped"),
		true, Optional.of(new TimeSpec(m,100.0,150.0)), Optional.of(1.0));
	
			simMapping.addOperationParameterSetForVariable("/ual/velocity", ops1, "twist.linear");
	
	

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