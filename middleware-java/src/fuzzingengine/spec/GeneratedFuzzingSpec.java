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
	
	
	
	
	
	
		
		
		
		
	simMapping.setComponentFuzzingInfo("/ual", FuzzingSimMapping.FuzzingNature.NO_MODIFICATIONS, Optional.empty(), Optional.empty(), false);
		
				
		simMapping.addRecord("/ual", "/ual/set_velocity_prime", "/ual/set_velocity", 
		FuzzingSimMapping.VariableDirection.OUTBOUND, Optional.of("/home/jharbin/catkin_ws/src/safemuv//ual"), Optional.of("geometry_msgs/TwistStamped"),
		true, Optional.of(new TimeSpec(m,0.0,100.0)), Optional.empty());
	
	
	

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