package fuzzingengine.spec;

import java.util.Optional;
import atlasdsl.Mission;

import fuzzingengine.*;
import fuzzingengine.operationparamsinfo.*;

public class GeneratedFuzzingSpec_backup2 {

	public static FuzzingEngine createFuzzingEngine(Mission m, boolean loadCSV) {
	FuzzingEngine fe = new FuzzingEngine(m);
	FuzzingSimMapping simMapping = new FuzzingSimMapping();
	
	
	   OperationParameterSet ops1 = new OperationParameterSet("JSONPointChange-RandomOffset", "JSONPointChange");
 
 
	   			OperationParameter opp1 = new StringConstantOperationParameter("Nature", "RANDOMOFFSET");
	   			
	   		
	   		
	   		
	   		ops1.addParameter(opp1);
	   			
 
	   			OperationParameter opp2 = new DoubleRangeOperationParameter("X", 0.0, 10.0);
	   		
	   		
	   		
	   		ops1.addParameter(opp2);
	   			
 
	   			OperationParameter opp3 = new DoubleRangeOperationParameter("Y", 0.0, 10.0);
	   		
	   		
	   		
	   		ops1.addParameter(opp3);
	   			
 
	   			OperationParameter opp4 = new DoubleRangeOperationParameter("Z", 0.0, 10.0);
	   		
	   		
	   		
	   		ops1.addParameter(opp4);
	
		
		
	simMapping.setComponentFuzzingInfo("ual", FuzzingSimMapping.FuzzingNature.NO_MODIFICATIONS, Optional.empty(), Optional.empty(), false);
		
				
	simMapping.addRecord("ual", "/ual/velocity", "/ual/set_velocity", 
	FuzzingSimMapping.VariableDirection.OUTBOUND, Optional.of("/home/jharbin/catkin_ws/src/safemuv//ual"), Optional.of("geometry_msgs/TwistStamped"),
	true);
	
	
		
		
	simMapping.setComponentFuzzingInfo("CalibrationDefinitions", FuzzingSimMapping.FuzzingNature.NO_MODIFICATIONS, Optional.empty(), Optional.empty(), true);
		
				
	simMapping.addRecord("CalibrationDefinitions", "calibration_points.yaml", "", 
	FuzzingSimMapping.VariableDirection.OUTBOUND, Optional.of("/home/jharbin/catkin_ws/src/safemuv/safemuv_situational_awareness/config"), Optional.empty(),
	false);
	
		simMapping.addOperationParameterSetForVariable("calibration_points.yaml", ops1, "twist.linear");
	
	

	// TODO: Add variables here
	
	fe.setSimMapping(simMapping);
	if (loadCSV) {
		fe.setupFromFuzzingFile("/home/jharbin//academic/atlas/atlas-middleware/fuzz-configs/ros-fuzztest.csv", m);
	}
	return fe;
	}
}