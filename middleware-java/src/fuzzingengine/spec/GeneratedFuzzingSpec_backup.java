package fuzzingengine.spec;

import java.util.Optional;
import atlasdsl.Mission;

import fuzzingengine.*;
import fuzzingengine.operationparamsinfo.*;

public class GeneratedFuzzingSpec_backup {
	public static FuzzingEngine createFuzzingEngine(Mission m, boolean loadCSV) {
	FuzzingEngine fe = new FuzzingEngine(m);
	FuzzingSimMapping simMapping = new FuzzingSimMapping();
	
	simMapping.setComponentFuzzingInfo("ual", FuzzingSimMapping.FuzzingNature.NO_MODIFICATIONS, Optional.empty(), Optional.empty(), false);
		
	simMapping.addRecord("ual", "/ual/velocity", "/ual/set_velocity", 
	FuzzingSimMapping.VariableDirection.OUTBOUND, Optional.of("/home/jharbin/catkin_ws/src/safemuv//ual"), Optional.of("geometry_msgs/TwistStamped"),
	true);
	
	// Need to add the parameters to the operations parameter set
	OperationParameterSet randomGen1 = new OperationParameterSet("randomPoint", "JSONPointChange");
	randomGen1.addParameter(new StringConstantOperationParameter("Nature", "RANDOMOFFSET"));
	randomGen1.addParameter(new DoubleRangeOperationParameter("X", 0.0, 2.0));
	randomGen1.addParameter(new DoubleRangeOperationParameter("Y", 0.0, 2.0));
	randomGen1.addParameter(new DoubleRangeOperationParameter("Z", 0.0, 0.1));
	simMapping.addOperationParameterSetForVariable("/ual/velocity", randomGen1, "twist.linear");
	
	fe.setSimMapping(simMapping);
	if (loadCSV)
		fe.setupFromFuzzingFile("/home/jharbin//academic/atlas/atlas-middleware/fuzz-configs/ros-fuzztest.csv", m);
	return fe;
	}
}