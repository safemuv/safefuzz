package fuzzingengine.spec;

import java.util.Optional;
import atlasdsl.Mission;

import fuzzingengine.*;
public class GeneratedFuzzingSpec {

	public static FuzzingEngine createFuzzingEngine(Mission m) {
	FuzzingEngine fe = new FuzzingEngine(m);
	FuzzingSimMapping simMapping = new FuzzingSimMapping();
	
	simMapping.setComponentFuzzingInfo("ual", FuzzingSimMapping.FuzzingNature.BINARY, Optional.empty(), Optional.of("PATH"));
		
	simMapping.addRecord("ual", "/ual/velocity", "/ual/set_velocity", 
	FuzzingSimMapping.VariableDirection.INBOUND, Optional.of("/home/atlas/atlas/atlas-middleware/custom-moos//bin/uSimMarine"),
		Optional.empty()
	);
	
	fe.setSimMapping(simMapping);
	fe.setupFromFuzzingFile("/home/jharbin/academic/atlas/atlas-middleware/fuzz-configs/ros-fuzztest.csv", m);
	return fe;
	}
}