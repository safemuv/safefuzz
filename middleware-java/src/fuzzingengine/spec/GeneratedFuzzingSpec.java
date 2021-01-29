package fuzzingengine.spec;

import java.util.Optional;
import atlasdsl.Mission;

import fuzzingengine.*;
public class GeneratedFuzzingSpec {

	public static FuzzingEngine createFuzzingEngine(Mission m) {
	FuzzingEngine fe = new FuzzingEngine(m);
	FuzzingSimMapping simMapping = new FuzzingSimMapping();
	
		
	simMapping.setComponentFuzzingInfo("uSimMarine", FuzzingSimMapping.FuzzingNature.BINARY, Optional.empty(), Optional.of("/home/atlas/atlas/atlas-middleware/custom-moos//bin/uSimMarine"));
		
	simMapping.addRecord("uSimMarine", "DESIRED_THRUST", "DEZIRED_THRUST", 
	FuzzingSimMapping.VariableDirection.INBOUND, Optional.of("/home/atlas/atlas/atlas-middleware/custom-moos//bin/uSimMarine"),
		Optional.empty()
	);
	simMapping.addRecord("uSimMarine", "DESIRED_RUDDER", "DEZIRED_RUDDER", 
	FuzzingSimMapping.VariableDirection.INBOUND, Optional.of("/home/atlas/atlas/atlas-middleware/custom-moos//bin/uSimMarine"),
		Optional.empty()
	);
	
	
			 
	
	simMapping.addRecord("NULL-DETECTION_ELLA", "TESTMSG1'", "TESTMSG1", 
	FuzzingSimMapping.VariableDirection.INBOUND, Optional.empty(),
		Optional.empty()
	);

	
	
	fe.setSimMapping(simMapping);
	fe.setupFromFuzzingFile("/home/atlas/atlas/atlas-middleware/middleware-java/moos-sim/fuzz-configs/test.fuzz", m);
	return fe;
	}
}