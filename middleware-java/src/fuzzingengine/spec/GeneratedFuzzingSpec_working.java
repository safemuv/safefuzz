package fuzzingengine.spec;

import java.util.Optional;
import atlasdsl.Mission;

import fuzzingengine.*;

public class GeneratedFuzzingSpec_working {

	public static FuzzingEngine createFuzzingEngine(Mission m) {
		FuzzingEngine fe = new FuzzingEngine(m);
		FuzzingSimMapping simMapping = new FuzzingSimMapping();

		simMapping.setComponentFuzzingInfo("ual", FuzzingSimMapping.FuzzingNature.NO_MODIFICATIONS, Optional.empty(),
				Optional.empty(), false);

		simMapping.addRecord("ual", "/ual/velocity", "/ual/set_velocity", FuzzingSimMapping.VariableDirection.OUTBOUND,
				Optional.of("/home/jharbin/catkin_ws/src/safemuv//ual"), Optional.of("geometry_msgs/TwistStamped"),
				true, Optional.empty(), Optional.empty());

		fe.setSimMapping(simMapping);
		fe.setupFromFuzzingFile("/home/jharbin//academic/atlas/atlas-middleware/fuzz-configs/ros-fuzztest.csv", m);
		return fe;
	}
}