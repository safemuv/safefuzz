package middleware.core;
import fuzzexperiment.runner.jmetal.grammar.variables.*;

public class SetupMiddlewareFunctionVars {
	public static void setup(ATLASCore core) {
		core.addMiddlewareFunctionVariables("airframe_clearance", new AirframeClearance());
		core.addMiddlewareFunctionVariables("time", new Time());
		core.addMiddlewareFunctionVariables("starting_point_distance", new StartingPointDistance());
		core.addMiddlewareFunctionVariables("interrobot_distance", new InterrobotDistance());
		core.addMiddlewareFunctionVariables("distance_to_left_wing_base", new DistanceToLeftWingBase());
		core.addMiddlewareFunctionVariables("distance_to_right_wing_base", new DistanceToRightWingBase());
		core.addMiddlewareFunctionVariables("distance_to_nose", new DistanceToNose());
	}
}