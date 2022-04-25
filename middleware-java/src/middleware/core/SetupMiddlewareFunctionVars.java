package middleware.core;
import fuzzexperiment.runner.jmetal.grammar.variables.*;

public class SetupMiddlewareFunctionVars {
	// TODO: this code needs to be auto-generated - from the listing of grammar variables in the DSL
	public static void setup(ATLASCore core) {
//		middlewareFunctionVariables.put("starting_point_distance", spdLambda);
		core.addMiddlewareFunctionVariables("airframe_clearance", new AirframeClearance());
		core.addMiddlewareFunctionVariables("time", new TimeVariable());
//		middlewareFunctionVariables.put("time", timeLambda);
//		middlewareFunctionVariables.put("interrobot_distance", irLambda);
	}
}