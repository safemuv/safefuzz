// protected region customHeaders on begin
package fuzzexperiment.runner.jmetal.grammar.variables;

import javax.json.*;

import atlasdsl.MissingProperty;
import atlassharedclasses.Point;
import middleware.core.ATLASCore;

// protected region customHeaders end

public class InterrobotDistance extends VariableTemplate {
	// protected region customFunction on begin
	// protected region customFunction end

	public double getValue(String robotName, ATLASCore core) {
		// Implement the metric here
		// protected region userCode on begin
		try {
			// ignore the name, just use the hardcoded robot names - since it is scenario specific this
			// is OK
			Point r1pos = core.getMission().getRobot("uav_1").getPointComponentProperty("location");
			Point r2pos = core.getMission().getRobot("uav_2").getPointComponentProperty("location");
			double irDist = r1pos.distanceTo(r2pos);
			return irDist;
		} catch (MissingProperty e) {
			e.printStackTrace();
			return Double.MAX_VALUE;
		}
		// protected region userCode end
	}
}
