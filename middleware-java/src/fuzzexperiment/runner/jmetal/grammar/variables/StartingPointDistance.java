// protected region customHeaders on begin
package fuzzexperiment.runner.jmetal.grammar.variables;

import javax.json.*;

import atlasdsl.MissingProperty;
import atlasdsl.Robot;
import atlassharedclasses.Point;
import middleware.core.ATLASCore;

// protected region customHeaders end

public class StartingPointDistance extends VariableTemplate {
	// protected region customFunction on begin
	// protected region customFunction end

	public double getValue(String robotName, ATLASCore core) {
		// Implement the metric here
		// protected region userCode on begin
		Robot r = core.getMission().getRobot(robotName);
		try {
			Point p = r.getPointComponentProperty("location");
			Point orig = r.getPointComponentProperty("startLocation");
			double distance = orig.distanceTo(p);
			return distance;
		} catch (MissingProperty p) {
			return Double.MAX_VALUE;
		}
		// protected region userCode end
	}
}
