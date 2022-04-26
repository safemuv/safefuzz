// protected region customHeaders on begin
package fuzzexperiment.runner.jmetal.grammar.variables;

import javax.json.*;

import atlasdsl.MissingProperty;
import atlasdsl.Robot;
import atlassharedclasses.Point;
import middleware.core.ATLASCore;

// protected region customHeaders end

public class DistanceToLeftWingBase extends VariableTemplate {
	// protected region customFunction on begin
	Point fixedPoint = new Point(-29.14, -2.28, 5.2);
	// protected region customFunction end

	public double getValue(String robotName, ATLASCore core) {
		// Implement the metric here
		// protected region userCode on begin
		Robot r = core.getMission().getRobot(robotName);
		try {
			Point p = r.getPointComponentProperty("location");
			double distance = fixedPoint.distanceTo(p);
			return distance;
		} catch (MissingProperty p) {
			return Double.MAX_VALUE;
		}
		// protected region userCode end
	}
}
