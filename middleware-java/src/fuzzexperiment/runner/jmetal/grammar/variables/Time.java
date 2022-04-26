// protected region customHeaders on begin
package fuzzexperiment.runner.jmetal.grammar.variables;

import javax.json.*;
import middleware.core.ATLASCore;

// protected region customHeaders end

public class Time extends VariableTemplate {
	// protected region customFunction on begin
	// protected region customFunction end

	public double getValue(String robotName, ATLASCore core) {
		// Implement the metric here
		// protected region userCode on begin
		double time = core.getTime();
		return time;
		// protected region userCode end
	}
}
