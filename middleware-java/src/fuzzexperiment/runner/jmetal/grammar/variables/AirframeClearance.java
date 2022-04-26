// protected region customHeaders on begin
package fuzzexperiment.runner.jmetal.grammar.variables;

import javax.json.*;
import middleware.core.ATLASCore;

// protected region customHeaders end

public class AirframeClearance extends VariableTemplate {
	// protected region customFunction on begin
	// protected region customFunction end

	public double getValue(String robotName, ATLASCore core) {
		// Implement the metric here
		// protected region userCode on begin
		Object gv = core.getGoalVariable(robotName, "/airframe_clearance");
		if (gv != null) {
			edu.wpi.rail.jrosbridge.messages.Message m = (edu.wpi.rail.jrosbridge.messages.Message)gv;
			String typ = m.getMessageType();
			JsonObject jobj = m.toJsonObject();
			JsonNumber n = (JsonNumber)jobj.get("data");
			double distVal = n.doubleValue();
			return distVal;
		} else {
			return Double.MAX_VALUE;
		}
		// protected region userCode end
	}
}
