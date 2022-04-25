package fuzzexperiment.runner.jmetal.grammar.variables;

import javax.json.JsonNumber;
import javax.json.JsonObject;

import middleware.core.ATLASCore;

public class TimeVariable extends VariableTemplate {
	public double getValue(String robotName, ATLASCore core) {
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
	}
}
