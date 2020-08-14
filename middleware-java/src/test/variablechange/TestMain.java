package test.variablechange;

import fuzzingengine.*;
import middleware.core.MOOSVariableUpdate;
import middleware.core.MOOSVariableUpdate.VariableInvalid;

public class TestMain {
	public static void testReplace(String invalue) {
		System.out.println("before = " + invalue);
		String out = invalue.replaceAll("A", "B");
		System.out.println("after=" + out);	
	}
	
	public static void main(String [] args) {
		testReplace("A");
		
		
		FuzzingEngine vf = new FixedNumericVariableChangeFuzzingEngine(0.0);
		
		
		MOOSVariableUpdate mup;
		try {
			mup = new MOOSVariableUpdate("ella", "TEST=q=23.5", 12.5);
			MOOSVariableUpdate mupChanged = vf.fuzzMessage(mup);
			System.out.println("changed value = " + mupChanged.getValue());
		} catch (VariableInvalid e) {
			e.printStackTrace();
		}
	}
}
