package test.fuzzingengine;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

import fuzzingengine.FuzzingEngine;
import fuzzingengine.FuzzingEngineMatchFailure;
import fuzzingengine.operations.*;

public class FuzzingEngineTest {
	public static void main(String [] args) {
		String regex = "\\(([0-9]+),([0-9]+)\\)";
		String source = "(13,27)";
		Pattern p = Pattern.compile(regex);
		
		Matcher m = p.matcher(source);
		m.find();
		String s1 = m.group(1);
		String s2 = m.group(2);
		
		System.out.println("TEST group1 = " + s1);
		System.out.println("TEST group2 = " + s2);
		
		int groupToReplace = 2;
		ValueFuzzingOperation op = IntegerVariableChange.Random(0,100);
		
		try {
			String res = FuzzingEngine.replaceGroup(p, source, groupToReplace, op);
			System.out.println("res = " + res);
		} catch (FuzzingEngineMatchFailure fe) {
			System.out.println("FuzzingEngineMatchFailure - " + fe);
		}
		
	}
}
