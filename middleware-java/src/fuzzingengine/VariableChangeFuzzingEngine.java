package fuzzingengine;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

import middleware.core.MOOSVariableUpdate;

public class VariableChangeFuzzingEngine extends FuzzingEngine {
	public boolean doFuzzing = true;
	private Pattern numberScanner;
	
	public VariableChangeFuzzingEngine() {
		numberScanner = Pattern.compile("^[+-]?([0-9]*.)?[0-9]+$");
	}
		
	public MOOSVariableUpdate fuzzMessage(MOOSVariableUpdate mup) {
		System.out.println("FUZZ ENGINE VARIABLE: " + mup.getValue());
		MOOSVariableUpdate mupOut = new MOOSVariableUpdate(mup);
		if (doFuzzing) {
			Matcher m = numberScanner.matcher(mup.getValue());
			int pos = m.start();
			// TODO: set it in the string here
			String replaced = "0.00";
			mupOut.setValue(replaced);
		}
		return mup;
	}
}
