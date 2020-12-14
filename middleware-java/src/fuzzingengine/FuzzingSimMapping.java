package fuzzingengine;

import java.util.HashMap;
import java.util.Map;

public class FuzzingSimMapping {
	// Needs to store classes containing all the info from the model
	
	// Returns all the strings to mutate for a binary
	public HashMap<String,String> getBinaryChanges(String component, HashMap<String,String> inNames) {
		//	TODO: returning null hashmap for getBinaryChanges 
		// replace this with filtering the input names to determine any defined as binary
		System.out.println("TODO: Define the binary changes from FuzzingSimMapping");
		HashMap<String,String> binaryVars = new HashMap<String,String>();
	
		for (Map.Entry<String,String> me : inNames.entrySet()) {
			String inkey = me.getKey();
			String outkey = me.getValue();
			if (isBinary(me.getKey())) {
				binaryVars.put(inkey, outkey);
			}
		}
		return binaryVars;
	}

	private boolean isBinary(String key) {
		return true;
	}

	public String getFullPath(String component) {
		// TODO Auto-generated method stub
		System.out.println("TODO: Define getFullPath");
		return "/tmp/uSimMarine";
	}
}
