package fuzzingengine;

import java.util.HashMap;

public class FuzzingSimMapping {
	// Needs to store classes containing all the info from the model
	
	// Returns all the strings to mutate for a binary
	public HashMap<String,String> getBinaryChanges(String component) {
		//	TODO: returning null hashmap for getBinaryChanges 
		System.out.println("TODO: Define the binary changes from FuzzingSimMapping");
		return new HashMap<String,String>();
	}

	public String getFullPath(String component) {
		// TODO Auto-generated method stub
		System.out.println("TODO: Define getFullPath");
		return "/tmp/incomplete";
	}
}
