package fuzzexperiment.runner.rmkg;

import java.util.ArrayList;
import java.util.List;

import fuzzexperiment.runner.jmetal.FuzzingSelectionsSolution;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.FuzzingSelectionRecord;

public class RMKGInterface {
	public static final boolean REGENERATE_SCENARIOS = false;
	
	// This is to bridge Argentina's script args. Gives the appropriate parameters for the
	// scenario generation script
	public static List<String> getFuzzTopicListFromScen(FuzzingSelectionsSolution solution) {
		List<String> fuzzTopics = new ArrayList<String>();
		
		// topicChoices options below has to match what is in generate_launchers.py
		List<String> topicChoices = new ArrayList<String>();
		topicChoices.add("trajectory_relative");
		topicChoices.add("desired_path");
		topicChoices.add("desired_trajectory_path");
		topicChoices.add("set_velocity");
		topicChoices.add("pose");
						
		List<FuzzingSelectionRecord> recs = solution.getVariables();
		
		for (FuzzingSelectionRecord r : recs) {
			if (r instanceof FuzzingKeySelectionRecord) {
				FuzzingKeySelectionRecord kr = (FuzzingKeySelectionRecord)r;
				for (String choice : topicChoices) {
					if (kr.getKey().contains(choice)) {
						fuzzTopics.add(choice);
					}
				}
			}
		}
		return fuzzTopics;
	}

	public static String getFuzzTopicListFromFile(String file) {
		return null;
	}
	
	// TODO: script runner conversions to launch generate_scenarios
	// TODO: script runner conversions to launch register_results
}
