package fuzzexperiment.runner.rmkg;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import fuzzexperiment.runner.jmetal.FuzzingSelectionsSolution;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.FuzzingSelectionRecord;
import utils.ExptHelper;

public class RMKGInterface {
	public static final boolean REGENERATE_SCENARIOS = true;
	
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
	
	public static void generateLaunchScripts(String workingPath, String scenarioName, List<String> fuzzTopicList, List<String> modifiedFiles, String tempDirName) {
		String fuzzTopicString = String.join(" ", fuzzTopicList);
		String[] args = new String[]{scenarioName, fuzzTopicString};
		ExptHelper.runScriptNew(workingPath, "./generate_launch_files.sh", args);
	}
	
	public static void generateLaunchScriptsRMKG_ROS(String scenarioID, String workingPath, String scenarioDirName, List<String> fuzzTopicList, List<String> modifiedFiles, String fuzzConfigCSV, String tempDirName, int testNumID, String configDir) {
		String fuzzTopicString = String.join(" ", fuzzTopicList);
		String testNumID_s = String.valueOf(testNumID);
		String[] args = new String[]{scenarioID, testNumID_s, fuzzTopicString, fuzzConfigCSV, configDir };
		String argsCombined = Arrays.stream(args).collect(Collectors.joining(" "));
		System.out.println("rmkg scenario generation args - " + argsCombined);
		ExptHelper.runScriptNew(workingPath, "./generate_launch_files_rmkg.sh", args);
	}
}
