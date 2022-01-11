package fuzzexperiment.runner.main;

import java.io.IOException;

import atlasdsl.Mission;
import atlasdsl.loader.DSLLoadFailed;
import atlasdsl.loader.GeneratedDSLLoader;
import fuzzexperiment.runner.*;
import fuzzexperiment.runner.metrics.*;
import fuzzexperiment.runner.rmkg.RMKGInterface;

public class TestExperiment_Lab {
	
	public static void main(String [] args) {
		String scenarioID = args[0];
		int runNum = Integer.valueOf(args[1]);
		int count = Integer.valueOf(args[2]);
		
		String fuzzFileName = RMKGInterface.getScenarioFuzzFile(scenarioID, runNum);
		
		String resFileName = "labresults.out";
		
		boolean startLaunchers = false;
		if (args.length > 3) {
			String startLaunchersStr = args[3];
			if (startLaunchersStr.toLowerCase().equals("startlaunchers")) {
				startLaunchers = true;
			}
		}
		
		try {
			Mission mission = new GeneratedDSLLoader().loadMission();
			MetricHandler mh = new MetricHandler(mission, resFileName);
			ExptParams ep = new RunRepeated(fuzzFileName, count, 1, startLaunchers);
			FuzzExptRunner r;
			r = new FuzzExptRunner(ep, mh, startLaunchers, scenarioID);
			r.run();
		} catch (DSLLoadFailed | IOException e) {
			e.printStackTrace();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
}
