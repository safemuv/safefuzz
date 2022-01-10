package fuzzexperiment.runner.main;

import java.io.IOException;

import atlasdsl.Mission;
import atlasdsl.loader.DSLLoadFailed;
import atlasdsl.loader.GeneratedDSLLoader;
import fuzzexperiment.runner.*;
import fuzzexperiment.runner.metrics.*;

public class RunExptRepeated_fuzzing {
	
	public static void main(String [] args) {
		String fuzzFileName = args[0];
		int count = Integer.valueOf(args[1]);
		String resFileName = args[2];
		String startLaunchersStr = args[3];
		boolean startLaunchers = false;
		
		if (startLaunchersStr.equals("yes")) {
			startLaunchers = true;
		}
		
		try {
			Mission mission = new GeneratedDSLLoader().loadMission();
			MetricHandler mh = new MetricHandler(mission, resFileName);
			ExptParams ep = new RunRepeated(fuzzFileName, count, 1, startLaunchers);
			FuzzExptRunner r;
			r = new FuzzExptRunner(ep, mh, startLaunchers);
			r.run();
		} catch (DSLLoadFailed | IOException e) {
			e.printStackTrace();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
}
