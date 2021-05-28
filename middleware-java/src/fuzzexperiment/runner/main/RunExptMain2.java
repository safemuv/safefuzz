package fuzzexperiment.runner.main;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import atlasdsl.Mission;
import atlasdsl.loader.DSLLoadFailed;
import atlasdsl.loader.GeneratedDSLLoader;
import fuzzexperiment.runner.*;
import fuzzexperiment.runner.metrics.*;

public class RunExptMain2 {
	public static void main(String[] args) {
		List<Metric> metrics = new ArrayList<Metric>();
		String resFileName = "fuzzexpt-generated-solutions.res";
		int missionCount = 30;

		try {
			Mission m = new GeneratedDSLLoader().loadMission();
			MetricHandler mh = new MetricHandler(metrics); 
			ExptParams ep = new RunRandomlyGeneratedExperiments(resFileName, m, resFileName, missionCount);
			FuzzExptRunner r = new FuzzExptRunner(ep, mh);
			r.run();
		} catch (DSLLoadFailed | IOException e) {
			e.printStackTrace();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
}
