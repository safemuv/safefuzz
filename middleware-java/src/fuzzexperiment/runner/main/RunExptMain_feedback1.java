package fuzzexperiment.runner.main;

import java.io.IOException;

import atlasdsl.Mission;
import atlasdsl.loader.DSLLoadFailed;
import atlasdsl.loader.GeneratedDSLLoader;
import fuzzexperiment.runner.*;
import fuzzexperiment.runner.metrics.*;

public class RunExptMain_feedback1 {
	public static void main(String[] args) {
		String resFileName = "fuzzexpt-generated-solutions.res";
		int missionCount = 30;

		try {
			Mission m = new GeneratedDSLLoader().loadMission();
			MetricHandler mh = new MetricHandler(m, resFileName); 
			String csvBaseName = "/tmp/fuzzexpt-feedback";
			ExptParams ep = new RunExperimentsMetricFeedback(resFileName, m, csvBaseName, missionCount);
			FuzzExptRunner r = new FuzzExptRunner(ep, mh);
			r.useFakeRun();
			r.run();
		} catch (DSLLoadFailed | IOException e) {
			e.printStackTrace();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
}
