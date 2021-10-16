package fuzzexperiment.runner.main;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import atlasdsl.Mission;
import atlasdsl.loader.DSLLoadFailed;
import atlasdsl.loader.GeneratedDSLLoader;
import fuzzexperiment.runner.*;
import fuzzexperiment.runner.metrics.*;
import fuzzexperiment.runner.metrics.fake.FindSpecificTime;

public class RunExptMain_feedback_test {
	public static void main(String[] args) {
		String resFileName = "feedback-test-results.res";
		int runCount = 3000;
		int populationLimit = 10;

		try {
			Mission m = new GeneratedDSLLoader().loadMission();
			//MetricHandler mh = new MetricHandler(m, resFileName);
			List<OfflineMetric> fakeMetrics = new ArrayList<OfflineMetric>();
			fakeMetrics.add(new FindSpecificTime(100.0, 120.0));
			MetricHandler mh = new FakeMetricHandler(fakeMetrics, resFileName);
			String csvBaseName = "/tmp/fuzzexpt-feedbacktest";
			//ExptParams ep = new RunExperimentsMetricFeedback(resFileName, m, csvBaseName, runCount, populationLimit);
			//FuzzExptRunner r = new FuzzExptRunner(ep, mh);
			//r.useFakeRun();
			//r/.run();
		} catch (DSLLoadFailed | IOException e) {
			e.printStackTrace();
		//} catch (InterruptedException e) {
		//	e.printStackTrace();
		//}
		}
	}
}
