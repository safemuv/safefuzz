package fuzzexperiment.runner.main;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import atlasdsl.loader.DSLLoadFailed;
import fuzzexperiment.runner.*;
import fuzzexperiment.runner.metrics.*;

public class RunExptMain {
	
	public static void main(String [] args) {
		List<OfflineMetric> metrics = new ArrayList<OfflineMetric>();
		String resFileName = "fuzzexpt-set-of-solutions.res";
		
		MetricHandler mh = new MetricHandler(metrics);
		ExptParams ep = new RunRepeated(resFileName, 30);
		FuzzExptRunner r;
		try {
			r = new FuzzExptRunner(ep, mh);
			r.run();
		} catch (DSLLoadFailed | IOException e) {
			e.printStackTrace();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
}
