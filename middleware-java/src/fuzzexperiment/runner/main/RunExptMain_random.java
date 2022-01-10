package fuzzexperiment.runner.main;

import java.io.IOException;
import java.util.Random;

import atlasdsl.Mission;
import atlasdsl.loader.DSLLoadFailed;
import atlasdsl.loader.GeneratedDSLLoader;
import fuzzexperiment.runner.*;
import fuzzexperiment.runner.metrics.*;
import fuzzingengine.exptgenerator.*;

public class RunExptMain_random {
	public static void main(String[] args) {
		String resFileName = "fuzzexpt-generated-solutions.res";
		int runNumFixed = 0;
		int missionCount = 30;

		try {
			Mission m = new GeneratedDSLLoader().loadMission();
			MetricHandler mh = new MetricHandler(m, resFileName); 
			String csvBaseName = "/tmp/fuzzexpt";
			FuzzingTimeSpecificationGenerator tgen = new FuzzingTimeSpecificationGeneratorStartEnd(m, new Random());
			ExptParams ep = new RunRandomlyGeneratedExperiments(resFileName, m, csvBaseName, missionCount, tgen, runNumFixed);
			FuzzExptRunner r = new FuzzExptRunner(ep, mh, true);
			r.run();
		} catch (DSLLoadFailed | IOException e) {
			e.printStackTrace();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
}
