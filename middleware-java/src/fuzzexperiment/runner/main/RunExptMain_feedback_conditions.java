package fuzzexperiment.runner.main;

import java.io.File;
import java.io.IOException;

import atlasdsl.Mission;
import atlasdsl.loader.DSLLoadFailed;
import atlasdsl.loader.GeneratedDSLLoader;
import fuzzexperiment.runner.*;
import fuzzexperiment.runner.jmetal.grammar.Grammar;
import fuzzexperiment.runner.metrics.*;
import fuzzingengine.exptgenerator.FuzzingExperimentModifier;

public class RunExptMain_feedback_conditions {
	public static void main(String[] args) {
		String resFileName = "feedback-expt.res";
		int runCount = 120;
		int populationLimit = 20;

		try {
			Mission m = new GeneratedDSLLoader().loadMission();
			MetricHandler mh = new MetricHandler(m, resFileName); 
			String csvBaseName = "/tmp/fuzzexpt-feedback";
			Grammar<String> g = Grammar.fromFile(new File("/home/jharbin/academic/atlas/atlas-middleware/grammar/safemuv-fuzzing-cond.bnf"));
			FuzzingExperimentModifier exptGen = new FuzzingExperimentModifier(m);
			ExptParams ep = new RunExperimentsMetricFeedback(exptGen, resFileName, m, csvBaseName, runCount, populationLimit);
			FuzzExptRunner r = new FuzzExptRunner(ep, mh);
			r.run();
		} catch (DSLLoadFailed | IOException e) {
			e.printStackTrace();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
}
