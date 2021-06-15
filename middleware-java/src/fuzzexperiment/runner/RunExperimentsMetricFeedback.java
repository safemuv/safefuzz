package fuzzexperiment.runner;

import java.util.Map;
import java.util.Optional;
import java.util.Set;

import atlasdsl.Mission;
import fuzzexperiment.runner.metrics.Metric;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.exptgenerator.FuzzingExperimentModifier;

public class RunExperimentsMetricFeedback extends ExptParams {
	
	private class BestPreviousResult {
		Set<FuzzingKeySelectionRecord> spec;
		Map<Metric,Object> metric;
	}
	
	private String resFileName;
	int count = 0;
	int countLimit;
	private Mission mission;
	private String fuzzCSVBaseName;
	private String bestFilename;
	private Map<Metric, Object> bestMetrics;
	
	private	FuzzingExperimentModifier g;

	private String getCurrentFilename() {
		return fuzzCSVBaseName + "-" + count + ".csv";
	}
	
	private void newGeneratedFile() {
		g.generateExperiment(Optional.of(getCurrentFilename()));
	}
	
	public RunExperimentsMetricFeedback(String resFileName, Mission mission, String fuzzCSVBaseName, int countLimit) {
		this.resFileName = resFileName;
		this.mission = mission;
		this.countLimit = countLimit;
		this.fuzzCSVBaseName = fuzzCSVBaseName;
		g = new FuzzingExperimentModifier(mission);
		newGeneratedFile();
	}

	public boolean completed() {
		return (count >= countLimit);
	}

	public void printState() {
		System.out.println("Evaluating entry " + getCurrentFilename());
	}

	public void advance() {
		count++;
		newGeneratedFile();
	}

	public Optional<String> getNextFuzzingCSVFileName() {
		return Optional.of(getCurrentFilename());
	}
	
	private boolean metricsMoreViolations(Map<Metric, Object> res, Map<Metric, Object> bestMetrics) {
		
		boolean better = true;
		for (Map.Entry<Metric,Object> entry : res.entrySet()) {
			Metric m = entry.getKey();
			Object o = entry.getValue();
			Object obest = bestMetrics.get(m);
			
			Double o_d = (Double)o;
			Double obest_d = (Double)obest;

			
			if (o_d < obest_d) {
				better = false; 
			}
		}
		return better;
	}

	public void advance(Map<Metric, Object> res) {
		count++;
		
		// If the metrics are better than the best, store this one
		if (metricsMoreViolations(res, bestMetrics)) {
			bestFilename = getCurrentFilename();
			bestMetrics = res;
		}
		
		g.generateExperimentBasedUpon(getCurrentFilename(), Optional.of(bestFilename), res);
	}
}
