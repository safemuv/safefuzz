package fuzzexperiment.runner;

import java.util.Optional;

import exptrunner.metrics.MetricsProcessing;

public class RunOnSetOfSolutions extends ExptParams {

	private MetricsProcessing mp;
	private String resFileName;
	
	public RunOnSetOfSolutions(MetricsProcessing mp, String resFileName) {
		this.mp = mp;
		this.resFileName = resFileName;
	}

	@Override
	public boolean completed() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void printState() {
		// TODO Auto-generated method stub

	}

	@Override
	public void advance() {
		// TODO Auto-generated method stub

	}

	@Override
	public void logResults(String string, String modelFile) {

	}

	@Override
	public Optional<String> getNextFileName() {
		// TODO Auto-generated method stub
		return null;
	}

}
