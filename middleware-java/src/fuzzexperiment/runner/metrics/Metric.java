package fuzzexperiment.runner.metrics;

public abstract class Metric {
	public abstract void computeFromLogs(String logDir);
	public abstract double getResult();
}
