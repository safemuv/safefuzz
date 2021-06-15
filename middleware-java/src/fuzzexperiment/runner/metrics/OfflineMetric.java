package fuzzexperiment.runner.metrics;

public abstract class OfflineMetric extends Metric {
	public abstract Double computeFromLogs(String logDir) throws MetricComputeFailure;
}
