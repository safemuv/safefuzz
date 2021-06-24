package fuzzexperiment.runner.metrics.fake;

import java.util.List;

import fuzzexperiment.runner.metrics.MetricComputeFailure;
import fuzzexperiment.runner.metrics.OfflineMetric;
import fuzzingengine.FuzzingKeySelectionRecord;

public abstract class FakeMetric extends OfflineMetric {
	public abstract Double computeFromLogs(List<FuzzingKeySelectionRecord> recs, String logDir) throws MetricComputeFailure;
}
