package fuzzexperiment.runner.metrics.fake;

import java.util.List;

import atlasdsl.Mission;
import fuzzexperiment.runner.metrics.MetricComputeFailure;
import fuzzexperiment.runner.metrics.OfflineMetric;
import fuzzingengine.FuzzingKeySelectionRecord;

public abstract class FakeMetric extends OfflineMetric {
	public abstract Double computeFromLogs(List<FuzzingKeySelectionRecord> recs, String logDir, Mission mission) throws MetricComputeFailure;
}
