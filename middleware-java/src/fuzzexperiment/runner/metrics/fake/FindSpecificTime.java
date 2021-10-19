package fuzzexperiment.runner.metrics.fake;

import java.util.List;

import fuzzexperiment.runner.metrics.Metric;
import fuzzexperiment.runner.metrics.MetricComputeFailure;
import fuzzingengine.FuzzingFixedTimeSpecification;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.FuzzingTimeSpecification;

public class FindSpecificTime extends FakeMetric {
	private double rangeStart;
	private double rangeEnd;
	
	public FindSpecificTime(double rangeStart, double rangeEnd) {
		this.rangeStart = rangeStart;
		this.rangeEnd = rangeEnd;
	}
	
	private double getMetricValue(double start, double end) {
		double len = end - start;
		boolean isInRange = (start > rangeStart) && (end < rangeEnd);
		if (isInRange) {
			return 1.0 / len;
		} else {
			return 0.0;
		}
	}
	
	public Double computeFromLogs(List<FuzzingKeySelectionRecord> recs, String logDir) throws MetricComputeFailure {
		// Get the first record
		double outval = 0.0;
		
		for (FuzzingKeySelectionRecord r : recs) {
			FuzzingTimeSpecification ts = r.getTimeSpec();
			if (ts instanceof FuzzingFixedTimeSpecification) {
				FuzzingFixedTimeSpecification fts = (FuzzingFixedTimeSpecification)ts;
				double start = fts.getStartTime();
				double end = fts.getEndTime();
				double val = getMetricValue(start, end);
				System.out.println("FindSpecificTime - key selection record = " + recs + ",[start=" + start + ",end=" + end + "]-val = " + val) ;
				outval = Math.max(val, outval);
			}
		}
		return outval;
	}
	
	public MetricDirection optimiseDirection() {
		return Metric.MetricDirection.HIGHEST;
	} 
}
