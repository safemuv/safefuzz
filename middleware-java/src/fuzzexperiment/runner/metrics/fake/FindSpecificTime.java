package fuzzexperiment.runner.metrics.fake;

import java.util.List;

import fuzzexperiment.runner.metrics.Metric;
import fuzzexperiment.runner.metrics.MetricComputeFailure;
import fuzzingengine.FuzzingFixedTimeSpecification;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.FuzzingTimeSpecification;

public class FindSpecificTime extends FakeMetric {
	private double centreValue;
	private double range;
	private double rangeStart;
	private double rangeEnd;
	
	public FindSpecificTime(double centreValue, double range) {
		this.centreValue = centreValue;
		this.range = range;
		this.rangeStart = centreValue - range;
		this.rangeEnd = centreValue + range;
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
		if (recs.size() > 0) {
			FuzzingKeySelectionRecord r = recs.get(0);
			FuzzingTimeSpecification ts = r.getTimeSpec();
			if (ts instanceof FuzzingFixedTimeSpecification) {
				FuzzingFixedTimeSpecification fts = (FuzzingFixedTimeSpecification)ts;
				double start = fts.getStartTime();
				double end = fts.getEndTime();
				double val = getMetricValue(start, end);
				System.out.println("FindSpecificTime - key selection record = " + recs + ",[start=" + start + ",end=" + end + "]-val = " + val) ;
				return val;
			} else {
				return 0.0;
			}
				
		} else {
			//throw new MetricComputeFailure("FindSpecificTime given zero-size recs");
			return 0.0;
		}
	}
	
	public MetricDirection optimiseDirection() {
		
		return Metric.MetricDirection.HIGHEST;
	} 
}
