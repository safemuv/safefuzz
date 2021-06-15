package fuzzexperiment.runner;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import fuzzexperiment.runner.metrics.Metric;
import fuzzingengine.FuzzingSelectionRecord;

public class FuzzingExptResult implements Comparable<FuzzingExptResult> {
	List<FuzzingSelectionRecord> spec;
	private String filename;
	Map<Metric,Double> metrics;
	List<Metric> metricOrdering;
	
	private enum Ordering {
		ASCENDING,
		DESCENDING
	}
	
	private Ordering sortOrder = Ordering.ASCENDING;
	
	public FuzzingExptResult(List<FuzzingSelectionRecord> currentFuzzingSels, String filename, Map<Metric,Double> metrics) {
		this.spec = currentFuzzingSels;
		this.filename = filename;
		this.metrics = metrics;
	}
	
	public String getFilename() {
		return filename;
	}
	
	public Map<Metric,Double> getMetrics() {
		return metrics;
	}
	
	public boolean isBetter(FuzzingExptResult other) {
		boolean better = true;
		Map<Metric,Double> others = other.getMetrics();
		for (Map.Entry<Metric,Double> entry : metrics.entrySet()) {
			Metric m = entry.getKey();
			Double v = entry.getValue();
			Double vOther = others.get(m);
		
			if (v > vOther) {
				better = false; 
			}
		}
		return better;
	}
	
	// Using the domaince test from JMetal
	private int dominanceTest(List<Double> solution1, List<Double> solution2) {
	    int bestIsOne = 0;
	    int bestIsTwo = 0;
	    int result;
	    for (int i = 0; i < solution1.size(); i++) {
	      double value1 = solution1.get(i);
	      double value2 = solution2.get(i);
	      if (value1 != value2) {
	        if (value1 < value2) {
	          bestIsOne = 1;
	        }
	        if (value2 < value1) {
	          bestIsTwo = 1;
	        }
	      }
	    }
	    result = Integer.compare(bestIsTwo, bestIsOne);
	    return result;
	}
	
	public List<Double> mapToList(Map<Metric,Double> map) {
		List<Double> resList = new ArrayList<Double>();
		for (Metric m : metricOrdering) {
			Double res = map.get(m);
			resList.add(res);
		}
		return resList;
	}
	
	public int compareMetrics(Map<Metric,Double> solution1, Map<Metric,Double> solution2) {
		List<Double> sol1 = mapToList(solution1);
		List<Double> sol2 = mapToList(solution2);
		return dominanceTest(sol1, sol2);
	}
	
	
	public int compareTo(FuzzingExptResult other) {
		return compareMetrics(this.metrics, other.metrics);
	}
}
