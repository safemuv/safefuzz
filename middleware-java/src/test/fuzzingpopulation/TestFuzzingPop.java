package test.fuzzingpopulation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Random;

import fuzzexperiment.runner.*;
import fuzzexperiment.runner.metrics.*;
import fuzzingengine.FuzzingSelectionRecord;

public class TestFuzzingPop {
	public static void main(String [] args) {
		int limit = 30;
		Random rng = new Random();
		FuzzingPopulation pop = new FuzzingPopulation(limit);
		
		List<OfflineMetric> metrics = new ArrayList<OfflineMetric>();
		metrics.add(new AvoidanceViolationsCount());
		metrics.add(new OutsideOfInnerRegionViolations());
		metrics.add(new OutsideOfOuterRegionViolations());
		metrics.add(new SpeedViolationsCount());
		
				
		for (int i = 0; i < limit; i++) {
			HashMap<Metric,Double> mres = new HashMap<Metric,Double>();
			
			for (OfflineMetric m : metrics) {
				mres.put(m, rng.nextDouble());
			}
			
			String file = "file" + i + ".csv";
			List<FuzzingSelectionRecord> kr = new ArrayList<FuzzingSelectionRecord>();
			FuzzingExptResult r = new FuzzingExptResult(kr, file, mres);
			pop.pushToPopulation(r);
		}

		pop.print();
		List<ArrayList<FuzzingExptResult>> rankings = pop.computeRankings();
		
		for (ArrayList<FuzzingExptResult> rank : rankings) {
			System.out.println(rank);
			System.out.println("=============================================================================================");
		}
		
	}
}
