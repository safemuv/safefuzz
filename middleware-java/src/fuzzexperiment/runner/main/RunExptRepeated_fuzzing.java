package fuzzexperiment.runner.main;

import java.io.IOException;

import atlasdsl.Mission;
import atlasdsl.loader.DSLLoadFailed;
import atlasdsl.loader.GeneratedDSLLoader;
import fuzzexperiment.runner.*;
import fuzzexperiment.runner.metrics.*;

public class RunExptRepeated_fuzzing {
	
	public static void main(String [] args) {
		String resFileName = "fuzzexpt-setvelocity057-repeated.res";

		try {
			Mission mission = new GeneratedDSLLoader().loadMission();
			MetricHandler mh = new MetricHandler(mission, resFileName);
			ExptParams ep = new RunRepeated("/home/jharbin/academic/atlas/atlas-middleware/middleware-java/fuzz-configs/setvelocity-057.csv", 30);
			FuzzExptRunner r;
			r = new FuzzExptRunner(ep, mh);
			r.run();
		} catch (DSLLoadFailed | IOException e) {
			e.printStackTrace();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
}
