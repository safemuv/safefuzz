package test.logparsing;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import atlasdsl.Mission;
import atlasdsl.loader.DSLLoadFailed;
import atlasdsl.loader.DSLLoader;
import atlasdsl.loader.GeneratedDSLLoader;
import ciexperiment.systematic.ExptParams;
import ciexperiment.systematic.RunOnSetOfModels;
import exptrunner.jmetal.FuzzingSelectionsSolution;
import exptrunner.jmetal.InvalidMetrics;
import exptrunner.metrics.MetricsProcessing;
import exptrunner.metrics.Metrics;

public class TestParseLogs {
	public static void main(String [] args) {
		DSLLoader loader = new GeneratedDSLLoader();
		List<Metrics> metricList = new ArrayList<Metrics>();
		double runTime = 1200.0;
		
		Mission baseMission;
		try {
			baseMission = loader.loadMission();
			FileWriter tempLog = new FileWriter("test-templog.res");
			
			// Load the number of valid and missed detections from the given model
			FuzzingSelectionsSolution s = new FuzzingSelectionsSolution(null, "", true, 0.0);
			MetricsProcessing mp = new MetricsProcessing(baseMission, metricList, tempLog);
			mp.readLogFiles("./temp-logdir", s);
			tempLog.close();
		} catch (DSLLoadFailed e) {
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (InvalidMetrics e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}
}
