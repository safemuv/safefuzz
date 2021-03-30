package ciexperiment.systematic;

import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;

import atlasdsl.Mission;
import atlasdsl.loader.DSLLoadFailed;
import atlasdsl.loader.DSLLoader;
import atlasdsl.loader.GeneratedDSLLoader;
import exptrunner.metrics.Metrics;
import exptrunner.metrics.MetricsProcessing;
import ciexperiment.runner.RunExperiment;
import faultgen.InvalidFaultFormat;

public class RepeatedRunner {
	public static void runFixedCIExpt(Mission mission, ExptParams eparams, String exptTag, boolean actuallyRun, double timeLimit) throws InterruptedException, IOException {
		while (!eparams.completed()) {
			eparams.printState();
			RunExperiment.doExperimentFromFile(mission, exptTag, actuallyRun, timeLimit);
			// TODO: path to replace
			eparams.logResults("/home/jharbin/academic/atlas/atlas-middleware/expt-working/logs");
			eparams.advance();
		}
	}
	
	public static void runRepeatedCIExperiment(List<Metrics> metricList, String fileTag, int runCount) {
		DSLLoader loader = new GeneratedDSLLoader();
		Mission mission;

		try {
			mission = loader.loadMission();
			double runTime = mission.getEndTime();
			String fileName = new SimpleDateFormat("yyyyMMddHHmm").format(new Date());
			FileWriter tempLog = new FileWriter("tempLog-" + fileName + ".res");
			MetricsProcessing mp = new MetricsProcessing(mission, metricList, tempLog);
			String resFileName = "ciexpt-"+fileTag+".res";
			// TODO: register the CIExpt model content in the experiment params
			ExptParams ep = new RepeatSingleExpt(mp, runTime, runCount, mission, resFileName);
			runFixedCIExpt(mission, ep, resFileName, true, runTime);
			System.out.println("Done");
		} catch (DSLLoadFailed e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		} catch (InterruptedException e) {
			e.printStackTrace();
		} catch (InvalidFaultFormat e) {
			e.printStackTrace();
		}
	}
	
	public static void main(String[] args) throws FileNotFoundException, InterruptedException {
		List<Metrics> l = new ArrayList<Metrics>();
		l.add(Metrics.PURE_MISSED_DETECTIONS);
		l.add(Metrics.OUTSIDE_REGION_COUNT);
		// TODO: other metrics to track... the total distance robots travel
		// TODO: track the total cost of the configuration
		runRepeatedCIExperiment(l, "ciexpt", 200);
	}
}
