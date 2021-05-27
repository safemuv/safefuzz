package fuzzexperiment.runner;

import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.net.URISyntaxException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Optional;

import org.eclipse.epsilon.egl.exceptions.EglRuntimeException;
import org.eclipse.epsilon.eol.exceptions.models.EolModelLoadingException;

import atlasdsl.Mission;
import atlasdsl.loader.DSLLoadFailed;
import atlasdsl.loader.DSLLoader;
import atlasdsl.loader.GeneratedDSLLoader;
import exptrunner.metrics.Metrics;
import exptrunner.metrics.MetricsProcessing;
import exptrunner.metrics.MetricsProcessing.MetricStateKeys;
import ciexperiment.runner.RunExperiment;
import faultgen.InvalidFaultFormat;

public class RepeatedRunner {
	private final String EMF_OUTPUT_PATH = "/home/atlas/atlas/atlas-middleware/middleware-java/src/atlasdsl/loader/GeneratedDSLLoader.java";
	
	private boolean actuallyRun;
	private double timeLimit;
	
	public void runFuzzingExptLoop(ExptParams eparams) throws InterruptedException, IOException {
		// The core logic for the loop
		
		// Read fuzzing specification?
		// Read the experiment specification as a model - or use the generated CSV creator...
		// In the loop...
		
		while (eparams.completed()) {
			// Generate a CSV file consisting of a fuzzing experiment
			// Invoke the middleware (with the correct mission model and fuzzing spec!)
			// Invoke the CARS / call ROS launch scripts
			// ... need to launch Docker and copy in shared containers from the outside 
			// Terminate the simulation after specific time
			// Assess the metrics (which the user computed from filled-in templates)
			
			eparams.printState();
			// Modify the mission from the parameters - and load the modified mission file here
			Optional<String> nextFile_o = eparams.getNextFileName();
			if (nextFile_o.isPresent()) {
				String file = nextFile_o.get();
				String exptTag = "exptcsv-" + file;
				System.out.println("Running fuzzing experiments for CSV file name " + file);
				RunFuzzingExperiment.doExperimentFromFile(exptTag, actuallyRun, timeLimit);
				eparams.logResults("/home/jharbin/academic/atlas/atlas-middleware/expt-working/logs", file);
			}
					
			eparams.advance();
		}
	}
	
	private Mission getCurrentMission() throws DSLLoadFailed {
		DSLLoader dl = new GeneratedDSLLoader();
		return dl.loadMission();
	}

	public void runFuzzExperiment(String sourceModelFile, List<Metrics> metricList, String fileTag, List<String> ciOptions) {
		DSLLoader loader = new GeneratedDSLLoader();

		try {
			Mission baseMission = loader.loadMission();
			double timeLimit = baseMission.getEndTime();
			// TODO: metrics handling - custom metrics
			FileWriter tempLog = new FileWriter("fuzzexpt-templog.log");
			MetricsProcessing mp = new MetricsProcessing(metricList, tempLog);
			mp.setMetricState(MetricStateKeys.MISSION_END_TIME, baseMission.getEndTime());
			
			String resFileName = "fuzzexpt-"+fileTag+".res";
			System.out.println("Starting experiment set");
			ExptParams ep = new RunOnSetOfSolutions(mp, resFileName);
			runFuzzingExptLoop(ep);
			
			System.out.println("Done");
		} catch (DSLLoadFailed e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		} catch (InterruptedException e) {
			e.printStackTrace();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
