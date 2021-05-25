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
	private static final String EMF_OUTPUT_PATH = "/home/atlas/atlas/atlas-middleware/middleware-java/src/atlasdsl/loader/GeneratedDSLLoader.java";
	
	public static void runFuzzingExptLoop() throws InterruptedException, IOException {
		// The core logic for the loop
		
		// Read fuzzing specification?
		// Read the experiment specification as a model - or use the generated CSV creator...
		// In the loop...
		// Generate a CSV file consisting of a fuzzing experiment
		// Invoke the middleware (with the correct mission model and fuzzing spec!)
		// Invoke the CARS / call ROS launch scripts
		// ... need to launch Docker and copy in shared containers from the outside 
		// Terminate the simulation after specific time
		// Assess the metrics (which the user computed from filled-in templates)
		
		while (eparams.completed()) {
			eparams.printState();
			// Modify the mission from the parameters - and load the modified mission file here
			try {
				Optional<String> nextFile_o = eparams.getNextFileName();
				if (nextFile_o.isPresent()) {
					String file = nextFile_o.get();
					
					for (String ciOption : ciOptions) {
						System.out.println("Running experiments for model file " + file + " and CI class " + ciOption);
						// Mission loader - recompile it
						generateCSV.info();
						System.out.println("Recompiling loader");
						RunExperiment.compileLoader();
						
						// The newly recompiled loader is not used by this process, only by subprocesses
						// So they see all the changes to the mission
						Thread.sleep(3000);
						System.out.println("Loader recompilation done");
						RunExperiment.doExperimentFromFile(exptTag, actuallyRun, timeLimit, ciOption);
						eparams.logResults("/home/jharbin/academic/atlas/atlas-middleware/expt-working/logs", file, ciOption);
					}
					
					eparams.advance();
				}
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (EolModelLoadingException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (EglRuntimeException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (URISyntaxException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} 
			Thread.sleep(1000);
		}
	}
	
	private static Mission getCurrentMission() throws DSLLoadFailed {
		DSLLoader dl = new GeneratedDSLLoader();
		return dl.loadMission();
	}

	public static void runFuzzExperiment(String sourceModelFile, List<Metrics> metricList, String fileTag, List<String> ciOptions) {
		DSLLoader loader = new GeneratedDSLLoader();

		try {
			Mission baseMission = loader.loadMission();
			double runTime = baseMission.getEndTime();
			String fileName = new SimpleDateFormat("yyyyMMddHHmm").format(new Date());
			FileWriter tempLog = new FileWriter("tempLog-" + fileName + ".res");
			MetricsProcessing mp = new MetricsProcessing(metricList, tempLog);
			mp.setMetricState(MetricStateKeys.MISSION_END_TIME, baseMission.getEndTime());
			String resFileName = "ciexpt-"+fileTag+".res";
			System.out.println("Model generation beginning for " + sourceModelFile);
			List<String> missionFiles = modelTransformer.retriveAllModels(sourceModelFile);
			System.out.println("Model generation completed successfully!");
			Thread.sleep(10000);
			System.out.println("Starting experiment set");
			ExptParams ep = new RunOnSetOfModels(mp, runTime, missionFiles, resFileName);
			runCIExptLoop(ep, resFileName, true, runTime, ciOptions);
			
			System.out.println("Done");
		} catch (DSLLoadFailed e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		} catch (InterruptedException e) {
			e.printStackTrace();
		} catch (InvalidFaultFormat e) {
			e.printStackTrace();
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
