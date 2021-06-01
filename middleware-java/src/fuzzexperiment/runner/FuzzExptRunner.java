package fuzzexperiment.runner;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Map;
import java.util.Optional;

import atlasdsl.Mission;
import atlasdsl.loader.DSLLoadFailed;
import atlasdsl.loader.DSLLoader;
import atlasdsl.loader.GeneratedDSLLoader;
import fuzzexperiment.runner.metrics.*;


public class FuzzExptRunner {
	private boolean actuallyRun = true;
	private double timeLimit;
	private MetricHandler mh;
	private ExptParams eparams;
	private String resFileName;
	
	private void setup() throws DSLLoadFailed, IOException {
		DSLLoader loader = new GeneratedDSLLoader();
		Mission baseMission;
		baseMission = loader.loadMission();
		timeLimit = baseMission.getEndTime();
		// TODO: metrics handling - custom metrics
		FileWriter tempLog = new FileWriter("fuzzexpt-templog.log");
		//p.setMetricState(MetricStateKeys.MISSION_END_TIME, baseMission.getEndTime());
		resFileName = "fuzzexpt.res";
	}
	
	public FuzzExptRunner(ExptParams ep, MetricHandler mh) throws DSLLoadFailed, IOException {
		this.eparams = ep;
		this.mh = mh;
		setup();
	}
	
	public void usefakeRun() {
		actuallyRun = false;
	}
	
	public void writeToFile(Map<Metric,Object> results, String metricResFile) {
		// 
	}
	
	public void run() throws InterruptedException, IOException {
		// The core logic for the loop
		
		// Read fuzzing specification?
		// Read the experiment specification as a model - or use the generated CSV creator...
		// In the loop...
		
		while (!eparams.completed()) {
			eparams.printState();
			// Get the CSV file consisting of a fuzzing experiment... internally either generate it 
			// in response to the metrics, or it comes from a constant list
			Optional<String> nextFile_o = eparams.getNextFuzzingCSVFileName();
			if (nextFile_o.isPresent()) {
				String file = nextFile_o.get();
				String exptTag = "exptcsv-" + file;
				System.out.println("Running fuzzing experiments for CSV file name " + file);
				// Generate a CSV file consisting of a fuzzing experiment
				// Invoke the middleware (with the correct mission model and fuzzing spec!)
				// Invoke the CARS / call ROS launch scripts
				// TODO: Terminate the simulation after specific time - need time tracking from the simulations
				StartFuzzingProcesses.doExperimentFromFile(exptTag, actuallyRun, timeLimit, file);
				
				// Assess the metrics (which the user defined using filled-in templates)
				// This should be done by the metrics handler now
				try {
					Map<Metric, Object> metricResults = mh.computeAllOffline("/home/jharbin/academic/atlas/atlas-middleware/expt-working/logs");
					mh.printMetrics(metricResults);
				} catch (MetricComputeFailure e) {
					// If we get metric computation failure, ignore the whole line
					// TODO: log the failure
					e.printStackTrace();
				}
				
			}
					
			eparams.advance();
		}
		System.out.println("Run completed");
	}
}
