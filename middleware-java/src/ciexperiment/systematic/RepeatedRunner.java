package ciexperiment.systematic;

import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
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
import ciexperiment.runner.RunExperiment;
import faultgen.InvalidFaultFormat;
import utils.ExptHelper;

public class RepeatedRunner {
	private static final String EMF_OUTPUT_PATH = "/home/atlas/atlas/atlas-middleware/middleware-java/src/atlasdsl/loader/GeneratedDSLLoader.java";

	public static void runFixedCIExpt(ExptParams eparams, String exptTag, boolean actuallyRun, double timeLimit) throws InterruptedException, IOException {
		
		GenerateModelsExecutor modelLoader = new GenerateModelsExecutor();
		// The core logic for the loop
		while (!eparams.completed()) {
			eparams.printState();
			// Modify the mission from the parameters - and load the modified mission file here
			try {
				Optional<String> file_opt = modelLoader.transformModel("test-experiment-models/mission-basis.model");
				if (file_opt.isPresent()) {
					String file = file_opt.get();
					modelLoader.executeEGL(file, EMF_OUTPUT_PATH);
					// TODO: mission loader - compile it
					RunExperiment.compileLoader();
					Mission mission = getCurrentMission();
					RunExperiment.doExperimentFromFile(mission, exptTag, actuallyRun, timeLimit);
					eparams.logResults("/home/jharbin/academic/atlas/atlas-middleware/expt-working/logs");
					eparams.advance();
				}
			} catch (InvocationTargetException | InterruptedException e) {
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
			} catch (DSLLoadFailed e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}
	
	private static Mission getCurrentMission() throws DSLLoadFailed {
		DSLLoader dl = new GeneratedDSLLoader();
		return dl.loadMission();
	}

	public static void runRepeatedCIExperiment(List<Metrics> metricList, String fileTag, int runCount) {
		DSLLoader loader = new GeneratedDSLLoader();

		try {
			Mission baseMission = loader.loadMission();
			double runTime = baseMission.getEndTime();
			String fileName = new SimpleDateFormat("yyyyMMddHHmm").format(new Date());
			FileWriter tempLog = new FileWriter("tempLog-" + fileName + ".res");
			MetricsProcessing mp = new MetricsProcessing(baseMission, metricList, tempLog);
			String resFileName = "ciexpt-"+fileTag+".res";
			// TODO: register the CIExpt model content in the experiment params
			ExptParams ep = new RepeatSingleExpt(mp, runTime, runCount, baseMission, resFileName);
			runFixedCIExpt(ep, resFileName, true, runTime);
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
