package exptrunner;

import java.io.BufferedReader;

import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.concurrent.TimeUnit;

import atlasdsl.loader.*;
import faultgen.*;
import atlasdsl.*;

public class RunExperiment {

	private final static String ABS_SCRIPT_PATH = "/home/jharbin/academic/atlas/atlas-middleware/bash-scripts/";
	private final static String ABS_WORKING_PATH = "/home/jharbin/academic/atlas/atlas-middleware/expt-working/";
	private final static String ABS_MIDDLEWARE_PATH = "/home/jharbin/academic/atlas/atlas-middleware/expt-working/";
	private final static String ABS_ATLAS_JAR = "/home/jharbin/academic/atlas/atlas-middleware/expt-jar/atlas.jar";

	private static void exptLog(String s) {
		System.out.println(s);
	}

	private static boolean doRandomGeneration() {
		return true;
	}

	private static void waitUntilMiddlewareTime(double time) throws FileNotFoundException {
		String pathToFile = ABS_MIDDLEWARE_PATH + "/logs/atlasTime.log";
		String target = Double.toString(time);
		boolean finished = false;
		BufferedReader reader = new BufferedReader(new FileReader(pathToFile));
		try {
			while (!finished) {
				TimeUnit.MILLISECONDS.sleep(100);
				while (reader.ready()) {
					String line = reader.readLine();
					Double lineVal = Double.valueOf(line);
					exptLog(line + "-" + target);
					if (lineVal >= time) {
						finished = true;
					}
				}
			}
		} catch (IOException e) {
			e.printStackTrace();
		} catch (InterruptedException e) {
			e.printStackTrace();
		} finally {
			try {
				reader.close();
			} catch (IOException e) {

				e.printStackTrace();
			}
		}
	}

	private static void scanExperiment(String exptTag, ExptParams eparams) {
		Process scriptRunner;
		Process middleware;
		Process ci;

		int faultInstanceFileNum = 0;
		Runtime r = Runtime.getRuntime();
		boolean complete = false;

		try {
			DSLLoader loader = new GeneratedDSLLoader();
			Mission mission = loader.loadMission();
			while (!eparams.completed()) {
				// TODO: set up the spec data for the start of the experiment from the
				// experiment plan

				// TODO: generate fresh MOOS code if required - need to specify the MOOSTimeWarp
				// here?

				String faultInstanceFileName = "expt_" + exptTag + Integer.toString(faultInstanceFileNum);
				exptLog("Running experiment with " + faultInstanceFileName);
				// Generate a fault instance file for the experiment according to the experiment
				// parameters
				if (doRandomGeneration()) {
					// TODO: get the faults from the exptParams.specificFaults instead
					FaultFileCreator fc = new FaultFileCreator(mission, ABS_WORKING_PATH);
				}

				try {
					// Launch the MOOS code, middleware and CI as seperate subprocesses
					scriptRunner = r.exec(ABS_SCRIPT_PATH + "launch-moos.sh");
					// sleep until MOOS is ready?
					TimeUnit.MILLISECONDS.sleep(1000);
					middleware = r.exec("java -cp " + ABS_ATLAS_JAR + " middleware.core.ATLASMain " + faultInstanceFileName);
					// sleep until the middleware is ready
					TimeUnit.MILLISECONDS.sleep(1000);
					ci = r.exec("java -cp " + ABS_ATLAS_JAR + " atlascollectiveintgenerator.runner.CollectiveIntRunner");

					// wait until the end condition for the middleware - time elapsed?
					waitUntilMiddlewareTime(eparams.getTimeLimit());
					r.exec(ABS_SCRIPT_PATH + "terminate-moos.sh");
					middleware.destroy();
					ci.destroy();

					// Read and process the result files from the experiment
					eparams.processGoalResults(ABS_WORKING_PATH + "logs/goalLog.log");

					// Modify the experiment spec data (either by using the results for mutation,
					// or according to predefined template)
					eparams.advance();

				} catch (IOException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}

			// TODO: plot the experimental data when complete (matplotlib)

		} catch (DSLLoadFailed e) {
			e.printStackTrace();
		}
	}

	private static void test() {
		Runtime r = Runtime.getRuntime();
		try {
			Process proc = r.exec("ls");
			BufferedReader stdInput = new BufferedReader(new InputStreamReader(proc.getInputStream()));
			BufferedReader stdError = new BufferedReader(new InputStreamReader(proc.getErrorStream()));
			String s = null;
			String res = "";
			while ((s = stdInput.readLine()) != null) {
				res = res + s + "\n";
			}

			exptLog("Result produced: " + res);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public static void main(String[] args) {
		// TODO: Read args to launch appropriate experiment
		ExptParams ep = new SingleFaultCoverageExpt(0.0, 1000.0, 1000.0, 300.0);
		scanExperiment("test", ep);
		exptLog("Done");
	}
}
