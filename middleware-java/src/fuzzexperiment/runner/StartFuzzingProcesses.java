package fuzzexperiment.runner;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.TimeUnit;

import atlasdsl.Mission;
import carsspecific.ros.codegen.ROSCodeGen;
import fuzzexperiment.runner.jmetal.FuzzingSelectionsSolution;
import fuzzexperiment.runner.rmkg.RMKGInterface;
import fuzzingengine.FuzzingEngine;
import fuzzingengine.spec.GeneratedFuzzingSpec;
import middleware.atlascarsgenerator.ConversionFailed;
import utils.ExptHelper;

public class StartFuzzingProcesses {

	private String ABS_SCRIPT_PATH;
	private String ABS_WORKING_PATH;
	public String ABS_MIDDLEWARE_PATH;

	private final static boolean CLEAR_ROS_LOGS_EACH_TIME = true;

	// This is an emergency time cutout if the failsafe is not operating normally
	private static double failsafeTimeLimit = 1000;

	private static void exptLog(String s) {
		System.out.println(s);
	}

	public StartFuzzingProcesses(String bashScriptPath, String workingPath, String middlewarePath) {
		ABS_SCRIPT_PATH = bashScriptPath;
		ABS_WORKING_PATH = workingPath;
		ABS_MIDDLEWARE_PATH = middlewarePath;
	}

	public String getWorkingPath() {
		return ABS_WORKING_PATH;
	}

	private void waitUntilMiddlewareTime(double time, double wallClockTimeOutSeconds) throws FileNotFoundException {
		String pathToFile = ABS_MIDDLEWARE_PATH + "/logs/atlasTime.log";
		String target = Double.toString(time);
		boolean finished = false;
		long timeStart = System.currentTimeMillis();

		BufferedReader reader = new BufferedReader(new FileReader(pathToFile));
		try {
			while (!finished) {
				try {
					System.out.print(".");
					TimeUnit.MILLISECONDS.sleep(1000);
				} catch (InterruptedException e) {
					// Ignore the interrupted exception - will retry soon
				}
				if (((System.currentTimeMillis() - timeStart) / 1000) > wallClockTimeOutSeconds) {
					finished = true;
				}
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
		} finally {
			try {
				reader.close();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}

	public void waitWallClockTime(double waitTimeSeconds) throws InterruptedException {
		long timeMillis = (long) (waitTimeSeconds * 1000.0);
		System.out.println("Waiting for " + timeMillis + " ms");
		Thread.sleep(timeMillis);
	}

	public List<String> codeGenerationROSFuzzing(Mission mission, String filename, Optional<String> generateDir) {
		FuzzingEngine fe = GeneratedFuzzingSpec.createFuzzingEngine(mission, false);
		try {
			ROSCodeGen rgen = new ROSCodeGen(mission, Optional.of(fe), generateDir);
			// Load the CSV file to produce fuzzing key selection records
			fe.setupFromFuzzingFile(filename, mission);
			rgen.convertDSL();
			System.out.println("Code generation completed");
			return rgen.getModifiedConfigFiles();
		} catch (ConversionFailed cf) {
			System.out.println("ERROR: DSL conversion to MOOS representation failed: reason " + cf.getReason());
			return new ArrayList<String>();
		}
	}

	public void sleepHandlingInterruption(long timeMillisecs) {
		try {
			TimeUnit.MILLISECONDS.sleep(timeMillisecs);
		} catch (InterruptedException e) {
			System.out.println("Cancelling sleep after interruption");
		}
	}

	public double doExperimentFromFile(String exptTag, boolean actuallyRun, double timeLimit, String fuzzFilePath,
			Optional<String> scenarioString_o, Optional<Integer> runNum_o, String launchBashScript,
			boolean useLaunchers, boolean useCustomLauncher) throws InterruptedException, IOException {
		double returnValue = 0;

		if (actuallyRun) {
			exptLog("Starting ROS/SAFEMUV launch scripts");

			if (useLaunchers) {
				// If no scenario is supplied, use the original launcher, which does not
				// generate new launch files
				if (scenarioString_o.isPresent() && runNum_o.isPresent() && useCustomLauncher) {
					String scenarioIDString = scenarioString_o.get();
					int runNum = runNum_o.get();

					String launchArg = scenarioIDString.toLowerCase() + "_" + String.valueOf(runNum);
					ExptHelper.startCmd(ABS_WORKING_PATH, "./custom_" + launchBashScript + " " + launchArg);
				} else {
					System.out.println("ABS_WORKING_PATH = " + ABS_WORKING_PATH);
					System.out.println("launchBashScript = " + launchBashScript);
					// ExptHelper.runScriptNewWithBash(ABS_WORKING_PATH, "./" + launchBashScript);
					ExptHelper.startCmd(ABS_WORKING_PATH, "./" + launchBashScript);
				}
			}

			// TODO: can we replace this delay with checking ROS status to launch the
			// middleware
			// TODO: check the timing when starting a lab experiment
			if (useLaunchers)
				sleepHandlingInterruption(30000);
		}

		String[] paths = fuzzFilePath.split("/");
		String fuzzFile = paths[paths.length - 1];
		System.out.println("Record rosbag: " + fuzzFile);
		String bagFile = "bag_" + fuzzFile + ".bag";
		ExptHelper.runScriptNew(ABS_WORKING_PATH, "./record_rosbag.sh", bagFile);

		System.out.println("Running middleware with " + fuzzFilePath);
		ExptHelper.runScriptNew(ABS_WORKING_PATH, "./start_middleware.sh", fuzzFilePath);
		// Version from JGEA: ExptHelper.runCommandQuitTimeout(ABS_WORKING_PATH,
		// "./start_middleware.sh", fuzzFilePath, 10000);

		String[] middlewareOpts = { "nofault", "nogui" };

		// This assumes that the mission time is at least 20 seconds, and gives time for
		// the middleware to start
		sleepHandlingInterruption(20000);

		// Wait until the end condition for the middleware
		waitUntilMiddlewareTime(timeLimit, failsafeTimeLimit);
		// waitWallClockTime(timeLimit);
		exptLog("Middleware end time reached");

		// TODO: ensure simulation/SAFEMUV state is properly cleared
		if (CLEAR_ROS_LOGS_EACH_TIME) {
			ExptHelper.startCmd(ABS_WORKING_PATH, "terminate_clear_logs.sh");
		} else {
			ExptHelper.startCmd(ABS_WORKING_PATH, "terminate.sh");
		}
		exptLog("Kill MOOS / Java processes command sent");
		sleepHandlingInterruption(40000);
		exptLog("Destroy commands completed");

		returnValue = 0;

		if (actuallyRun) {
			exptLog("Waiting to restart experiment");
			// Wait 10 seconds before ending
			try {
				TimeUnit.MILLISECONDS.sleep(10000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

		return returnValue;
	}

	public void compileLoader() throws IOException {
		ExptHelper.startScript(ABS_SCRIPT_PATH, "compile_dsl_loader.sh");
	}

	public void cleanRun(Mission baseMission, String file) throws IOException {
		ExptHelper.startCmd(ABS_WORKING_PATH, "temp_clean_config_files.sh");
	}

	public void runExptProcesses(String exptTag, Mission baseMission, String csvFileName, double timeLimit,
			FuzzingSelectionsSolution solution, boolean regenerateScenarios, boolean runSimLaunchers, int runNum)
			throws InterruptedException, IOException {
		boolean actuallyRun = true;
		String launchScript = baseMission.getLaunchBashScript();
		// TODO: do we need to call cleanRun here
		if (regenerateScenarios) {
			List<String> fuzzTopicList = RMKGInterface.getFuzzTopicListFromScen(solution);

			// TODO: set these as discussed in the meeting on Friday
			String scenarioDirName = exptTag;
			// Generate the ROS configuration files, e.g. modified launch scripts, YAML
			// config files etc for this CSV definition experimental run
			final String TEMP_WRITTEN_PATH_DIR = "/tmp/ROS_config_files/";

			List<String> modifiedTempFiles = codeGenerationROSFuzzing(baseMission, csvFileName,
					Optional.of(TEMP_WRITTEN_PATH_DIR));
			// RMKGInterface.generateLaunchScriptsRMKG_ROS(ABS_WORKING_PATH,
			// scenarioDirName, fuzzTopicList, modifiedTempFiles, scenarioDirName);
			// doExperimentFromFile(exptTag, actuallyRun, timeLimit, csvFileName,
			// Optional.of(scenarioDirName), launchScript);
		} else {
			// Not regenerating scenarios
			codeGenerationROSFuzzing(baseMission, csvFileName, Optional.empty());
			doExperimentFromFile(exptTag, actuallyRun, timeLimit, csvFileName, Optional.of(exptTag),
					Optional.of(runNum), launchScript, runSimLaunchers, false);
		}
	}
}
