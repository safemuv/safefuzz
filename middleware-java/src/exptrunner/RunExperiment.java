package exptrunner;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

import atlasdsl.loader.*;
import faultgen.*;
import atlasdsl.*;

public class RunExperiment {

	private final static String ABS_SCRIPT_PATH = "/home/jharbin/academic/atlas/atlas-middleware/bash-scripts/";
	private final static String ABS_WORKING_PATH = "/home/jharbin/academic/atlas/atlas-middleware/expt-working/";
	private final static String ABS_MIDDLEWARE_PATH = "/home/jharbin/academic/atlas/atlas-middleware/expt-working/";
	private final static String ABS_ATLAS_JAR = "/home/jharbin/academic/atlas/atlas-middleware/expt-jar/atlas.jar";
	private final static String ABS_MOOS_PATH = "/home/jharbin//academic/atlas/atlas-middleware/middleware-java/moos-sim/";

	private final static String JVM_PATH = "/usr/bin/java";

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

	public static Process startNewJavaProcess(final String optionsAsString, final String mainClass,
			final String[] arguments, String path) throws IOException {

		ProcessBuilder processBuilder = createJVMProcess(optionsAsString, mainClass, arguments);
		processBuilder.directory(new File(path));
		Process process = processBuilder.start();
		
		return process;
	}

	// https://wiki.cantara.no/display/dev/Start+a+new+JVM+from+Java
	private static ProcessBuilder createJVMProcess(final String optionsAsString, final String mainClass,
			final String[] arguments) {
		String jvm = System.getProperty("java.home") + File.separator + "bin" + File.separator + "java";
		String classpath = System.getProperty("java.class.path");

		String[] options = optionsAsString.split(" ");
		List<String> command = new ArrayList<String>();
		command.add(jvm);
		command.addAll(Arrays.asList(options));
		command.add(mainClass);
		command.addAll(Arrays.asList(arguments));

		ProcessBuilder processBuilder = new ProcessBuilder(command);
		Map<String, String> environment = processBuilder.environment();
		environment.put("CLASSPATH", classpath);
		return processBuilder;
	}

	private static Process startScript(String path, String scriptFile) throws IOException {
		exptLog("starting script");
		ProcessBuilder pb = new ProcessBuilder("/bin/bash", "-c", path + "/" + scriptFile);
		pb.directory(new File(path));
		Process proc = pb.start();
		return proc;
	}
	
	private static Process startCmd(String path, String cmd) throws IOException {
		exptLog("starting script");
		ProcessBuilder pb = new ProcessBuilder("/bin/bash", "-c", cmd);
		pb.directory(new File(path));
		Process proc = pb.start();
		return proc;
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
					String[] scriptOpts = {};
					File moosDir = new File(ABS_MOOS_PATH);
					startScript("/home/jharbin/academic/atlas/atlas-middleware/middleware-java/moos-sim", "launch_shoreside.sh");
					startScript("/home/jharbin/academic/atlas/atlas-middleware/middleware-java/moos-sim", "launch_ella.sh");
					startScript("/home/jharbin/academic/atlas/atlas-middleware/middleware-java/moos-sim", "launch_frank.sh");
					startScript("/home/jharbin/academic/atlas/atlas-middleware/middleware-java/moos-sim", "launch_gilda.sh");
					startScript("/home/jharbin/academic/atlas/atlas-middleware/middleware-java/moos-sim", "launch_henry.sh");
					
					exptLog("Started MOOS launch scripts");
					// sleep until MOOS is ready?
					TimeUnit.MILLISECONDS.sleep(1000);

					String[] middlewareCmd = { JVM_PATH };
					String[] middlewareOpts = { faultInstanceFileName };
					File workingDir = new File(ABS_WORKING_PATH);
					middleware = startNewJavaProcess("-jar " + ABS_ATLAS_JAR, "middleware.core.ATLASMain", middlewareOpts, ABS_WORKING_PATH);

					// sleep until the middleware is ready, then start the CI
					TimeUnit.MILLISECONDS.sleep(1000);

					String[] ciCmd = { JVM_PATH };
					String[] ciOpts = {};
					ci = startNewJavaProcess("-jar " + ABS_ATLAS_JAR,	"atlascollectiveintgenerator.runner.CollectiveIntRunner", ciOpts, ABS_WORKING_PATH);

					// wait until the end condition for the middleware - time elapsed?
					waitUntilMiddlewareTime(eparams.getTimeLimit());
					exptLog("Middleware end time reached");
					r.exec(ABS_SCRIPT_PATH + "terminate-moos.sh");
					exptLog("Kill MOOS processes command sent");
					exptLog("Destroying middleware and CI processes");
					middleware.destroy();
					ci.destroy();
					exptLog("Destroy commands completed");

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

	private static void test2() {
		String[] opts = {};
		try {
			startNewJavaProcess("-jar " + ABS_ATLAS_JAR, "middleware.core.ATLASMain", opts, ABS_WORKING_PATH);
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
