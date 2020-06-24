package exptrunner;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Optional;
import atlasdsl.Mission;
import atlasdsl.faults.Fault;
import atlasdsl.loader.DSLLoadFailed;
import atlasdsl.loader.DSLLoader;
import atlasdsl.loader.GeneratedDSLLoader;
import faultgen.FaultFileCreator;

public class ExptRunnerTest {
	private final static String ABS_ATLAS_JAR = "/home/jharbin/academic/atlas/atlas-middleware/expt-jar/atlas.jar";
	private final static String ABS_WORKING_PATH = "/home/jharbin/academic/atlas/atlas-middleware/expt-working/";
	
	public static void test() {
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

			System.out.println("Result produced: " + res);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public static void setupFaultFile() throws DSLLoadFailed {
		DSLLoader loader = new GeneratedDSLLoader();
		Mission mission = loader.loadMission();
		FaultFileCreator ffc = new FaultFileCreator(mission, "/home/jharbin/academic/atlas/atlas-middleware/middleware-java/");
		ffc.generateFaultListFromScratch("testfaults.fif");
	}

	public static void test2() {
		String[] opts = {};
		try {
			ExptHelper.startNewJavaProcess("-jar " + ABS_ATLAS_JAR, "middleware.core.ATLASMain", opts, ABS_WORKING_PATH);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public static void testExpSequence() {
		try {
			DSLLoader loader = new GeneratedDSLLoader();
			Mission mission = loader.loadMission();
			Optional<Fault> f_o = mission.lookupFaultByName("SPEEDFAULT");
			if (f_o.isPresent()) {
				Fault f = f_o.get();
				//TODO: Read args to launch appropriate experiment
				ExptParams ep = new SingleFaultCoverageExpt("SPEEDFAULT.res", 999.0, 0.0, 999.0, 999.0, 100.0, 0.5, f, Optional.empty());
				
				while (!ep.completed()) {
					ep.printState();
					ep.advance();	
				}
			}
		} catch (DSLLoadFailed e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	public static void main(String [] args) {
		try {
			setupFaultFile();
		} catch (DSLLoadFailed e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}