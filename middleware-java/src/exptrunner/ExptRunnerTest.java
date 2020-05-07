package exptrunner;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

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

	public static void test2() {
		String[] opts = {};
		try {
			ExptHelper.startNewJavaProcess("-jar " + ABS_ATLAS_JAR, "middleware.core.ATLASMain", opts, ABS_WORKING_PATH);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
