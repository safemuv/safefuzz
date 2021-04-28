package ciexperiment.systematic;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.net.URISyntaxException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.eclipse.epsilon.egl.exceptions.EglRuntimeException;
import org.eclipse.epsilon.eol.exceptions.models.EolModelLoadingException;

public class ResultsAnalyser {
	private static final String EMF_OUTPUT_PATH = "/home/atlas/atlas/atlas-middleware/middleware-java/src/atlasdsl/loader/GeneratedDSLLoader.java";
	
	public static ModelsTransformer modelTransformer = new ModelsTransformer();
	public static ModelEGLExecutor modelExecutor = new ModelEGLExecutor();
	private static final int DELAY = 100;
	
	private static Pattern nodeReportScanner = Pattern.compile("SENSORNAME=([^,]+)");

	private static List<String> getSensorNames() throws FileNotFoundException {
		List<String> sensorNames = new ArrayList<String>();
		File f = new File(EMF_OUTPUT_PATH);
		Scanner s = new Scanner(f);
		while (s.hasNextLine()) {
			String line = s.nextLine();
			if (line.contains("SENSORNAME")) {
				Matcher m = nodeReportScanner.matcher(line);
				if (m.find()) {
					sensorNames.add(m.group(1));
				}
			}
		}
		s.close();
		return sensorNames;
	}
	
	public static void runAnalysisLoop(String sourceResFile, String outputFile, String ciChosen, SpecMatcher spec) throws InterruptedException, IOException, EolModelLoadingException, EglRuntimeException, URISyntaxException {
		File f = new File(sourceResFile);
		Scanner s = new Scanner(f);
		FileWriter fOut = new FileWriter(outputFile);
		while (s.hasNextLine()) {
			String line = s.nextLine();
			String [] fields = line.split(",");
			String modelFile = fields[0];
			String ciName = fields[1];
			 			
			if (spec.lineMatches(fields)) {
				if (ciName.contains(ciChosen)) {
					System.out.println("FOUND MATCH - line " + line);
					// Invoke something that appends the necessary model parameters to the model file 
					modelExecutor.executeEGL(modelFile, EMF_OUTPUT_PATH);
					Thread.sleep(DELAY);
					// 	Scan the generated DSL loader for the "SENSORNAME" lines
					List<String> sensorNames = getSensorNames();
					System.out.println(line + "," + sensorNames + "\n");
					fOut.write(String.join(",", sensorNames) + "\n");
				}
			}
			
		}
		s.close();
		fOut.close();
	}
	
	public static void printAllConfigs(String filepath) {
		try {
			SpecMatcher smAlways = new SpecMatcherAlways();
			runAnalysisLoop(filepath, "allconfigs-standard-ci.res", "ComputerCIshoreside_standard", smAlways);
		} catch (EolModelLoadingException | EglRuntimeException | InterruptedException | IOException
				| URISyntaxException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public static void printConfigsForExpt1(String filepath) {
		try {
			SpecMatcher smOptimal = new SpecMatcherIntValue(3, 0);
			SpecMatcher sm3Missed = new SpecMatcherIntValue(3, 3);
			
			runAnalysisLoop(filepath, "optimal-standard-ci.res", "ComputerCIshoreside_standard", smOptimal);
			runAnalysisLoop(filepath, "3missed-standard-ci.res", "ComputerCIshoreside_standard", sm3Missed);
		} catch (EolModelLoadingException | EglRuntimeException | InterruptedException | IOException
				| URISyntaxException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public static void main(String [] args) {
		String filepath = "/home/atlas/atlas/atlas-middleware/middleware-java/res/ciexpt/2021_04_27/ciexpt-casestudy1.res";
		printAllConfigs(filepath);
		printConfigsForExpt1(filepath);
	}
	
}