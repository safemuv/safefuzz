// Commented out until the JAR export is working
//
//package exptrunner.systematic;
//
//import java.io.FileNotFoundException;
//import java.io.FileWriter;
//import java.io.IOException;
//import java.text.SimpleDateFormat;
//import java.util.ArrayList;
//import java.util.Comparator;
//import java.util.Date;
//import java.util.HashMap;
//import java.util.List;
//import java.util.Map.Entry;
//import java.util.Optional;
//import java.util.stream.Collectors;
//
//import atlasdsl.Mission;
//import atlasdsl.Robot;
//import atlasdsl.faults.Fault;
//import atlasdsl.loader.DSLLoadFailed;
//import atlasdsl.loader.DSLLoader;
//import atlasdsl.loader.GeneratedDSLLoader;
//import atlassharedclasses.FaultInstance;
//import exptrunner.metrics.MetricsProcessing;
//import exptrunner.runner.RunExperiment;
//import exptrunner.jmetal.FuzzingSelectionsSolution;
//import exptrunner.metrics.Metrics;
//
//public class SystematicRunner {
//	private static final double HIGH_RESULT_PROPORTION = 0.1;
//	private static final double LOW_RESULT_PROPORTION = 0.1;
//	private static final int BEST_VERIFY_COUNT = 0;
//	private static final int WORST_VERIFY_COUNT = 0;
//	
//	//public static void runGeneralExpt(Mission mission, ExptParams eparams, String exptTag, boolean actuallyRun, double timeLimit) throws InterruptedException, IOException {
//		//while (!eparams.completed()) {
////			eparams.printState();
//			// TODO: create FuzzingSetSolution representing the config
//			//List<FuzzingSolutionSets> fis = eparams.specificSolution();
//			//RunExperiment.doExperiment(mission, exptTag, fis, actuallyRun, timeLimit);
//			//eparams.logResults("/home/jharbin/academic/atlas/atlas-middleware/expt-working/logs");
////			eparams.advance();
////		}
//	//}
//	
//
//	
//	public static Optional<HashMap<FuzzingSelectionsSolution,Double>> runCoverage(List<Metrics> metricList, Mission mission, String faultName, double additionalDataVal) {
//		try {
//			Optional<Fault> f_o = mission.lookupFaultByName(faultName);
//			if (f_o.isPresent()) {
//				String resFileName = faultName;
//				Fault f = f_o.get();
//				double runTime = mission.getEndTime();
//				Optional<String> speedOverride_o = Optional.of(Double.toString(additionalDataVal));
//				resFileName = resFileName + "_coverage_" + additionalDataVal + ".res";
//				FileWriter tempLog = new FileWriter("tempLog-" + faultName + ".res");
//				MetricsProcessing mp = new MetricsProcessing(mission, metricList, tempLog);
//				//ExptParams ep = new SystematicSingleFaultSearch(mp, resFileName, runTime, 0.0, runTime, runTime, 50.0, 0.5, f,
//				//		speedOverride_o, mission);
//				//runGeneralExpt(mission, ep, faultName + "_coverage", false, runTime);
//				//System.out.println("Done");
//				//return Optional.of(ep.returnResultsInfo());
//			}
//		} catch (IOException e) {
//			e.printStackTrace();
//	//	} catch (InterruptedException e) {
//	//		e.printStackTrace();
//	//	}
//		return Optional.empty();
//	//}
//	
//	//public static void runRepeated(List<Metrics> l, List<Fuzzing> res, int runCount) {
////		for (FaultInstanceSetSolution fis : res) {
//			// TODO: produce a name for each and tag
//			// for now, just use the start of the first fault and length
////			List<FaultInstance> finstances = fis.getFaultInstances();
////			FaultInstance fi1 = finstances.get(0);
////			String fileTag = "COVERAGE-" + Double.toString(fi1.getStartTime()) + "-" + Double.toString(fi1.getEndTime());
////			RepeatedRunner.runRepeatedFaultSetFI(l, finstances, fileTag, runCount);
////		}
////	}
//	
//	//public static void main(String[] args) throws JMetalException, FileNotFoundException {
////		DSLLoader loader = new GeneratedDSLLoader();
////	}
//}
