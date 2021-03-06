package fuzzingengine.exptgenerator;

import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Random;
import java.util.stream.Collectors;

import atlasdsl.Mission;
import atlasdsl.Robot;
import fuzzexperiment.runner.jmetal.grammar.TreeGenerationFailed;
import fuzzingengine.FuzzingEngine;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.FuzzingSelectionRecord;
import fuzzingengine.FuzzingSimMapping;
import fuzzingengine.FuzzingSimMapping.OpParamSetType;
import fuzzingengine.FuzzingSimMapping.VariableSpecification;
import fuzzingengine.FuzzingTimeSpecification;
import fuzzingengine.operationparamsinfo.OperationParameterSet;
import fuzzingengine.TimeSpec;
import fuzzingengine.operations.FuzzingOperation;
import fuzzingengine.spec.GeneratedFuzzingSpec;

public class FuzzingExperimentGenerator {

	private final double DEFAULT_PROB_OF_INCLUDING_VARIABLE = 0.5;
	private final double DEFAULT_PROB_OF_INCLUDING_ROBOT = 0.5;

	protected Random rng = new Random();
	protected Mission mission;
	protected FuzzingEngine fuzzEngine;
	protected FuzzingTimeSpecificationGenerator timeSpecGenerator;

	public FuzzingExperimentGenerator(FuzzingTimeSpecificationGenerator timeSpecGenerator, Mission mission) {
		this.rng = new Random();
		this.mission = mission;
		this.timeSpecGenerator = timeSpecGenerator;
		fuzzEngine = GeneratedFuzzingSpec.createFuzzingEngine(mission, false);
	}

	public void outputAsCSV(String filename, List<FuzzingSelectionRecord> records) throws IOException {
		FileWriter output = new FileWriter(filename);
		for (FuzzingSelectionRecord r : records) {
			String l = r.generateCSVLine();
			output.write(l + "\n");
		}
		output.close();
	}

	public List<FuzzingSelectionRecord> generateExperiment(Optional<String> csvFileName_o) {
		// Load the fuzzing specification from the engine
		FuzzingSimMapping fsm = fuzzEngine.getSimMapping();

		// These records will become part of the CSV file
		List<FuzzingSelectionRecord> records = new ArrayList<FuzzingSelectionRecord>();

		// Iterate over the variable specs
		for (Map.Entry<String, VariableSpecification> vs : fsm.getRecords().entrySet()) {
			double probOfInclusion = getInclusionProb(vs.getValue());

			if (rng.nextDouble() < probOfInclusion) {
				FuzzingKeySelectionRecord ksr;
				try {
					ksr = this.generateVariableEntry(vs.getValue());
					records.add(ksr);
				} catch (OperationLoadFailed e) {
					e.printStackTrace();
				} catch (ListHasNoElement e) {
					System.out.println("Skipping key selection record - " + vs
							+ " - probably no operation types defined in model for this variable?");
					e.printStackTrace();
				} catch (TreeGenerationFailed e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}

		// Convert them to CSV lines
		if (csvFileName_o.isPresent()) {
			String csvFileName = csvFileName_o.get();
			try {
				outputAsCSV(csvFileName, records);
			} catch (IOException e1) {
				e1.printStackTrace();
			}
		}

		return records;
	}

	private double getInclusionProb(VariableSpecification vs) {
		Optional<Double> prob_o = vs.getFuzzProb();
		if (prob_o.isPresent()) {
			Double prob = prob_o.get();
			return prob;
		} else {
			return DEFAULT_PROB_OF_INCLUDING_VARIABLE;
		}
	}

	protected <E> E selectRandomElementFrom(List<E> l, String what) throws ListHasNoElement {
		int length = l.size();
		if (length == 0) {
			throw new ListHasNoElement(what); 
		} else {
			int i = rng.nextInt(length);
			return l.get(i);
		}
	}

	protected List<String> getRandomParticipantsFromMission() {
		List<String> participants = new ArrayList<String>();
		for (Robot r : mission.getAllRobots()) {
			if (rng.nextDouble() < DEFAULT_PROB_OF_INCLUDING_ROBOT) {
				participants.add(r.getName());
			}
		}
		return participants;
	}

	public FuzzingOperation loadOperationFromParams(String operationClassName, String params)
			throws OperationLoadFailed {
		try {
			Class<?> c = Class.forName("fuzzingengine.operations." + operationClassName);
			System.out.println("className for operation " + operationClassName);
			Method method = c.getDeclaredMethod("createFromParamString", String.class);
			Object res = method.invoke(null, params);
			if (res instanceof FuzzingOperation) {
				return (FuzzingOperation) res;
			}
		} catch (ClassNotFoundException e) {
			e.printStackTrace();
		} catch (NoSuchMethodException e) {
			e.printStackTrace();
		} catch (SecurityException e) {
			e.printStackTrace();
		} catch (IllegalAccessException e) {
			e.printStackTrace();
		} catch (IllegalArgumentException e) {
			e.printStackTrace();
		} catch (InvocationTargetException e) {
			e.printStackTrace();
		}
		throw new OperationLoadFailed();
	}

	protected String paramsAsString(List<Object> object) {
		List<String> strList = object.stream().map(e -> e.toString()).collect(Collectors.toList());
		// TODO: merge the specificOpParams with pipes
		String paramStr = String.join("|", strList);
		return paramStr;
	}

	protected double getStartTime(Optional<TimeSpec> ts_o) {
		double startLimit;

		if (ts_o.isPresent()) {
			startLimit = ts_o.get().getEndTime();
		} else {
			startLimit = mission.getEndTime();
		}

		return startLimit * rng.nextDouble();
	}

	protected double getEndTime(Optional<TimeSpec> ts_o, double startTime) {
		double endLimit;

		if (ts_o.isPresent()) {
			endLimit = ts_o.get().getEndTime();
		} else {
			endLimit = mission.getEndTime();
		}

		double endTime = (endLimit - startTime) * rng.nextDouble() + startTime;
		return endTime;
	}
	
	protected FuzzingKeySelectionRecord generateVariableEntry(VariableSpecification var)
			throws OperationLoadFailed, ListHasNoElement {
		// Select an operation parameter set to use
		List<OpParamSetType> opsetTypes = var.getOperationParamSets();
		if (opsetTypes.size() == 0) {
			throw new ListHasNoElement("opsetType in generateVariableEntry");
		} else {
			OpParamSetType opsetType = selectRandomElementFrom(opsetTypes, "opsetType in generateVariableEntry");
			OperationParameterSet opset = opsetType.getOpset();
			String subSpec = opsetType.getSubSpec();

			List<Object> specificOpParams = opset.generateSpecific();

			// Set up the timing using the injected generator
			FuzzingTimeSpecification fts = timeSpecGenerator.gen(var);

			// Get a random list of participants from the mission
			List<String> participants = getRandomParticipantsFromMission();

			// TODO: merge the specificOpParams with pipes
			String paramStr = paramsAsString(specificOpParams);
			FuzzingOperation op = loadOperationFromParams(opset.getOperationClassName(), paramStr);

			FuzzingKeySelectionRecord ksr = new FuzzingKeySelectionRecord(var.getVariable(),
					var.getReflectionName_opt(), var.getComponent(), var.getRegexp(), subSpec, op, participants, fts);
			ksr.setParams(specificOpParams);
			return ksr;
		}
	}
}
