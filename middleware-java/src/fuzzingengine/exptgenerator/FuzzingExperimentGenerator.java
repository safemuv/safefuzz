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
import fuzzingengine.FuzzingEngine;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.FuzzingSelectionRecord;
import fuzzingengine.FuzzingSimMapping;
import fuzzingengine.FuzzingSimMapping.OpParamSetType;
import fuzzingengine.FuzzingSimMapping.VariableSpecification;
import fuzzingengine.operationparamsinfo.OperationParameterSet;
import fuzzingengine.operations.FuzzingOperation;
import fuzzingengine.spec.GeneratedFuzzingSpec;

public class FuzzingExperimentGenerator {
	
	private final double PROB_OF_INCLUDING_VARIABLE = 1.0;
	private final double PROB_OF_INCLUDING_ROBOT = 1.0;
	
	Random rng;
	private Mission mission;
	private FuzzingEngine fuzzEngine;

	public FuzzingExperimentGenerator(Mission mission) {
		this.rng = new Random();
		this.mission = mission;
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
		rng = new Random();

		// These records will become part of the CSV file
		List<FuzzingSelectionRecord> records = new ArrayList<FuzzingSelectionRecord>();

		// Iterate over the variable specs
		for (Map.Entry<String, VariableSpecification> vs : fsm.getRecords().entrySet()) {
			if (rng.nextDouble() < PROB_OF_INCLUDING_VARIABLE) {
				FuzzingKeySelectionRecord ksr;
				try {
					ksr = generateVariableEntry(vs.getValue());
					records.add(ksr);
				} catch (OperationLoadFailed e) {
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

	private <E> E selectRandomElementFrom(List<E> ops) {
		int length = ops.size();
		int i = rng.nextInt(length);
		return ops.get(i);
	}
	
	private List<String> getRandomParticipantsFromMission() {
		List<String> participants = new ArrayList<String>();
		for (Robot r : mission.getAllRobots()) {
			if (rng.nextDouble() < PROB_OF_INCLUDING_ROBOT) {
				participants.add(r.getName());
			}
		}
		return participants;
	}
	
	public FuzzingOperation loadOperationFromParams(String operationClassName, String params) throws OperationLoadFailed {
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
	
	private String paramsAsString(List<Object> object) {
		List<String> strList = object.stream().map(e -> e.toString()).collect(Collectors.toList());
		// TODO: merge the specificOpParams with pipes
		String paramStr = String.join("|", strList);
		return paramStr;
	}
	
	private double getStartTime() {
		double startLimit = mission.getEndTime();
		return startLimit * rng.nextDouble();
	}
	
	private double getEndTime(double startTime) {
		double endLimit = mission.getEndTime();
		double endTime = (endLimit - startTime) * rng.nextDouble() + startTime;
		return endTime;
	}
	
	private FuzzingKeySelectionRecord generateVariableEntry(VariableSpecification var) throws OperationLoadFailed {
		// Select an operation parameter set to use
		List<OpParamSetType> opsetTypes = var.getOperationParamSets();
		OpParamSetType opsetType = selectRandomElementFrom(opsetTypes);
		OperationParameterSet opset = opsetType.getOpset();
		String subSpec = opsetType.getSubSpec();
		
		List<Object> specificOpParams = opset.generateSpecific();

		// Set up the timing from the mission constraints - for now, the full time range

		// TODO: get the start time parameters from the model, and generate a random spec within this range
		// Currently it uses a random specification throughout the defined mission
		double startTime = getStartTime();
		double endTime = getEndTime(startTime); 

		// Get a random list of participants from the mission
		List<String> participants = getRandomParticipantsFromMission();
		
		// TODO: merge the specificOpParams with pipes
		String paramStr = paramsAsString(specificOpParams);
		FuzzingOperation op = loadOperationFromParams(opset.getOperationClassName(), paramStr);

		FuzzingKeySelectionRecord ksr = new FuzzingKeySelectionRecord(var.getVariable(), var.getReflectionName_opt(),
				var.getComponent(), var.getRegexp(), subSpec, op, participants, startTime, endTime);
		ksr.setParams(specificOpParams);
		return ksr;
	}
}
