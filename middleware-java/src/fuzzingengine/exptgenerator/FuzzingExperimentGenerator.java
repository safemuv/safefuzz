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
import fuzzingengine.TimeSpec;
import fuzzingengine.operationparamsinfo.OperationParameterSet;
import fuzzingengine.operations.FuzzingOperation;
import fuzzingengine.spec.GeneratedFuzzingSpec;

public abstract class FuzzingExperimentGenerator {

	private final double DEFAULT_PROB_OF_INCLUDING_VARIABLE = 0.5;
	private final double DEFAULT_PROB_OF_INCLUDING_ROBOT = 0.5;

	Random rng = new Random();
	private Mission mission;
	protected FuzzingEngine fuzzEngine;

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

	protected abstract FuzzingKeySelectionRecord generateVariableEntry(VariableSpecification var)
			throws OperationLoadFailed, ListHasNoElement;
}
