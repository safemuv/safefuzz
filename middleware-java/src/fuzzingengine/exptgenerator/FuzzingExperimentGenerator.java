package fuzzingengine.exptgenerator;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Random;

import atlasdsl.Mission;
import fuzzingengine.FuzzingEngine;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.FuzzingSelectionRecord;
import fuzzingengine.FuzzingSimMapping;
import fuzzingengine.FuzzingSimMapping.VariableSpecification;
import fuzzingengine.operationparamsinfo.OperationParameterSet;

public class FuzzingExperimentGenerator {
	private final double PROB_OF_INCLUDING_VARIABLE = 1.0;
	Random rng;
	private Mission mission;
	
	public FuzzingExperimentGenerator(Mission mission) {
		this.rng = new Random();
		this.mission = mission;
	}
	
	public void outputAsCSV(String filename, List<FuzzingSelectionRecord> records) throws IOException {
		FileWriter output = new FileWriter(filename);
		for (FuzzingSelectionRecord r : records) {
			String l = r.generateCSVLine();
			output.write(l + "\n");
		}
	}
	
	public void generateExperiment(String csvFileName) {
		// Load the fuzzing engine
		FuzzingEngine e = new FuzzingEngine(mission);
		FuzzingSimMapping fsm = e.getSimMapping();
		rng = new Random();
		
		// These records will become part of the CSV file
		List<FuzzingSelectionRecord> records = new ArrayList<FuzzingSelectionRecord>();
		
		// Iterate over the variable specs
		for (Map.Entry<String,VariableSpecification> vs : fsm.getRecords().entrySet()) {
			if (rng.nextDouble() < PROB_OF_INCLUDING_VARIABLE) {
				FuzzingKeySelectionRecord ksr = generateVariableEntry(vs.getValue());
				records.add(ksr);
			}
		}
		
		// Convert them to CSV lines
		try {
			outputAsCSV(csvFileName, records);
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}
	}

	private OperationParameterSet selectRandomElementFrom(List<OperationParameterSet> ops) {
		int length = ops.size();
		int i = rng.nextInt(length);
		return ops.get(i);
	}
	
	private FuzzingKeySelectionRecord generateVariableEntry(VariableSpecification var) {
		// Select an operation parameter set to use
		List<OperationParameterSet> opsets = var.getOperationParamSets();
		OperationParameterSet opset = selectRandomElementFrom(opsets);
		List<Object> res = opset.generateSpecific();
		
		
		// Set up the timing from the mission constraints - for now, the full time range
		String subSpec = "twist.linear";
		
		// TODO: get the start time parameters from the model
		double startTime = 0.0;
		double endTime = mission.getEndTime();
		
		// TODO: Get a random list of participants from the mission
		List<String> participants = new ArrayList<String>();
		
		FuzzingKeySelectionRecord ksr = new FuzzingKeySelectionRecord(var.getVariable(), 
				var.getReflectionName_opt(), var.getComponent(), var.getRegexp(), subSpec, 
				opset.getOperation(), participants, startTime, endTime);
		return ksr;
	}
}
