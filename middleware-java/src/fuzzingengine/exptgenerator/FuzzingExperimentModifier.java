package fuzzingengine.exptgenerator;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Random;

import atlasdsl.Mission;
import fuzzexperiment.runner.metrics.Metric;
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

public class FuzzingExperimentModifier extends FuzzingExperimentGenerator {

	private final double DEFAULT_PROB_OF_INCLUDING_VARIABLE = 0.5;
	private final double DEFAULT_PROB_OF_INCLUDING_ROBOT = 0.5;

	Random rng;
	private Mission mission;
	private FuzzingEngine fuzzEngine;

	public enum ChangeOp {
		SHIFT_TIME, CHANGE_PARAM,
	}

	public FuzzingExperimentModifier(Mission mission) {
		super(mission);
	}

	public List<FuzzingSelectionRecord> generateExperimentBasedUpon(String newFile, List<FuzzingSelectionRecord> basis,
			Map<Metric, Double> res) {
		// Modify the timings or parameters of one of the values
		FuzzingSelectionRecord m;
		try {
			m = selectRandomElementFrom(basis);

			// TODO: replace with random selection
			ChangeOp operation = ChangeOp.SHIFT_TIME;

			if (operation == ChangeOp.SHIFT_TIME) {
				m = adjustTime(m);
			}

			if (operation == ChangeOp.CHANGE_PARAM) {

			}

			outputAsCSV(newFile, basis);

		} catch (ListHasNoElement e) {
			System.out.println("generateExperimentBasedUpon - empty original experiment");
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return basis;
	}

	private FuzzingSelectionRecord adjustTime(FuzzingSelectionRecord m) {
		double startTime = getStartTime(Optional.empty());
		double endTime = getStartTime(Optional.empty());
		m.setStartTime(startTime);
		m.setEndTime(endTime);
		return m;
	}
}
