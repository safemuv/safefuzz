package fuzzexperiment.runner.jmetal;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.uma.jmetal.operator.mutation.MutationOperator;

import com.opencsv.bean.AbstractCsvConverter;

import atlasdsl.Mission;
import fuzzexperiment.runner.jmetal.grammar.Grammar;
import fuzzingengine.FuzzingEngine;

// TODO: check mutation logic against the changes that I made with Simos on github last year
// Things to check: ensuring all strings/options etc are fresh

// I think it has to be ensured in the crossover operation - ensuring creating a fresh one
public class FuzzingSelectionsMutationConditionsOnly extends FuzzingSelectionsMutation {

	private static final long serialVersionUID = 1L;

	FuzzingSelectionsMutationConditionsOnly(Grammar g, Random rng, Mission mission, FuzzingEngine fuzzEngine,
			String mutationLogFileName, double mutationProb) throws IOException {
		super(g, rng, mission, fuzzEngine, mutationLogFileName, mutationProb);
	}

	public ChangeOp selectRandomOperation() {
			return ChangeOp.CHANGE_TIME_SPEC;
	}
}
