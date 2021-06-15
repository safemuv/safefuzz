package fuzzexperiment.runner.jmetal;

import fuzzingengine.FuzzingSelectionRecord;

public interface FuzzingSelectionLambdaBoolean {
	boolean op(FuzzingSelectionRecord fi);
}
