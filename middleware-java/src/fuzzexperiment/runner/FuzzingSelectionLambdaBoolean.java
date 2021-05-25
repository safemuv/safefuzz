package fuzzexperiment.runner;

import fuzzingengine.FuzzingSelectionRecord;

public interface FuzzingSelectionLambdaBoolean {
	boolean op(FuzzingSelectionRecord fi);
}
