package exptrunner.operations;

import fuzzingengine.FuzzingSelectionRecord;

public interface CreationOperation {
	public FuzzingSelectionRecord op(int index);
}