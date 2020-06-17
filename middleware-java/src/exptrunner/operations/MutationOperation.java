package exptrunner.operations;

import atlassharedclasses.FaultInstance;

public interface MutationOperation {
	public FaultInstance op(FaultInstance orig);
}