package exptrunner.operations;

import java.io.FileWriter;
import java.io.IOException;

import fuzzingengine.FuzzingSelectionRecord;

public abstract class MutationOperation {
	protected FileWriter mutationLog;
	
	// Performing the operation in place
	public abstract void perform(FuzzingSelectionRecord orig) throws IOException;
	public abstract String name();
	
	public MutationOperation(FileWriter mutationLog) {
		this.mutationLog = mutationLog;
	}
}