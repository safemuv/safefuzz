package exptrunner.operations;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Random;

import fuzzingengine.FuzzingSelectionRecord;

public class MoveTimeStart extends MutationOperation {
	private double maxTimeShift;
	// TODO: inserting random seeds
	private Random rng = new Random();
	
	public MoveTimeStart(FileWriter mutationLog, double maxTimeShift) {
		super(mutationLog);
		this.maxTimeShift = maxTimeShift;
	}
	
	public String name() {
		return "MoveTimeStart";
	} 
	
	public void perform(FuzzingSelectionRecord fs) throws IOException {
		double absTimeShift = (rng.nextDouble() - 0.5) * maxTimeShift * 2;
		System.out.println("Moving fault: absTimeShift = " + absTimeShift);
		mutationLog.write("Moving fault: absTimeShift = " + absTimeShift + "\n");
		fs.absShiftTimes(absTimeShift);
	}
}
