package fuzzingengine;

import java.util.Optional;
import middleware.core.CARSVariableUpdate;

public class NumericVariableChangeFuzzingEngine extends FuzzingEngine {
	private static final boolean debugging = false;
	private String regexNumberScanner = "[+-]?([0-9]*\\.)?[0-9]+";
	
	private final double defaultFixedChange = 0.0;
	private String fixedChange = Double.toString(defaultFixedChange);
	private Optional<DoubleLambda> generateDouble = Optional.empty();
	
	public NumericVariableChangeFuzzingEngine(DoubleLambda generateDouble) {
		this.generateDouble = Optional.of(generateDouble);
	}
	
	public NumericVariableChangeFuzzingEngine(double fixedChange) {
		this.fixedChange = Double.toString(fixedChange);
	}

	public String getReplacement(String inValue) {
		return fixedChange;
	}

	public <E> E fuzzTransformEvent(E event) {
		if (event instanceof CARSVariableUpdate) {
			CARSVariableUpdate cvu = (CARSVariableUpdate)event;
			CARSVariableUpdate newUpdate = new CARSVariableUpdate(cvu);
			// TODO: handle checking the type of the update here - ensure we
			// only fuzz a standard operation
			
			// Need to be able to fuzz parts of a structural message
			String change = fixedChange;
			if (generateDouble.isPresent()) {
				DoubleLambda dl = generateDouble.get();
				change = Double.toString(dl.op());
			}
			newUpdate.setValue(change);
			return (E)newUpdate;
		} else {
			return event;
		}
	}
	
}
