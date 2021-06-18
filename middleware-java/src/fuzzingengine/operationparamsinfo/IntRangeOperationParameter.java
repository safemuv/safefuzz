package fuzzingengine.operationparamsinfo;

public class IntRangeOperationParameter extends IntOperationParameter {
	private int lower;
	private int upper;
	
	public IntRangeOperationParameter(String name, int lower, int upper) {
		super(name);
		this.lower = lower;
		this.upper = upper;

	}

	Object getSpecificValue() {
		// TODO: random
		return lower;
	}
}
