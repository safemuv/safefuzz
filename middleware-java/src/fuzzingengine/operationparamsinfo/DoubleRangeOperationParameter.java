package fuzzingengine.operationparamsinfo;

public class DoubleRangeOperationParameter extends DoubleOperationParameter {
	private double lower;
	private double upper;

	public DoubleRangeOperationParameter(String name, double lower, double upper) {
		super(name);
		this.lower = lower;
		this.upper = upper;
	}

	// TODO: random
	public Object getSpecificValue() {
		return lower;
	}
}
