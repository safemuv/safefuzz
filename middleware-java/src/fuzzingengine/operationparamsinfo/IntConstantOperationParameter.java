package fuzzingengine.operationparamsinfo;

public class IntConstantOperationParameter extends IntOperationParameter {
	private int value;
	
	public IntConstantOperationParameter(String name, int value) {
		super(name);
		this.value = value;
	}

	Object getSpecificValue() {
		return value;
	}
}
