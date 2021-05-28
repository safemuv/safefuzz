package fuzzingengine.operationparamsinfo;

import java.util.ArrayList;
import java.util.List;

import fuzzingengine.operations.FuzzingOperation;

public class OperationParameterSet {
	private String name;
	private String operationClassName;
	List<OperationParameter> params = new ArrayList<OperationParameter>();
	private FuzzingOperation op;
	
	private void loadFuzzingOperation() {
		try {
			Class<?> op = Class.forName("fuzzingengine.operations." + operationClassName);
		} catch (ClassNotFoundException e) {
			e.printStackTrace();
		}
	}
	
	public OperationParameterSet(String name, List<OperationParameter> params, String operationClassName) {
		this.name = name;
		this.params = params;
		this.operationClassName = operationClassName;
		loadFuzzingOperation();
	}
	
	public FuzzingOperation getOperation() {
		return op;
	}
	
	public OperationParameterSet(String name, String operationClassName) {
		this.name = name;
		this.operationClassName = operationClassName;
		loadFuzzingOperation();
	}

	public void addParameter(OperationParameter p) {
		params.add(p);
	}

	public List<Object> generateSpecific() {
		List<Object> res = new ArrayList<Object>();
		for (OperationParameter p : params) {
			res.add(p.getSpecificValue());
		}
		return res;
	}
}