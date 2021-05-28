package fuzzingengine.operationparamsinfo;

import java.util.ArrayList;
import java.util.List;

public class IntListOperationParameter extends IntOperationParameter {
	List<Integer> values = new ArrayList<Integer>();
	
	IntListOperationParameter(String name) {
		super(name);
	}

	Object getSpecificValue() {
		return null;
	}
}
