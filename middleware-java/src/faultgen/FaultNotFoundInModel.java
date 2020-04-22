package faultgen;

public class FaultNotFoundInModel extends Exception {
	private static final long serialVersionUID = 1L;
	private String faultName;
	
	FaultNotFoundInModel(String faultName) {
		this.faultName = faultName;
	}
}