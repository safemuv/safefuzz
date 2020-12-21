package utils.binarymodify;

public class IncorrectStringLength extends Exception {
	private static final long serialVersionUID = 1L;
	private String v1;
	private String v2;

	public IncorrectStringLength(String v1, String v2) {
		this.v1 = v1;
		this.v2 = v2;
	}
}