package atlascarsgenerator;

public class ConversionFailed extends Exception {
	private static final long serialVersionUID = 1L;
	private ConversionFailedReason reason;
	
	public ConversionFailed(ConversionFailedReason r) {
		this.reason = r;
	}
	
	public ConversionFailedReason getReason() {
		return reason;
	}
}
