package atlasdsl.faults;

public class DoubleMessageChange extends MessageChange {
	private double absoluteValue;
	private double incrementValue;
	private double multFactor;
	
	public Object apply(Object orig) {
		return orig;
	}
}
