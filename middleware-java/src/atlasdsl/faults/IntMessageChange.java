package atlasdsl.faults;

import java.util.Optional;

public class IntMessageChange extends MessageChange {
	private Optional<Integer> absoluteValue;
	private int incrementValue;

	private IntMessageChange() {
		
	}
	
	public static IntMessageChange forAbsolute(int v) {
		IntMessageChange i = new IntMessageChange();
		i.absoluteValue = Optional.of(v);
		return i;
	}
	
	public static IntMessageChange forIncrement(int dv) {
		IntMessageChange i = new IntMessageChange();
		i.incrementValue = dv;
		return i;
	}
	
	public Object apply(Object ov) {
		int v = (Integer)ov;
		if (absoluteValue.isPresent()) {
			return absoluteValue.get();
		} else {
			return v + incrementValue;
		}
	}
}
