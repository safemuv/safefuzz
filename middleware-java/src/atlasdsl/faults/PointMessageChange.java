package atlasdsl.faults;

import java.util.Optional;

import atlassharedclasses.Point;

public class PointMessageChange extends MessageChange {
	private Optional<Point> absoluteValue;
	private Point incrementValue;

	private PointMessageChange() {
		
	}
	
	public static PointMessageChange forAbsolute(Point p) {
		PointMessageChange i = new PointMessageChange();
		i.absoluteValue = Optional.of(p);
		return i;
	}
	
	public static PointMessageChange forIncrement(Point dp) {
		PointMessageChange i = new PointMessageChange();
		i.incrementValue = dp;
		return i;
	}
	
	public Object apply(Object ov) {
		Point v = (Point)ov;
		if (absoluteValue.isPresent()) {
			return absoluteValue.get();
		} else {
			return v.add(incrementValue);
		}
	}
}
