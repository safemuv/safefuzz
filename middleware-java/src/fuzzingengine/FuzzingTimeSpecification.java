package fuzzingengine;

import java.util.Optional;

public abstract class FuzzingTimeSpecification {
	public abstract boolean isActiveAtTime(double time, String vehicle);
	public abstract String getCSVContents();
	public abstract String getCSVRecordTag();
	protected abstract FuzzingTimeSpecification dup();
	public abstract void validateSpecification() throws InvalidSpecification;
	
	// Only the fixed time specification have a static length.
	// The others return empty
	public Optional<Double> getStaticLength() {
		return Optional.empty();
	}
}
