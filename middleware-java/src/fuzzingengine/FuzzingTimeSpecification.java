package fuzzingengine;

public abstract class FuzzingTimeSpecification {
	public abstract boolean isActiveAtTime(double atTime);
	public abstract String getCSVContents();
	protected abstract FuzzingTimeSpecification dup();
}
