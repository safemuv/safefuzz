package fuzzingengine;

import middleware.core.ATLASCore;

public abstract class FuzzingTimeSpecification {
	public abstract boolean isActiveNow(ATLASCore core);
	public abstract String getCSVContents();
	public abstract String getCSVRecordTag();
	protected abstract FuzzingTimeSpecification dup();
}
