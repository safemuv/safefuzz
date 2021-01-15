package fuzzingengine.operations;

public abstract class FuzzingOperation {
	public abstract boolean isEventBased();
	public abstract boolean isValueBased();
	
	// This should really be a static abstract method. But can't create one 
	public static FuzzingOperation createFromParamString(String s) throws CreationFailed {
		throw new CreationFailed("Cannot create FuzzingOperation from string directly");
	}
}
