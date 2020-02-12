package atlasdsl.faults;

import java.util.Optional;

import atlasdsl.*;

public class MutateMessage extends MessageImpact {
	private MessageField affectedField;
	private Optional<SubFieldSpec> subfield;
	private MessageChange newValue;

	public Object applyImpact(Object orig) {
		// TODO: Need to check subfield here 
		return newValue.apply(orig);
	}
}
