package atlasdsl.faults;

import java.util.List;
import java.util.Map;
import java.util.Optional;
import atlasdsl.*;
import middleware.core.ATLASCore;

public class MutateMessage extends MessageImpact {
	private MessageField affectedField;
	private Optional<SubFieldSpec> subfield;
	private MessageChange newValue;
	
	public MutateMessage(Message affectedMessage, Optional<SubFieldSpec> subfield, MessageChange newValue) {
		super(affectedMessage);
		this.subfield = subfield;
		this.newValue = newValue;
	}

	public Object applyImpact(Object orig, Optional<String> additionalData) {
		if (subfield.isPresent()) {
			System.out.println("subfield present");
			if (orig instanceof List) {
				List orig_l = (List)orig;
				SubFieldSpec sf = subfield.get();
				Map.Entry<Integer,Object> indexAndEntry = sf.extract(orig_l);
				Object part = indexAndEntry.getValue();
				int index = indexAndEntry.getKey();
				System.out.println("index=" + index + ",part=" + part);
				part = newValue.apply(part);
				return sf.store(part,orig_l,index);
			} else {
				return orig;
			}
		}
		else return newValue.apply(orig);
	}

	public void immediateEffects(ATLASCore core, Optional<String> additionalData) {
		
	}

	public void completionEffects(ATLASCore core, Optional<String> additionalData) {
		
	}
}
