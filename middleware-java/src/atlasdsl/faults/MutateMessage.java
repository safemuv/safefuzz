package atlasdsl.faults;

import java.util.List;
import java.util.Map;
import java.util.Optional;

import atlasdsl.*;

public class MutateMessage extends MessageImpact {
	private MessageField affectedField;
	private Optional<SubFieldSpec> subfield;
	private MessageChange newValue;

	public Object applyImpact(Object orig) {
		if (subfield.isPresent()) {
			if (orig instanceof List) {
				List orig_l = (List)orig;
				SubFieldSpec sf = subfield.get();
				Map.Entry<Integer,Object> indexAndEntry = sf.extract(orig_l);
				Object part = indexAndEntry.getValue();
				int index = indexAndEntry.getKey();
				newValue.apply(part);
				return sf.store(part,orig_l,index);
			} else {
				return orig;
			}
		}
		else return newValue.apply(orig);
	}
}
