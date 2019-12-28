package atlasdsl;

import java.util.List;

public class Robot extends Component {
	private String robotname;
	private List<Subcomponent> contains;
	
	public String getName() {
		return robotname;
	}
}
