package atlasdsl;

import java.util.ArrayList;
import java.util.List;

public class Computer extends Component {
	private String computername;
	private List<Subcomponent> contains = new ArrayList<Subcomponent>();
	
	public Computer(String computername) {
		this.computername = computername;
	}
	
	public String getName() {
		return computername;
	}
}
