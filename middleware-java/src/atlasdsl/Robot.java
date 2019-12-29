package atlasdsl;

import java.util.List;

public class Robot extends Component {
	private String robotname;
	private List<Subcomponent> contains;
	
	public Robot(String robotname) {
		this.robotname = robotname;
	}
	
	public String getName() {
		return robotname;
	}
	
	public void addSubcomponent(Subcomponent s) {
		contains.add(s);
	}
}
