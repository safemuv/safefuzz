package atlasdsl;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;


public class Robot extends Component {
	private String robotname;
	private VehicleType vtype;
	private List<Subcomponent> contains = new ArrayList<Subcomponent>();
	
	public Robot(String robotname) {
		this.robotname = robotname;
		this.vtype = VehicleType.KAYAK;
	}
	
	public String getName() {
		return robotname;
	}
	
	public void addSubcomponent(Subcomponent s) {
		contains.add(s);
	}
	
	public String vehicleTypeAsString() {
		if (vtype == VehicleType.LIGHT_DRONE) return "LIGHT_DRONE";
		if (vtype == VehicleType.HEAVY_DRONE) return "HEAVY_DRONE";
		return "KAYAK";
	}

	public Sensor getSensor(SensorType st) {
		List<Subcomponent> sensors = contains.stream()
				.filter(sc -> sc instanceof Sensor)
				.collect(Collectors.toList());
		if (sensors.size() > 0) {
			return (Sensor) sensors.get(0);
		} else return null;
	}
}
