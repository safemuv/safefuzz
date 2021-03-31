package atlasdsl;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
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
		s.setParent(this);
	}
	
	public String vehicleTypeAsString() {
		if (vtype == VehicleType.LIGHT_DRONE) return "LIGHT_DRONE";
		if (vtype == VehicleType.HEAVY_DRONE) return "HEAVY_DRONE";
		return "KAYAK";
	}

	// TODO: should use Optional for these. And the same for MotionSource etc
	public Sensor getSensor(SensorType st) {
		List<Subcomponent> sensors = contains.stream()
				.filter(sc -> sc instanceof Sensor)
				.filter(sc -> ((Sensor)sc).getType() == st)
				.collect(Collectors.toList());
		
		if (sensors.size() > 0) {
			return (Sensor) sensors.get(0);
		} else return null;
	}
	
	public List<Sensor> getAllSensors() {
		return (List<Sensor>)(contains.stream()
				.filter(sc -> sc instanceof Sensor)
				.map(sc -> (Sensor)sc)
				.collect(Collectors.toList()));
	}

	public Optional<MotionSource> getMotionSource() {
		List<Subcomponent> mss = contains.stream()
				.filter(sc -> sc instanceof MotionSource)
				.collect(Collectors.toList());
		if (mss.size() > 0) {
			return Optional.of((MotionSource)mss.get(0));
		} else return Optional.empty();
	}

	public Optional<Sensor> getFirstSensor() {
		List<Sensor> sensors = getAllSensors();
		if (sensors.size() > 0) {
			return Optional.of(sensors.get(0));
		} else {
			return Optional.empty();
		}
	}
}
