package atlasdsl;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

import atlassharedclasses.Point;

public class Robot extends Component {
	private static final boolean ROBOT_ENERGY_DEBUGGING = false;
	private String robotname;
	private VehicleType vtype;
	private List<Subcomponent> contains = new ArrayList<Subcomponent>();
	
	private double currentEnergy = 0.0;
	private double startingEnergy = 0.0;
	
	private void setInitialLocation() throws MissingProperty {
		// Need to set the initial location property for the robot from startLocation
		// Otherwise the energy updates will fail
		Point loc = getPointComponentProperty("startLocation");
		setPointComponentProperty("location", new Point(loc.getX(), loc.getY()));
	}
	
	public Robot(String robotname) {
		this.robotname = robotname;
		this.vtype = VehicleType.KAYAK;
	}
	
	public void checkPropertiesAndSetupState() {
		try {
			setInitialLocation();
		} catch (MissingProperty e) {
			e.printStackTrace();
		}
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
	
	public List<Battery> getBatteries() {
		List<Battery> mss = contains.stream()
				.filter(sc -> sc instanceof Battery)
				.map(sc -> (Battery)sc)
				.collect(Collectors.toList());
		return mss;
	}

	public Optional<Sensor> getFirstSensor() {
		List<Sensor> sensors = getAllSensors();
		if (sensors.size() > 0) {
			return Optional.of(sensors.get(0));
		} else {
			return Optional.empty();
		}
	}

	public void setupRobotEnergy() {
		// Enumerate all the Battery devices for the robot
		for (Battery b : getBatteries()) {
			currentEnergy += b.getMaxEnergy();
			startingEnergy += b.getMaxEnergy();
			System.out.println("Robot " + getName() + " found battery of capacity " + b.getMaxEnergy());
		}
	}
	
	public void registerEnergyUsage(Point newLocation) {
		// Get the current location and the MotionSource
		Optional<MotionSource> ms_o = getMotionSource();
		if (ms_o.isPresent()) {
			try {
				Point currentLocation = getPointComponentProperty("location");
				double distanceTravelled = currentLocation.distanceTo(newLocation);
				MotionSource ms = ms_o.get();
				double energyConsumed = ms.getEnergyPerDistance() * distanceTravelled;
				currentEnergy -= energyConsumed;
				currentEnergy = Math.max(currentEnergy, 0.0);
				if (ROBOT_ENERGY_DEBUGGING) {
					System.out.println("Robot " + getName() + " distanceTravelled = " + distanceTravelled +",energyConsumed = " + energyConsumed + ",currentEnergy = " + currentEnergy);
				}
				
			} catch (MissingProperty e) {
				System.out.println("Missing properties in registerEnergyUsage - location for robot " + getName());
				e.printStackTrace();
			}
		
		} else {
			if (ROBOT_ENERGY_DEBUGGING) {
				System.out.println("No Motion source - energy update ignored");
			}
		}
	}
	
	public double getEnergyProportionRemaining() {
		return ((double)currentEnergy) / ((double)startingEnergy);
	}
	
	public double getEnergyRemaining() {
		return currentEnergy;
	}

	public void depleteEnergy(double fixedEnergyLoss) {
		currentEnergy -= fixedEnergyLoss;
		System.out.println("depleteEnergy: robot " + getName() + " experienced energy loss of " + fixedEnergyLoss + ",currentEnergy = " + currentEnergy);
	}
	
	public boolean noEnergyRemaining() {
		return (currentEnergy <= 0.0);
	}
}
