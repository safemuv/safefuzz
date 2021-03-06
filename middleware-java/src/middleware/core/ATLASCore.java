package middleware.core;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

import javax.jms.JMSException;
import javax.json.JsonNumber;
import javax.json.JsonObject;

import com.fasterxml.jackson.core.JsonProcessingException;

import atlasdsl.*;
import atlasdsl.faults.*;
import atlassharedclasses.*;
import faultgen.FaultGenerator;
import fuzzexperiment.runner.jmetal.grammar.variables.VariableTemplate;
import middleware.carstranslations.CARSTranslations;
import middleware.gui.GUITest;
import middleware.logging.ATLASLog;
import middleware.missionmonitor.*;

import fuzzingengine.*;
import fuzzingengine.spec.GeneratedFuzzingSpec;

public abstract class ATLASCore {
	private static final boolean DEBUG_PRINT_DESERIALISED_MSGS_SENT_TO_CI = false;
	protected boolean stopOnNoEnergy = false;

	protected ATLASEventQueue<?> carsIncoming;
	protected ATLASEventQueue<?> fromCI;

	private double timeLimit;

	protected ActiveMQProducer outputToCI;
	private final int CI_QUEUE_CAPACITY = 100;
	protected Mission mission;
	protected MissionMonitor monitor;
	protected CARSTranslations carsOutput;

	protected FuzzingEngine fuzzEngine;

	protected Map<String, Boolean> vehicleBatteryFlat = new HashMap<String, Boolean>();

	protected Map<String, Object> simVariables = new HashMap<String, Object>();
	
	// Function variables are computed directly by the middleware
	protected Map<String, VariableTemplate> middlewareFunctionVariables = new HashMap<String, VariableTemplate>();

	protected GUITest gui;

	@SuppressWarnings("rawtypes")
	protected List<ATLASEventQueue> queues = new ArrayList<ATLASEventQueue>();
	protected HashMap<String, Object> goalVariables = new LinkedHashMap<String, Object>();
	protected List<FaultInstance> activeFaults = new ArrayList<FaultInstance>();
	protected List<SensorDetectionLambda> sensorWatchers = new ArrayList<SensorDetectionLambda>();
	protected List<PositionUpdateLambda> positionWatchers = new ArrayList<PositionUpdateLambda>();
	protected List<SpeedUpdateLambda> speedWatchers = new ArrayList<SpeedUpdateLambda>();
	
	// If the string matches the given topic, the lambda will be called when the
	// given simulator variable
	// is updated
	protected Map<String, List<SimVarUpdateLambda>> simVarWatchers = new HashMap<String, List<SimVarUpdateLambda>>();

	private FaultGenerator faultGen;
	private static ATLASCore coreRef;
	private double time = 0.0;

	private ATLASObjectMapper atlasOMapper = new ATLASObjectMapper();

	public void setupPositionPropertyUpdaters() {
		setupPositionWatcher((pos) -> {
			String rname = pos.getRobotName();
			Robot r = mission.getRobot(rname);
			r.setPointComponentProperty("location", new Point(pos.getX(), pos.getY(), pos.getZ()));
		});
	}

	public static ATLASCore getCore() {
		return coreRef;
	}

	public static void setCore(ATLASCore core) {
		if (coreRef == null) {
			coreRef = core;
		}
	}

	public ATLASCore(Mission mission) {
		this.mission = mission;
		stopOnNoEnergy = mission.stopOnNoEnergy();
		this.timeLimit = mission.getEndTime();
		this.monitor = new MissionMonitor(this, mission);
		fromCI = new CIEventQueue(this, mission, CI_QUEUE_CAPACITY);
		queues.add(fromCI);
		faultGen = new FaultGenerator(this, mission);
		fuzzEngine = GeneratedFuzzingSpec.createFuzzingEngine(mission, true);
		setCore(this);
		setupEnergyOnRobots();
		setupPositionPropertyUpdaters();
	}

	public ATLASCore(Mission mission, boolean includeCIQueue) {
		this.mission = mission;
		stopOnNoEnergy = mission.stopOnNoEnergy();
		this.timeLimit = mission.getEndTime();
		this.monitor = new MissionMonitor(this, mission);
		if (includeCIQueue) {
			fromCI = new CIEventQueue(this, mission, CI_QUEUE_CAPACITY);
			queues.add(fromCI);
		}
		faultGen = new FaultGenerator(this, mission);
		fuzzEngine = GeneratedFuzzingSpec.createFuzzingEngine(mission, true);
		setCore(this);
		setupEnergyOnRobots();
		setupPositionPropertyUpdaters();
	}

	public CARSTranslations getCARSTranslator() {
		return carsOutput;
	}

	public void createGUI() {
		gui = new GUITest(this, mission, faultGen);
	}

	public synchronized void registerFault(FaultInstance fi) {
		activeFaults.add(fi);
		System.out.println("Fault instance added");
		Fault f = fi.getFault();
		Optional<String> additionalData = fi.getExtraDataOpt();
		f.immediateEffects(this, additionalData);
	}

	public synchronized void completeFault(FaultInstance fi) {
		activeFaults.remove(fi);
		Fault f = fi.getFault();
		Optional<String> additionalData = fi.getExtraDataOpt();
		System.out.println("Calling completion effect");

		fi.getFault().completionEffects(this, additionalData);
		System.out.println("Fault instance " + fi + " completed: " + f.getClass());
	}

	public void clearFaults() {
		activeFaults.clear();
	}

	public ActiveMQProducer getCIProducer() {
		return outputToCI;
	}

	public void sendMessageToCI(Object o) {
		try {
			String msg = atlasOMapper.serialise(o);
			if (DEBUG_PRINT_DESERIALISED_MSGS_SENT_TO_CI) {
				System.out.println("sendMessageToCI: serialised message " + msg);
			}
			outputToCI.sendMessage(msg);
		} catch (JsonProcessingException e1) {
			e1.printStackTrace();
		} catch (JMSException e1) {
			e1.printStackTrace();
		}
		;
	}

//	public ObjectLambda setupLambdaFromFixedPoint(String varName, Point fixedPoint) {
//		ObjectLambda resLambda = (name) -> {
//			Robot r = mission.getRobot(name);
//			try {
//				Point p = r.getPointComponentProperty("location");
//				double distance = fixedPoint.distanceTo(p);
//				return distance;
//			} catch (MissingProperty p) {
//				return Double.MAX_VALUE;
//			}
//		};
//
//		middlewareFunctionVariables.put(varName, resLambda);
//		return resLambda;
//	}

//	public void setupMiddlewareFunctionVariables() {
//		ObjectLambda spdLambda = (name) -> {
//			Robot r = mission.getRobot(name);
//			try {
//				Point p = r.getPointComponentProperty("location");
//				Point orig = r.getPointComponentProperty("startLocation");
//				double distance = orig.distanceTo(p);
//				return distance;
//			} catch (MissingProperty p) {
//				return Double.MAX_VALUE;
//			}
//		};
//
//		ObjectLambda afdLambda = (name) -> {
//			Object gv = getGoalVariable(name, "/airframe_clearance");
//			if (gv != null) {
//				edu.wpi.rail.jrosbridge.messages.Message m = (edu.wpi.rail.jrosbridge.messages.Message)gv;
//				String typ = m.getMessageType();
//				JsonObject jobj = m.toJsonObject();
//				JsonNumber n = (JsonNumber)jobj.get("data");
//				double distVal = n.doubleValue();
//				return distVal;
//			} else {
//				return Double.MAX_VALUE;
//			}
//		};
//		
//		ObjectLambda irLambda = (name) -> {
//			try {
//				// ignore the name, just use the hardcoded robot distances
//				Point r1pos = mission.getRobot("uav_1").getPointComponentProperty("location");
//				Point r2pos = mission.getRobot("uav_2").getPointComponentProperty("location");
//				double irDist = r1pos.distanceTo(r2pos);
//				return irDist;
//			} catch (MissingProperty e) {
//				e.printStackTrace();
//				return Double.MAX_VALUE;
//			}
//		};
//		
//		ObjectLambda timeLambda = (name) -> {
//			double time = getTime();
//			return time;
//		};
//
//		middlewareFunctionVariables.put("starting_point_distance", spdLambda);
//		middlewareFunctionVariables.put("airframe_clearance", afdLambda);
//		middlewareFunctionVariables.put("time", timeLambda);
//		middlewareFunctionVariables.put("interrobot_distance", irLambda);
//		
//		setupLambdaFromFixedPoint("distance_to_left_wing_base", new Point(-29.14, -2.28, 5.2));
//		setupLambdaFromFixedPoint("distance_to_right_wing_base", new Point(-29.14, 6.26, 5.2));
//		setupLambdaFromFixedPoint("distance_to_nose", new Point(-6.79, 2.136, 5.8772));
//	}

	public void runMiddleware() {
		SetupMiddlewareFunctionVars.setup(this);

		for (ATLASEventQueue<?> q : queues) {
			// Since the GUI displays global status, it
			// needs to be updated following every event on any queue
			if (gui != null) {
				q.registerAfterHook(() -> gui.updateGUI());
			}
			// Also after events, need to check for faults
			q.registerAfterHook(() -> faultGen.pollFaultsNow());
			q.registerAfterHook(() -> monitor.runStep());
			q.setup();
		}

		for (ATLASEventQueue<?> q : queues) {
			new Thread(q).start();
		}
	}

	public double getTime() {
		return time;
	}

	public synchronized void updateTime(double time) throws CausalityException {
		// System.out.println("updateTime called with " + time);
		if ((time > this.time)) {
			this.time = time;
			if (this.time > timeLimit) {
				ATLASLog.logTime(time);
			}
		}
	}

	// This is used by active faults to inject their immediate effects
	// upon the low-level CARS simulation
	public void sendToCARS(Robot r, String key, String value) {
		CIEventQueue CIq = (CIEventQueue) fromCI;
		CIq.sendToCARS(r, key, value);
	}

	public List<FaultInstance> activeFaultsOfClass(Class<?> class1) {
		for (FaultInstance fi : activeFaults) {
			System.out.println("fault instance name " + fi.getFault().getImpact().getClass().getName());
		}

		System.out.println("class1 name: = " + class1.getName());
		System.out.println("activeFaults count: " + activeFaults.size());
		return activeFaults.stream().filter(fi -> fi.getFault().getImpact().getClass() == class1)
				.collect(Collectors.toList());
	}

	// This is called by the simulator-side event queues when a low
	// level sensor detection occurs
	public void notifySensorDetection(SensorDetection d) {
		for (SensorDetectionLambda watcher : sensorWatchers) {
			watcher.op(d);
		}
	}

	public void setupSensorWatcher(SensorDetectionLambda l) {
		sensorWatchers.add(l);
	}

	public void setFaultDefinitionFile(String filePath) {
		faultGen.setFaultDefinitionFile(filePath);

		if (gui != null) {
			gui.setFaultDefinitionFile(filePath);
		}
	}

	public FuzzingEngine getFuzzingEngine() {
		return fuzzEngine;
	}

	public void setupPositionWatcher(PositionUpdateLambda l) {
		positionWatchers.add(l);
	}

	public void setupSpeedWatcher(SpeedUpdateLambda l) {
		speedWatchers.add(l);
	}

	public void notifyPositionUpdate(GPSPositionReading gps) {
		for (PositionUpdateLambda watcher : positionWatchers) {
			watcher.op(gps);
		}
	}

	public void notifySpeedUpdate(SpeedReading s) {
		for (SpeedUpdateLambda watcher : speedWatchers) {
			watcher.op(s);
		}
	}

	public double getTimeLimit() {
		return timeLimit;
	}

	public void registerEnergyUsage(Robot r, Point newLocation) {
		r.registerEnergyUsage(newLocation);
		if (stopOnNoEnergy) {
			String robotName = r.getName();
			if (r.noEnergyRemaining() && !vehicleBatteryFlat.containsKey(robotName)) {
				vehicleBatteryFlat.put(robotName, true);
				getCARSTranslator().stopVehicle(r.getName());
				System.out.println("STOPPING VEHICLE " + r.getName() + " due to no energy remaining");
			}
		}
	}

	public double getRobotEnergyRemaining(Robot r) {
		return r.getEnergyRemaining();
	}

	public void setupEnergyOnRobots() {
		for (Robot r : mission.getAllRobots()) {
			r.setupRobotEnergy();
		}
	}

	public Object getVariable(String varName, String robotName) {
		// Use the middleware function variables first, then the sim variables
		String combinedName = varName;
		if (middlewareFunctionVariables.containsKey(combinedName)) {
			VariableTemplate l = middlewareFunctionVariables.get(combinedName);
			Object res = l.getValue(robotName, this);
			return res;
		} else {
			if (simVariables.containsKey(combinedName)) {
				Object res = simVariables.get(combinedName);
				return res;
			} else {
				return false;
			}
		}
	}

	public void setGoalVariable(String vehicleName, String topicName, Object val) {
		System.out.println("setGoalVariable: vehicleName=" + vehicleName + ",topicName=" + topicName + ",val=" + val);
		goalVariables.put(vehicleName + "-_-" + topicName, val);
	}

	public Object getGoalVariable(String vehicleName, String topicName) {
		return goalVariables.get(vehicleName + "-_-" + topicName);
	}
	
	public void setupSimVarWatcher(String topic, SimVarUpdateLambda lambda) {
		List<SimVarUpdateLambda> current = simVarWatchers.get(topic);
		if (current == null) {
			current = new ArrayList<SimVarUpdateLambda>();
		}
		current.add(lambda);
		simVarWatchers.put(topic, current);
	}

	// The boolean value indicates if this sim variable should be propagated to the CI.
	// This allows local events a chance to override this
	public boolean notifySimVarUpdate(SimulatorVariable sv, String robotName, Object value) {
		boolean shouldNotify = true;
		List<SimVarUpdateLambda> lambdas = simVarWatchers.get(sv.getVarName());
		if (lambdas != null) {
			for (SimVarUpdateLambda l : lambdas) {
				shouldNotify = shouldNotify && (l.op(sv, robotName, value));
			}
		}
		return shouldNotify;
	}
	
	public void addMiddlewareFunctionVariables(String varName, VariableTemplate vt) {
		middlewareFunctionVariables.put(varName, vt);
	}

	public Mission getMission() {
		return mission;
	}
}
