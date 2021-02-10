package carsspecific.moos.codegen;

import carsspecific.moos.moosmapping.*;
import fuzzingengine.FuzzingEngine;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.FuzzingSimMapping;
import fuzzingengine.MissingComponentPath;
import middleware.atlascarsgenerator.*;
import middleware.atlascarsgenerator.carsmapping.*;
import utils.binarymodify.BinaryModify;
import utils.binarymodify.IncorrectStringLength;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

import atlassharedclasses.Point;
import atlassharedclasses.Region;
import activemq.portmapping.PortMappings;
import atlasdsl.*;

// FIX: maybe move to MOOSSimulation?

public class MOOSCodeGen extends CARSCodeGen {
	// TODO: how to specify the sensor behaviour
	public MOOSCodeGen(Mission m, Optional<FuzzingEngine> fe_o) {
		super(m, fe_o);
	}

	// This performs the necessary processing to add sensors to the robots
	// If the robot has a Sensor (sonar) then include the appropriate
	// uFldHazardSensor
	// The shoreside also has to include the uFldHazardMgr component
	private void setupSensors(Mission mission, MOOSSimulation moossim, List<String> moosSharedVars) {
		// The robots which have sensors have to have an appropriate uFldHazardMgr?
		// process for them
		// Shoreside or other monitoring computers have to have uFldHazardSensor
		// variables have to be added
		// if an unexpected sensor is present, raise an exception?

	}

	// This performs the necessary processing to set up actuators
	private void setupActuators(Mission mission, MOOSSimulation moossim, List<String> moosSharedVars) {

	}

	private List<String> varsForMissionGoals(Mission mission) {
		// TODO: process the mission, translate the mission goals to variables
		List<String> vs = new ArrayList<String>();
		// Assume all missions monitor the X,Y coordinates of robots
		// TODO: check and add other necessary variables here
		vs.add("VAR_X");
		vs.add("VAR_Y");
		vs.add("DB_UPTIME");
		return vs;
	}

	private List<String> varsForCI(Mission mission) {
		return new ArrayList<>();
	}

	// Creates an ATLASDBInterface component to watch the given variables
	private void createATLASLink(MOOSCommunity c, List<String> middleWareVars, int port) {
		ATLASInterfaceProcess dbwatch = new ATLASInterfaceProcess(c, c.getCommunityName());
		for (String v : middleWareVars) {
			dbwatch.addWatchVariable(v);
		}
		c.addProcess(dbwatch);
	}

	public CARSSimulation convertDSL() throws ConversionFailed {
		// Names of variables to be shared via pShare
		List<String> moosSharedVars = new ArrayList<String>();
		List<String> middlewareVars = varsForMissionGoals(mission);
		List<String> collectiveIntelVars = varsForCI(mission);

		boolean needShoresideObstacles = false;

		int atlasPort = 61613;
		int pSharePortToMiddleware = PortMappings.portNumberForPShareReception();
		int pShareListenPort = PortMappings.portBaseForPShareListen();
		try {

			MOOSSimulation moossim = new MOOSSimulation();

			// This currently encodes the assumption of one shoreside computer
			// performing system monitoring/management of CI
			// TODO: Should be made more flexible - we could have multiple
			// computers involved
			MOOSCommunity shoreside = null;
			if (mission.includesComputer()) {
				shoreside = new ComputerCommunity(moossim, "shoreside", PortMappings.portNumberForPShareReception(),
						PortMappings.portBaseForPShareListen());
				moossim.addCommunity(shoreside);
				System.out.println("Adding community for fixed computer");
			}

			// This performs the translation from DSL objects to a MOOS mission definition
			// Firstly: for each Robot, generate a MOOSCommunity

			boolean shoresideSonar = false;
			for (Robot r : mission.getAllRobots()) {
				middlewareVars.add("NODE_REPORT_" + r.getName().toUpperCase());
				Point startPos = r.getPointComponentProperty("startLocation");
				double startSpeed = r.getDoubleComponentProperty("startSpeed");
				double maxSpeed = r.getDoubleComponentProperty("maxSpeed");
				double maxDepth = r.getDoubleComponentProperty("maxDepth");
				String pShareHostname = PortMappings.addressForPShare();

				System.out.println("pSharePortBase = " + pSharePortToMiddleware);

				MOOSCommunity rprocess = new RobotCommunity(moossim, r, startPos, startSpeed, maxSpeed, maxDepth,
						pShareHostname, pSharePortToMiddleware, pShareListenPort);
				moossim.addCommunity(rprocess);
				System.out.println("Adding community for robot: " + r.getName());

				// TODO: robot properties, such as maximum speed, should be propagated to the
				// robot community here
				// TODO: AvoidCollision should be added to the new Robot's Helm behaviours when
				// an an avoidance goal exists

				Sensor s;
				// Check for particular sensors and add the necessary processes
				if ((s = (r.getSensor(SensorType.SONAR))) != null) {
					double sensorWidth = s.getDoubleComponentProperty("swathWidth");
					double detectionProb = s.getDoubleComponentProperty("detectionProb");
					MOOSProcess sonar_proc = new UFldHazardMgrProcess(rprocess, r.getName(), sensorWidth,
							detectionProb);
					rprocess.addProcess(sonar_proc);
					shoresideSonar = true;
				}

				// TODO: Can we support sonar and camera sensors correctly on a single vehicle?
				// For the heterogeneous case study it doesn't matter for now
				if ((s = (r.getSensor(SensorType.CAMERA))) != null) {
					// System.out.println("sensor camera, parent = " +
					// ((Robot)s.getParent()).getName());
					double sensorWidth = s.getDoubleComponentProperty("imagingRange");
					double detectionProb = 1.0;
					// The camera is implemented as a hazard process
					MOOSProcess sonar_proc = new UFldHazardMgrProcess(rprocess, r.getName(), sensorWidth,
							detectionProb);
					rprocess.addProcess(sonar_proc);
					shoresideSonar = true;
				}

				// If there are any environmental obstacles, then the shoreside needs to include
				// uFldObstacleSim
				// and the vehicles need to include PObstacleMgr, so set a flag
				if (mission.hasObstacles()) {
					needShoresideObstacles = true;
					MOOSProcess pObstacleMgr = new PObstacleMgrProcess(rprocess);
					MOOSProcess helmProcess = rprocess.getProcess("pHelmIvP");
					String rname = r.getName();
					MOOSBehaviour avoidBHV = new HelmBehaviourAvoidObstacles(helmProcess, rname);
					helmProcess.addBehaviour(avoidBHV);
					rprocess.addProcess(pObstacleMgr);
				}
			}

			if (needShoresideObstacles) {
				// TODO: specify this region in the model
				Region r = new Region(new Point(-150.0, -260.0), new Point(245.0, 20.0));
				System.out.println("TEST: using a fixed region in obstacle code generation");
				UFldObstacleSim uFldObstacleSim = new UFldObstacleSim(shoreside, r);
				for (EnvironmentalObstacle eo : mission.getObstacles()) {
					uFldObstacleSim.addDetectionObject(eo);
				}
				shoreside.addProcess(uFldObstacleSim);
			}

			// If there is a sonar sensor anywhere, then the shoreside needs to include a
			// HazardSensor process
			if (shoresideSonar) {
				moosSharedVars.add("UHZ_SENSOR_CONFIG");
				moosSharedVars.add("UHZ_HAZARD_REPORT");
				moosSharedVars.add("UHZ_CONFIG_REQUEST");
				moosSharedVars.add("UHZ_SENSOR_REQUEST");
				moosSharedVars.add("HAZARDSET_REPORT");

				middlewareVars.add("UHZ_DETECTION_REPORT");
				middlewareVars.add("UHZ_HAZARD_REPORT");

				if (shoreside == null) {
					throw new ConversionFailed(ConversionFailedReason.NO_SHORESIDE);
				} else {
					UFldHazardSensorProcess shoreside_sonar_proc = new UFldHazardSensorProcess(shoreside);

					// The shoreside_sonar_proc must be told of all the objects to find
					for (EnvironmentalObject eo : mission.getEnvironmentalObjects()) {
						shoreside_sonar_proc.addObject(eo);
					}
					shoreside.addProcess(shoreside_sonar_proc);

					// For sensor communications to work, we must add these lines to the shoreside's
					// uFldShoreBroker
					MOOSProcess shoreBroker = shoreside.getProcess("uFldShoreBroker");
					shoreBroker.setProperty("bridge", "src=UHZ_CONFIG_ACK_$V,       alias=UHZ_CONFIG_ACK");
					shoreBroker.setProperty("bridge", "src=UHZ_DETECTION_REPORT_$V, alias=UHZ_DETECTION_REPORT");
					shoreBroker.setProperty("bridge", "src=HAZARDSET_REQUEST_$V,    alias=HAZARDSET_REQUEST");

					if (needShoresideObstacles) {
						shoreBroker.setProperty("bridge", "src=GIVEN_OBSTACLE");
						shoreBroker.setProperty("qbridge", "TRACKED_FEATURE");
					}
				}
			}

			setupActuators(mission, moossim, moosSharedVars);

			for (MOOSCommunity c : moossim.getAllCommunities()) {
				c.registerSharedVars(moosSharedVars);

				// ATLASDBWatch process must be created in the each community
				// to watch the given variables for the middleware
				createATLASLink(c, middlewareVars, atlasPort);
			}

			// If there is a fuzzing engine, perform the fuzzing conversion
			if (fe_o.isPresent()) {
				FuzzingEngine fe = fe_o.get();
				fuzzingEngineAllChanges(fe, moossim);
			}
			
			// The catch-all output element in pShare has to be inserted AFTER the
			// fuzzing engine modifications (since messaage keys modify pShare).
			// Otherwise, it is ignored
			for (MOOSCommunity c : moossim.getAllCommunities()) {
				c.registerSharedVars(moosSharedVars);
				PShareProcess ps = (PShareProcess)c.getProcess("pShare");
				ps.setCatchAllOutputRecord();
			}

			// This returns a MOOS community without any faults
			// There will be a later stage to modify the community to inject faults.
			return moossim;
		} catch (MissingProperty mp) {
			System.out.println("Conversion failed: component " + mp.getComponent() + " is missing property "
					+ mp.getPropertyName() + "...");
			mp.printStackTrace();
			throw new ConversionFailed(ConversionFailedReason.MISSING_PROPERTY);
		}
	}

	private void fuzzingEngineAllChanges(FuzzingEngine fe, MOOSSimulation moossim) {
		fuzzingEngineKeyChanges(fe, moossim);
		fuzzingEngineComponentChanges(fe, moossim);
		fuzzingEngineMessageChange(fe, moossim);
	}

	private void fuzzingEngineMessageChange(FuzzingEngine fe, MOOSSimulation moossim) {
		// On the output message side, need to setup explicit pShare message settings
		// for output keys
		for (MOOSCommunity community : moossim.getAllCommunities()) {
			String robotName = community.getCommunityName();
			List<String> outboundKeys = fe.getMessageKeys(robotName, FuzzingSimMapping.VariableDirection.OUTBOUND);
			PShareProcess pShare = (PShareProcess) community.getProcess("pShare");
			ATLASInterfaceProcess dbInt = (ATLASInterfaceProcess) community.getProcess("ATLASDBInterface");

			for (String k : outboundKeys) {
				String kprimed = k + "'";
				System.out.println(
						"For robot " + robotName + " adding outbound key to pShare reflection " + k + " -> " + kprimed);
				pShare.addOutputRecord(k, kprimed);
			}

			List<String> inBoundKeys = fe.getMessageKeys(robotName, FuzzingSimMapping.VariableDirection.INBOUND);
			for (String k : inBoundKeys) {
				String kprimed = k + "'";
				System.out.println("For robot " + robotName + " adding inbound fuzzing key to ATLASDBWatch " + k
						+ " -> " + kprimed);
				dbInt.addWatchVariable(kprimed);
			}
		}
	}
	
	private void fuzzingEngineKeyChanges(FuzzingEngine fe, MOOSSimulation moossim) {
		// Specifies what is added to fuzzing process name
		final String PROCESSNAME_FUZZED_APPEND = "_f";

		// Get all the selected components
		Set<String> componentNames = fe.getConfig().getComponentsByEither();
		// Get the binary for the component
		for (String component : componentNames) {
			System.out.println("component = " + component);
			if (fe.getSimMapping().isBinary(component)) {
				try {
					String componentFullPath = fe.getSimMapping().getFullPath(component);
					// Get the original/reflected variable name mappings
					Map<String, String> varChanges = fe.getAllChanges(component);
					System.out.println("varChanges = " + varChanges);
					String componentFullPath_modified = componentFullPath + PROCESSNAME_FUZZED_APPEND;

					BinaryModify.BBEModifyFile(componentFullPath, componentFullPath_modified, varChanges);
				} catch (IncorrectStringLength | IOException | InterruptedException e) {
					e.printStackTrace();
				} catch (MissingComponentPath e) {
					e.printStackTrace();
				}
			}
		}
			
		// Now do the fuzzing component variable name changes for the selected keys
		for (MOOSCommunity c : moossim.getAllCommunities()) {
				String cname = c.getCommunityName();
				Set<String> componentNamesForRobot = fe.getConfig().getComponentsByEitherForRobot(cname);
				for (String component : componentNamesForRobot) {
					Map<String, String> varChanges = fe.getAllChanges(component);
					System.out.println("varChanges = " + varChanges);
					for (Map.Entry<String, String> me : varChanges.entrySet()) {
						String varSource = me.getKey();			
						ATLASInterfaceProcess dbInt = (ATLASInterfaceProcess) c.getProcess("ATLASDBInterface");
						dbInt.addWatchVariable(varSource);
						MOOSProcess moosProcess = c.getProcess(component);
						if (moosProcess != null) {
							moosProcess.setProcessName(component + PROCESSNAME_FUZZED_APPEND);
						}
				}
			}
		}
	}
	
	// TODO: factor common things between fuzzingEngineKeyChanges and this
	private void fuzzingEngineComponentChanges(FuzzingEngine fe, MOOSSimulation moossim) {
		// Specifies what is added to fuzzing process name
		final String PROCESSNAME_FUZZED_APPEND = "_f";

		// Get all the selected components selected for component-based fuzzing
		Set<String> componentNames = fe.getConfig().getComponents();
		// Get the binary for the component
		for (String component : componentNames) {
			System.out.println("component = " + component);
			if (fe.getSimMapping().isBinary(component)) {
				try {
					String componentFullPath = fe.getSimMapping().getFullPath(component);
					// Get the original/reflected variable name mappings
					Map<String, String> varChanges = fe.getAllChangesDefinedOn(component);
					System.out.println("varChanges = " + varChanges);
					String componentFullPath_modified = componentFullPath + PROCESSNAME_FUZZED_APPEND;

					BinaryModify.BBEModifyFile(componentFullPath, componentFullPath_modified, varChanges);
				} catch (IncorrectStringLength | IOException | InterruptedException e) {
					e.printStackTrace();
				} catch (MissingComponentPath e) {
					e.printStackTrace();
				}
			}
		}
			
		// Now do the fuzzing component variable name changes for the selected keys
		for (MOOSCommunity c : moossim.getAllCommunities()) {
				String cname = c.getCommunityName();
				Set<String> componentNamesForRobot = fe.getConfig().getComponentsForRobot(cname);
				for (String component : componentNamesForRobot) {
					Map<String, String> varChanges = fe.getAllChangesDefinedOn(component);
					System.out.println("varChanges = " + varChanges);
					for (Map.Entry<String, String> me : varChanges.entrySet()) {
						String varSource = me.getKey();			
						ATLASInterfaceProcess dbInt = (ATLASInterfaceProcess) c.getProcess("ATLASDBInterface");
						dbInt.addWatchVariable(varSource);
						MOOSProcess moosProcess = c.getProcess(component);
						if (moosProcess != null) {
							moosProcess.setProcessName(component + PROCESSNAME_FUZZED_APPEND);
						}
				}
			}
		}
	}
}
