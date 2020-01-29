package atlascollectiveintgenerator;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import javax.lang.model.element.Modifier;

import com.squareup.javapoet.*;

import atlasdsl.*;
import middleware.core.SensorDetection;

public class JavaCollectiveIntGen extends CollectiveIntGen {
	public JavaCollectiveIntGen(Mission m) {
		super(m);
	}
	
	//find all the messages relevant for this match this code generation type/component
	private List<Message> findRelevantMessages(CollectiveIntGenTypes cg) {
		if (cg == CollectiveIntGenTypes.ALL_ROBOTS) {
			List<Robot> rbTargets = mission.getAllRobots();
			// TODO: look at this and see if we can do something better
			List<Component> targets = rbTargets.stream().map(rb -> (Component)rb).collect(Collectors.toList());
			System.out.println(targets);
			return mission.messagesToAnyComponent(targets);
		} else {
			// TODO: handle computers in findRelevantMessages
			//targets = mission.getAllComputers();
			System.out.println("cg not handled");
			return new ArrayList<Message>();
		}
	}
	
	private void generateStandardHooks(TypeSpec.Builder ciClass) {
		MethodSpec hook = MethodSpec.methodBuilder("init")
				.addModifiers(Modifier.PRIVATE)
				.returns(void.class)
				.build();
		ciClass.addMethod(hook);
	}

	private void generalMessageReceptionHooks(TypeSpec.Builder ciClass, CollectiveIntGenTypes cgt) {
		List<Message> msgs = findRelevantMessages(cgt);
		System.out.println("generateMessageReceptionHooks - msg count = " + msgs.size());
		for (Message m : msgs) {
			String msgMethodName = m.getName() + "Hook";
			System.out.println("adding method - " + msgMethodName);
			MethodSpec hook = MethodSpec.methodBuilder(msgMethodName)
						.addModifiers(Modifier.PRIVATE)
						.returns(void.class)
						.addParameter(Message.class, m.getName())
						.build();
			ciClass.addMethod(hook);
		}
	}
	
	// Generates a general set of hooks for all robots
	// Message arrivals
	// Sensors, with a second parameter indicating the identity
	public void generateRobotCI(MethodSpec.Builder activeMQHooks, CIFiles cif, Robot robot) {
		String className = "RobotCI";
		if (robot != null) {
			className = className + robot.getName();
		}
			
		TypeSpec.Builder ciClass = TypeSpec.classBuilder(className);
		if (robot != null) {
			// TODO: store the parent class generated
			//ciClass.superclass(cif.getClass("robotCollectiveIntelligence").class);
		}
		
		// Need to find all the potential hooks for ANY robot
		// and need to know which robot the hooks came from?
		
		List<Message> msgs = findRelevantMessages(CollectiveIntGenTypes.ALL_ROBOTS);
		for (Message m : msgs) {
			String hookName = m.getName() + "MessageHook";
			System.out.println("adding method - " + hookName);
			MethodSpec hook = MethodSpec.methodBuilder(hookName)
						.addModifiers(Modifier.PRIVATE)
						.returns(void.class)
						.addParameter(Message.class, m.getName())
						.addParameter(Component.class, "from")
						.addParameter(Component.class, "to")
						.build();
			ciClass.addMethod(hook);
		}
		
		try {
			FileWriter robotOut = cif.getOpenFile("robotCI.java");
			JavaFile javaFile = JavaFile.builder("collectiveint", ciClass.build()).build();
			javaFile.writeTo(robotOut);
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	public void generateComputerCI(MethodSpec.Builder activeMQHooks, CIFiles cif, Computer c) {
		String className = "ComputerCI";
		if (c != null) {
			className = className + c.getName();
		}
		
		TypeSpec.Builder ciClass = TypeSpec.classBuilder(className);
		
		// Need to find all the potential hooks for ANY robot
		// and need to know which robot the hooks came from?
		List<Message> msgs = findRelevantMessages(CollectiveIntGenTypes.ALL_COMPUTERS);
		for (Message m : msgs) {
			String hookName = m.getName() + "MessageHook";
			System.out.println("DEBUG: adding method - " + hookName);
			MethodSpec hook = MethodSpec.methodBuilder(hookName)
						.addModifiers(Modifier.PRIVATE)
						.returns(void.class)
						.addParameter(Message.class, m.getName())
						.addParameter(Component.class, "from")
						.addParameter(Component.class, "to")
						.build();
			
			ciClass.addMethod(hook);

        };
	
		// Generate a hook for any possible sensor notification 
		// on any robot in the system - since the computers process
        // all the notifications via pShare
		for (Map.Entry<SensorType, Robot> sr : mission.getAllSensorTypesOnVehicles().entrySet()) {
			// should be sensor types?
			SensorType st = sr.getKey();
			Robot r = sr.getValue();
			String sensorTypeName = Sensor.sensorTypeToString(st).toUpperCase();
			
			System.out.println("DEBUG: adding method stub for sensor " + r.getClass());
			String hookName = st.toString() + "DetectionHook";
			MethodSpec hook = MethodSpec.methodBuilder(hookName)
							.addModifiers(Modifier.PRIVATE)
							.returns(void.class)
							.addParameter(SensorDetection.class, "detection")
							.addParameter(Robot.class, "robot")
							.build();
			ciClass.addMethod(hook);
			
			activeMQHooks.addCode("if (d.getSensorType() == SensorType." + sensorTypeName +") {\n"
					   + "ComputerCIShoreside." + sensorTypeName + "DetectionHook(d,d.getRobot());\n"
					   + "}\n"); 
		}
		
		try {
			FileWriter robotOut = cif.getOpenFile("computerCI.java");
			JavaFile javaFile = JavaFile.builder("collectiveint.user", ciClass.build()).build();
			javaFile.writeTo(robotOut);
			robotOut.flush();
			robotOut.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	// Generates a general file for all computers
	//public void generateGeneralComputerCI(CIFiles cif) {
	//	TypeSpec.Builder ciClass = TypeSpec.classBuilder("computerCollectiveIntelligence");
	//}
	
	public void generateCollectiveIntFiles(String baseDir, CollectiveIntGenTypes cgt) {
			CIFiles cif = new CIFiles(baseDir);
			
			
			TypeSpec.Builder activeMQLink = TypeSpec.classBuilder("CustomCollectiveInt");
			activeMQLink.superclass(CollectiveInt.class);
			
			MethodSpec.Builder addMQHooks = MethodSpec.methodBuilder("handleDetction");
			addMQHooks.addParameter(SensorDetection.class, "d");
						
			for (Robot r : mission.getAllRobots()) {
				generateRobotCI(addMQHooks, cif, r);
			}
			
			for (Computer c : mission.getAllComputers()) {
				generateComputerCI(addMQHooks, cif,c);
			}
			
			activeMQLink.addMethod(addMQHooks.build());
			JavaFile activeMQLinkFile = JavaFile.builder("collectiveint.user", activeMQLink.build()).build();
			try {
				FileWriter mqFileOut = cif.getOpenFile("CustomCollectiveInt.java");
				activeMQLinkFile.writeTo(mqFileOut);
				mqFileOut.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			
			// TODO: step 4. map subscriptions to topics with ActiveMQ for every needed reception - build up a list of all 
			// topics needed and subscription code to connect to them over ActiveMQ
	}
}
