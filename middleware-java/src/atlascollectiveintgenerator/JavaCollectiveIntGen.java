package atlascollectiveintgenerator;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import javax.lang.model.element.Modifier;

import com.squareup.javapoet.*;

import atlasdsl.*;

public class JavaCollectiveIntGen extends CollectiveIntGen {
	
	public JavaCollectiveIntGen(Mission m) {
		super(m);
	}
	
	// TEST CODE for javapoet - remove when finished
	public void testJavaPoet(FileWriter fw) throws IOException {
	MethodSpec main = MethodSpec.methodBuilder("main")
		    .addModifiers(Modifier.PUBLIC, Modifier.STATIC)
		    .returns(void.class)
		    .addParameter(String[].class, "args")
		    .addStatement("$T.out.println($S)", System.class, "Hello, JavaPoet!")
		    .build();

		TypeSpec helloWorld = TypeSpec.classBuilder("HelloWorld")
		    .addModifiers(Modifier.PUBLIC, Modifier.FINAL)
		    .addMethod(main)
		    .build();

		JavaFile javaFile = JavaFile.builder("com.example.helloworld", helloWorld)
		    .build();

		javaFile.writeTo(fw);
		System.out.println("testJavaPoet...");
		javaFile.writeTo(System.out);
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
	public void generateRobotCI(CIFiles cif, Robot robot) {
		String className = "robotCollectiveIntelligence";
		if (robot != null) {
			className = className + robot.getName();
		}
			
		TypeSpec.Builder ciClass = TypeSpec.classBuilder(className);
		if (robot != null) {
			// TODO: store the parent class generated
			ciClass.superclass(cif.getClass("robotCollectiveIntelligence").class);
		}
		
		// Need to find all the potential hooks for ANY robot
		// and need to know which robot the hooks came from?
		
		List<Message> msgs = findRelevantMessages(CollectiveIntGenTypes.ALL_ROBOTS);
		for (Message m : msgs) {
			String hookName = m.getName() + "Hook";
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
	
	// Generates a general file for all computers
	public void generateGeneralComputerCI(CIFiles cif) {
		TypeSpec.Builder ciClass = TypeSpec.classBuilder("computerCollectiveIntelligence");
	}
	
	public void generateCollectiveIntFiles(String baseDir, CollectiveIntGenTypes cgt) {
			CIFiles cif = new CIFiles(baseDir);
			// First generate the general robot CI
			generateGeneralRobotCI(cif, null);
						
			for (Robot r : mission.getAllRobots()) {
			}
			
			//FileWriter fw = new FileWriter(fileName);
			//TypeSpec.Builder ciClass = TypeSpec.classBuilder("robotCollectiveIntelligence");
			// Generate a method for the reception of each message
			//generateStandardHooks(ciClass);
			//generateMessageReceptionHooks(ciClass, cgt);
			// TODO: step 3. generate a method from each sensor notification
			//JavaFile javaFile = JavaFile.builder("collectiveint", ciClass.build()).build();
			//javaFile.writeTo(fw);
			//javaFile.writeTo(System.out);
			// TODO: step 4. generate topics with ActiveMQ for every needed reception - build up a list of all 
			// topics needed and subscription code to connect to them over ActiveMQ
			//fw.close();
		//} catch (IOException e) {
			// TODO Auto-generated catch block
			//e.printStackTrace();
		//}
	}
}
