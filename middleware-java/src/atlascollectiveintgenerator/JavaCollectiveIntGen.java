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
			return mission.messagesToAnyComponent(targets);
		} else {
			// TODO: handle computers in findRelevantMessages
			//targets = mission.getAllComputers();
			return new ArrayList<Message>();
		}
	}

	private void generateMessageReceptionHooks(TypeSpec.Builder ciClass, CollectiveIntGenTypes cgt) {
		List<Message> msgs = findRelevantMessages(cgt);
		for (Message m : msgs) {
			String msgMethodName = m.getName() + "Hook";
			MethodSpec hook = MethodSpec.methodBuilder(msgMethodName)
						.addModifiers(Modifier.PRIVATE)
						.returns(void.class)
						.addParameter(Message.class, m.getName())
						.build();
			ciClass.addMethod(hook);
		}
	}
	
	public void generateCollectiveIntStub(String fileName, CollectiveIntGenTypes cgt) {
		try {
			FileWriter fw = new FileWriter(fileName);
			TypeSpec.Builder ciClass = TypeSpec.classBuilder("robotCollectiveIntelligence");
			// Generate a method for the reception of each message
			generateMessageReceptionHooks(ciClass, cgt);
			// TODO: step 3. generate a method from each sensor notification
			JavaFile javaFile = JavaFile.builder("collectiveint", ciClass.build()).build();
			javaFile.writeTo(fw);
			// TODO: step 4. generate topics with ActiveMQ for every needed reception - build up a list of all 
			// topics needed and subscription code to connect to them over ActiveMQ
			fw.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}
