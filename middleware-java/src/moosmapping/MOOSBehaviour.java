package moosmapping;

import java.io.FileWriter;
import java.io.IOException;

public class MOOSBehaviour extends MOOSElement {
	private String name;
	private MOOSProcess parentProcess;
	
	public void generateCode(FileWriter bhv) throws IOException {
		bhv.write("Behaviour = " + name + " {\n");
		bhv.write("");
		bhv.write("}\n");
	}
}
