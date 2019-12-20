package moosmapping;

import java.io.FileWriter;
import java.io.IOException;

public class MOOSBehaviour extends MOOSElement {
	private String name;
	private MOOSProcess parentProcess;
	
	public void generateCode(FileWriter bhvFile) throws IOException {
		bhvFile.write("Behaviour = " + name + " \n{\n");
		writePropertiesDefault(bhvFile);
		bhvFile.write("}\n\n");
	}
	
	public MOOSBehaviour(String name, MOOSProcess parentProcess) {
		this.name = name;
		this.parentProcess = parentProcess;
	}
}