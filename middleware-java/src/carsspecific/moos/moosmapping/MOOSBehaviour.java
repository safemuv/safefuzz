package carsspecific.moos.moosmapping;

import java.io.FileWriter;
import java.io.IOException;

public class MOOSBehaviour extends MOOSElement {
	private String name;
	private MOOSProcess parentProcess;
	protected int priority = 100;
	
	public void generateCode(FileWriter bhvFile) throws IOException {
		bhvFile.write("Behavior = " + name + " \n{\n");
		writePropertiesDefault(bhvFile);
		bhvFile.write("}\n\n");
	}
	
	public MOOSBehaviour(String name, MOOSProcess parentProcess) {
		this.name = name;
		this.setParentProcess(parentProcess);
		setProperty("pwt", priority);
	}

	public MOOSProcess getParentProcess() {
		return parentProcess;
	}

	public void setParentProcess(MOOSProcess parentProcess) {
		this.parentProcess = parentProcess;
	}
	
	protected void setName(String name) {
		this.name = name;
	}
}