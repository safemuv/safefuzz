package moosmapping;

import java.io.FileWriter;
import java.io.IOException;

public class MOOSFiles {
	private String baseDirectory;
	
	public MOOSFiles(String baseDirectory) {
		this.baseDirectory = baseDirectory;
	}
	public FileWriter createOpenFile(String fileName) throws IOException {
		String filePathName = baseDirectory + "/" + fileName;
		return new FileWriter(filePathName);
	}
}
