package moosmapping;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;

public class MOOSFiles {
	private String baseDirectory;
	
	public MOOSFiles(String baseDirectory) {
		this.baseDirectory = baseDirectory;
	}
	public FileOutputStream createOpenFile(String fileName) throws FileNotFoundException {
		String filePathName = baseDirectory + "/" + fileName;
		return new FileOutputStream(filePathName);
	}
}
