package utils;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;

import org.buildobjects.process.ProcBuilder;
import org.buildobjects.process.ProcResult;

public class ExptHelper {
	public static Process startNewJavaProcess(final String optionsAsString, final String mainClass,
			final String[] arguments, String path) throws IOException {

		ProcessBuilder processBuilder = ExptHelper.createJVMProcess(optionsAsString, mainClass, arguments);
		processBuilder.directory(new File(path));
		
		Process process = processBuilder.start();
		InputStream stdout = process.getInputStream();
		InputStream stderr = process.getErrorStream();
		Thread threadOut = new Thread( new MyInputStreamSink( stdout, "out" ));
		Thread threadErr = new Thread( new MyInputStreamSink( stderr, "err" ));

		threadOut.setDaemon(true);
		threadErr.setDaemon(true);
		threadOut.setName( String.format("stdout reader" ));
		threadErr.setName( String.format("stderr reader" ));
		threadOut.start();
		threadErr.start();
		return process;
	}
	
	public static ProcessBuilder createJVMProcess(final String optionsAsString, final String mainClass,
			final String[] arguments) {
		String jvm = System.getProperty("java.home") + File.separator + "bin" + File.separator + "java";
		String classpath = System.getProperty("java.class.path");

		String[] options = optionsAsString.split(" ");
		List<String> command = new ArrayList<String>();
		command.add(jvm);
		command.addAll(Arrays.asList(options));
		command.add(mainClass);
		command.addAll(Arrays.asList(arguments));

		System.out.println("command: " + String.join(" ", command));
		ProcessBuilder processBuilder = new ProcessBuilder(command);
		Map<String, String> environment = processBuilder.environment();
		environment.put("CLASSPATH", classpath);
		return processBuilder;
	}

	public static Process startScript(String path, String scriptFile) throws IOException {
		System.out.println("Starting script " + scriptFile);
		ProcessBuilder pb = new ProcessBuilder("/bin/bash", "-c", path + "/" + scriptFile);
		pb.directory(new File(path));
		Process proc = pb.start();
		return proc;
	}

	public static Process startCmd(String path, String cmd) throws IOException {
		System.out.println("Starting command " + cmd);
		ProcessBuilder pb = new ProcessBuilder("/bin/bash", "-c", path + "/" + cmd);
		pb.directory(new File(path));
		Process proc = pb.start();
		return proc;
	}
	
	public static void runScriptNew(String dir, String command, String arg) {
		ByteArrayOutputStream output = new ByteArrayOutputStream();
		File dirFile = new File(dir);
		ProcBuilder pb = new ProcBuilder(command).withArg(arg).withOutputStream(output).withWorkingDirectory(dirFile);
		try {
		ProcResult res = pb.run();
		} catch (org.buildobjects.process.TimeoutException e) {
			System.out.println("Ignoring timeout exception when launching");
		}
	}
}