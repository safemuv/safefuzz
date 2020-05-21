package exptrunner;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Scanner;
import java.util.stream.Collectors;

import atlasdsl.Mission;
import atlasdsl.faults.Fault;
import atlasdsl.loader.DSLLoader;
import atlasdsl.loader.GeneratedDSLLoader;
import atlassharedclasses.FaultInstance;
import faultgen.FaultFileCreator;
import faultgen.FaultFileIO;

public class RandomFaultConfigs extends ExptParams {

	private FileWriter combinedResults;
	
	// The time range for the fixed fault
	private int repeats;
	private int repeatsCount;
	private Mission mission;
	private FaultFileIO fio;
	private String filename;
	
	private String path = "/home/jharbin/academic/atlas/atlas-middleware/expt-working/";
	
	public RandomFaultConfigs(String filename, int repeatsCount, Mission mission) throws IOException {
		this.combinedResults = new FileWriter(filename);
		this.repeats = 0;
		this.repeatsCount = repeatsCount;
		this.mission = mission;
		this.filename = filename;
		fio = new FaultFileIO(mission);
		genNewFaults();
	}
	
	private void genNewFaults() {
		FaultFileCreator ffc = new FaultFileCreator(mission, path);
		ffc.generateFaultListFromScratch(filename);
	}
	
	public void advance() {
		repeats++;
		genNewFaults();
	}

	public List<FaultInstance> specificFaults() {
		try {
			List<FaultInstance> fs = fio.loadFaultsFromFile(path + "testfaults.fif");
			return fs;
		} catch (FileNotFoundException e) {
			e.printStackTrace();
			return new ArrayList<FaultInstance>();
		}
	}
	
	private String specificFaultsAsString() {
		List<FaultInstance> fis = specificFaults();
		String str = fis.stream().map(f -> f.toString()).collect(Collectors.joining());
		return str;
	}

	public boolean completed() {
		System.out.println("repeats = " + repeats + ",repeatsCount = " + repeatsCount);
		return (repeats >= repeatsCount); 
	}

	public void logResults(String logFileDir) {
		// Read the goal result file here - process the given goals
		// Write it out to a common result file - with the fault info
		File f = new File(logFileDir + "/goalLog.log");
		Scanner reader;
		try {
			reader = new Scanner(f);
			while (reader.hasNextLine()) {
				String line = reader.nextLine();
				String[] fields = line.split(",");
				String goalClass = fields[0];
				String time = fields[1];
				String robot = fields[2];
				String num = fields[3];
				if (goalClass.equals("atlasdsl.DiscoverObjects")) {
					combinedResults.write(repeats + "," + specificFaultsAsString() + "," + time + "," + robot + "," + num + "\n");
					combinedResults.flush();
				}
			}
			reader.close();
		} catch (FileNotFoundException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	
	}

	public void printState() {
		System.out.println("repeats = " + repeats);
		
	}
}
