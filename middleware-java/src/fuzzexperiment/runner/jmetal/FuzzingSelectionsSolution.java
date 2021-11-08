package fuzzexperiment.runner.jmetal;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.uma.jmetal.solution.*;

import atlasdsl.*;
import fuzzexperiment.runner.metrics.Metric;
import fuzzingengine.FuzzingFixedTimeSpecification;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.FuzzingSelectionRecord;
import fuzzingengine.FuzzingTimeSpecification;

public class FuzzingSelectionsSolution implements Solution<FuzzingSelectionRecord> {
	private static final long serialVersionUID = 1L;

	private Mission mission;
	private boolean actuallyRun;
	private String exptTag;
	private double exptRunTime;
	String baseDir = "/tmp";
	
	private static int csvFileCount = 0;
	
	private int runNum;
	private int fuzzingTestNum;

	private Map<Object, Object> attributes = new HashMap<Object, Object>();
	private Map<Integer, Double> objectives = new HashMap<Integer, Double>();
	private Map<Integer, Metric> objectiveNames = new HashMap<Integer, Metric>();
	private Map<Integer, Double> constraints = new HashMap<Integer, Double>();
	private List<FuzzingSelectionRecord> contents = new ArrayList<FuzzingSelectionRecord>();

	private String csvFileName;

	public FuzzingSelectionsSolution(Mission mission, String exptTag, boolean actuallyRun, double exptRunTime) {
		this.mission = mission;
		this.exptTag = exptTag;
		this.actuallyRun = actuallyRun;
		this.exptRunTime = exptRunTime;
	}

	public FuzzingSelectionsSolution(Mission mission, String exptTag, boolean actuallyRun, double exptRunTime,
			List<FuzzingKeySelectionRecord> recs, int runNum) {
		this.mission = mission;
		this.exptTag = exptTag;
		this.actuallyRun = actuallyRun;
		this.exptRunTime = exptRunTime;
		this.contents = new ArrayList<FuzzingSelectionRecord>(recs.size());
		this.runNum = runNum;

		for (FuzzingSelectionRecord fi : recs) {
			this.contents.add(fi.dup());
		}
		System.out.println("contents = " + contents);
	}

	FuzzingSelectionsSolution(FuzzingSelectionsSolution other) {
		this.mission = other.mission;
		this.actuallyRun = other.actuallyRun;
		this.exptRunTime = other.exptRunTime;
		this.contents = new ArrayList<FuzzingSelectionRecord>(other.contents.size());

		for (FuzzingSelectionRecord fi : other.contents) {
			this.contents.add(fi.dup());
		}
	}

	public static FuzzingSelectionsSolution empty(FuzzingSelectionsSolution other) {
		FuzzingSelectionsSolution fi = new FuzzingSelectionsSolution(other.mission, other.exptTag, other.actuallyRun,
				other.exptRunTime);
		fi.contents = new ArrayList<FuzzingSelectionRecord>();
		return fi;
	}

	public void setObjective(int index, double value) {
		objectives.put(index, value);
	}

	public double getObjective(int index) {
		return objectives.get(index);
	}

	public double[] getObjectives() {
		int size = objectives.size();
		double[] res = new double[size];
		for (int i = 0; i < size; i++) {
			res[i] = objectives.get(i);
		}
		return res;
	}

	public FuzzingSelectionRecord getVariable(int index) {
		// System.out.println("index = " + index + ",contents.size()=" +
		// contents.size());
		if (index < contents.size() && index >= 0) {
			return contents.get(index);
		} else {
			return null;
		}
	}

	public List<FuzzingSelectionRecord> getVariables() {
		return contents;
	}

	public void setVariable(int index, FuzzingSelectionRecord variable) {
		contents.set(index, variable);
	}

	public double[] getConstraints() {
		int size = constraints.size();
		double[] res = new double[size];
		for (int i = 0; i < size; i++) {
			res[i] = getConstraint(i);
		}
		return res;
	}

	double FuzzingSelectionIntensity(FuzzingSelectionRecord fs) {
		// Assume the intensity is always 1 unless otherwise
		double intensity = 1.0;
		return intensity;
	}

	public double getConstraint(int index) {
		return 0.0;
	}

	public void setConstraint(int index, double value) {
		constraints.put(index, value);
	}

	public int getNumberOfVariables() {
		return contents.size();
	}

	public int getNumberOfObjectives() {
		return objectives.size();
	}

	public int getNumberOfConstraints() {
		return constraints.size();
	}

	public FuzzingSelectionsSolution copy() {
		FuzzingSelectionsSolution copy = new FuzzingSelectionsSolution(this);
		return copy;
	}

	public FuzzingSelectionsSolution dup() {
		return this.copy();
	}

	public List<FuzzingSelectionRecord> getFuzzingSelections() {
		return contents;
	}

	public void setContents(int index, FuzzingSelectionRecord fi) {
		contents.set(index, fi);
	}

	public void addContents(int index, FuzzingSelectionRecord fi) {
		contents.add(index, fi);
	}

	public int numberOfFuzzingSelections() {
		return contents.size();
	}

	public void setAttribute(Object id, Object value) {
		attributes.put(id, value);
	}

	public Object getAttribute(Object id) {
		return attributes.get(id);
	}

	public boolean hasAttribute(Object id) {
		return attributes.containsKey(id);
	}

	public Map<Object, Object> getAttributes() {
		return attributes;
	}

	public List<FuzzingSelectionRecord> testAllFuzzingSelections(FuzzingSelectionLambdaBoolean test) {
		List<FuzzingSelectionRecord> res = new ArrayList<FuzzingSelectionRecord>();
		for (FuzzingSelectionRecord fi : contents) {
			if (test.op(fi)) {
				res.add(fi);
			}
		}
		return res;
	}

	public void setAllContents(List<FuzzingSelectionRecord> fis) {
		int i = 0;
		for (FuzzingSelectionRecord fi : fis) {
			addContents(i, fi);
			i++;
		}
	}

	public double faultTimeTotal() {
		return 0.0;
	}

	public void generateCSVFile(String filePath) throws IOException {
		FileWriter fw = new FileWriter(filePath);
		for (FuzzingSelectionRecord fs : contents) {
			String line = fs.generateCSVLine();
			fw.write(line + "\n");
		}
		fw.close();
	}

	public void printCSVContentsToFile(FileWriter fw) throws IOException {
		for (FuzzingSelectionRecord fs : contents) {
			String line = fs.generateCSVLine();
			fw.write(line + "\n");
		}
	}

	public List<FuzzingSelectionRecord> variables() {
		return getVariables();
	}

	public double[] objectives() {
		return getObjectives();
	}

	public double[] constraints() {
		return getConstraints();
	}

	public Map<Object, Object> attributes() {
		return getAttributes();
	}

	public String createCSVFileName() throws IOException {
		csvFileCount++;
		fuzzingTestNum = csvFileCount;
		csvFileName = String.format("%s/jmetalfuzz-%03d.csv", baseDir, csvFileCount);
		generateCSVFile(csvFileName);
		return csvFileName;
	}
	
	public int getFuzzingTestNum() {
		return fuzzingTestNum;
	}

	public String getCSVFileName() throws IOException {
		if (csvFileName == null) {
			return createCSVFileName();
		} else {
			return csvFileName;
		}
	}

	public String toString() {
		try {
			String out = "CSV:" + getCSVFileName() + "\n";
			for (int i = 0; i < contents.size(); i++) {
				out += contents.get(i).generateCSVLine() + "\n";
			}
			return out;
		} catch (IOException e) {
			return "<IOException in getCSVFilename()>";
		}
	}

	public void regenerateCSVFile() throws IOException {
		generateCSVFile(csvFileName);
	}

	public boolean containsTopicKey(String key) {
		return contents.stream().anyMatch(r -> r.containsTopic(key));
	}

	public void deleteOverlapping() {
		List<FuzzingKeySelectionRecord> toRemove = new ArrayList<FuzzingKeySelectionRecord>();
		// Find all matching overlapping keys and delete them
		// Find any with two keys - and with timing overlapping
		for (FuzzingSelectionRecord r1 : contents) {
			for (FuzzingSelectionRecord r2 : contents) {
				FuzzingKeySelectionRecord rk1 = (FuzzingKeySelectionRecord)r1;
				FuzzingKeySelectionRecord rk2 = (FuzzingKeySelectionRecord)r2;
				if (rk1.getKey().equals(rk2.getKey())) {
					
					if (isInside(rk1.getTimeSpec(), rk2.getTimeSpec())) {
						// delete rk1
						toRemove.add(rk1);
					} else {					
							if (isInside(rk2.getTimeSpec(), rk1.getTimeSpec())) {
								// delete rk1
								toRemove.add(rk2);
							} else {
								// If there is a partial overlapping between the two of them? cut them to not overlap
								trimIfOverlap(rk1, rk2);
							}
					}
				}
			}
		}
		
		// Remove the selected records
		for (FuzzingKeySelectionRecord r : toRemove) {
			contents.remove(r);
		}
	}

	// If ts1 and ts2 overlap (ts1 before ts2), trim it so ts1 ends before ts2
	// starts
	private void trimIfOverlap(FuzzingKeySelectionRecord rk1, FuzzingKeySelectionRecord rk2) {
		FuzzingTimeSpecification timeSpec1 = rk1.getTimeSpec();
		FuzzingTimeSpecification timeSpec2 = rk2.getTimeSpec();
		if ((timeSpec1 instanceof FuzzingFixedTimeSpecification)
				&& (timeSpec2 instanceof FuzzingFixedTimeSpecification)) {
			FuzzingFixedTimeSpecification tsfixed1 = (FuzzingFixedTimeSpecification) timeSpec1;
			FuzzingFixedTimeSpecification tsfixed2 = (FuzzingFixedTimeSpecification) timeSpec2;
			if ((tsfixed1.getStartTime() < tsfixed2.getStartTime())
					&& (tsfixed1.getEndTime() > tsfixed2.getStartTime())) {
				tsfixed1.setEndTime(tsfixed2.getStartTime());
			}
		}
	}

	// Returns true if 1 is inside 2
	private boolean isInside(FuzzingTimeSpecification timeSpec1, FuzzingTimeSpecification timeSpec2) {
		if ((timeSpec1 instanceof FuzzingFixedTimeSpecification)
				&& (timeSpec2 instanceof FuzzingFixedTimeSpecification)) {
			FuzzingFixedTimeSpecification tsfixed1 = (FuzzingFixedTimeSpecification) timeSpec1;
			FuzzingFixedTimeSpecification tsfixed2 = (FuzzingFixedTimeSpecification) timeSpec2;
			return ((tsfixed1.getStartTime() > tsfixed2.getStartTime())
					&& (tsfixed1.getEndTime() < tsfixed2.getEndTime()));
		} else {
			return false;
		}
	}

	public int getRunNum() {
		return runNum;
	}

	public void setObjectiveMetric(int i, Metric m) {
		objectiveNames.put(i, m);
	}
	
	public Metric getObjectiveMetric(int i) {
		return objectiveNames.get(i);
	}
	
	public String getObjectiveMetricName(int i) {
		Metric m = objectiveNames.get(i);
		return m.getClass().getSimpleName();
	}
	
}