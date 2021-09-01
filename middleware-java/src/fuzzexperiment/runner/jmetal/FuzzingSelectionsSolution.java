package fuzzexperiment.runner.jmetal;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;

import org.uma.jmetal.solution.*;

import atlasdsl.*;
import fuzzingengine.FuzzingSelectionRecord;

public class FuzzingSelectionsSolution implements Solution<FuzzingSelectionRecord> {
	private static final long serialVersionUID = 1L;

	private Mission mission;
	private boolean actuallyRun;
	private String exptTag;
	private double exptRunTime;
	String baseDir = "/tmp";
	
	private Map<Object,Object> attributes = new HashMap<Object,Object>();
	private Map<Integer,Double> objectives = new HashMap<Integer,Double>();
	private Map<Integer,Double> constraints = new HashMap<Integer,Double>();
	private List<FuzzingSelectionRecord> contents = new ArrayList<FuzzingSelectionRecord>();

	private String csvFileName;
	
	public FuzzingSelectionsSolution(Mission mission, String exptTag, boolean actuallyRun, double exptRunTime) {
		this.mission = mission;
		this.exptTag = exptTag;
		this.actuallyRun = actuallyRun;
		this.exptRunTime = exptRunTime;
	}
	
	public FuzzingSelectionsSolution(Mission mission, String exptTag, boolean actuallyRun, double exptRunTime, List<FuzzingSelectionRecord> recs) {
		this.mission = mission;
		this.exptTag = exptTag;
		this.actuallyRun = actuallyRun;
		this.exptRunTime = exptRunTime;
		this.contents = new ArrayList<FuzzingSelectionRecord>(recs.size());
		
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
		
		for (FuzzingSelectionRecord fi : contents) {
			this.contents.add(fi.dup());
		}
	}
	
	public static FuzzingSelectionsSolution empty(FuzzingSelectionsSolution other) {
		FuzzingSelectionsSolution fi = new FuzzingSelectionsSolution(other.mission, other.exptTag, other.actuallyRun, other.exptRunTime);
		fi.contents = new ArrayList<FuzzingSelectionRecord>();
		return fi;
	}
		
	public void setObjective(int index, double value) {
		objectives.put(index,value);
	}
	
	public double getObjective(int index) {
		return objectives.get(index);
	}
	
	public double[] getObjectives() {
		int size = objectives.size();
		double [] res = new double [size];
		for (int i = 0; i < size; i++) {
			res[i] = objectives.get(i);
		}
		return res;
	}

	public FuzzingSelectionRecord getVariable(int index) {
		System.out.println("index = " + index +  ",contents.size()=" + contents.size());
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
		double [] res = new double [size];
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
	
//	private double totalActiveFaultTimeLengthScaledByIntensity() {
//		double total = 0.0;
//		for (FuzzingSelectionRecord fs : contents) {
//			if (fs.isActive()) {
//				total += FuzzingSelectionIntensity(fs) * (fs.getEndTime() - fs.getStartTime());
//			}
//		}
//		return total;
//	}
	
//	public double faultCostProportion() {
//		return ((totalActiveFaultTimeLengthScaledByIntensity() / (contents.size() * exptRunTime)));
//	}

	public double getConstraint(int index) {
		//return faultCostProportion();
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

	public Solution<FuzzingSelectionRecord> copy() {
		FuzzingSelectionsSolution copy = new FuzzingSelectionsSolution(this);
		return copy;
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
		attributes.put(id,value);
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
	
	public String toString() {
		return contents.toString();
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
		this.csvFileName = baseDir + "/" + UUID.randomUUID().toString() + ".csv";
		generateCSVFile(csvFileName);
		return csvFileName;
	}
	
	public String getCSVFileName() throws IOException {
		if (csvFileName == null) {
			return createCSVFileName();
		} else {
			return csvFileName;
		}
	}
}