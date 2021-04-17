package exptrunner.metrics;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Scanner;
import java.util.stream.Collectors;

import atlasdsl.Mission;
import exptrunner.jmetal.InvalidMetrics;

public class MetricsProcessing {
	private List<Metrics> metrics;
	private FileWriter tempLog;
	
	public enum MetricStateKeys {
		BENIGN_OBJECTS_IN_MISSION,
		MALICIOUS_OBJECTS_IN_MISSION,
		VERIFICATIONS_PER_BENIGN_OBJECT,
		VERIFICATIONS_PER_MALICIOUS_OBJECT,
	}
	
	private HashMap<MetricStateKeys,Object> metricState = new HashMap<MetricStateKeys,Object>();
	
	public MetricsProcessing(List<Metrics> metrics, FileWriter tempLog) {
		this.metrics = metrics;
		this.tempLog = tempLog;
	}
	
//	private static final int DETECTIONS_PER_OBJECT_EXPECTED = 2;
	
	public List<Metrics> getMetrics() {
		return metrics;
	}
	
	public int readObstacleFileObsCount(File obstacleFile) {
		Scanner reader;
		int count = 0;
		try {
			reader = new Scanner(obstacleFile);
			// Find the result from the line num
			while (reader.hasNextLine()) {
				String line = reader.nextLine();
				String[] fields = line.split(",");
				String robotName = fields[0];
				Integer collisionCount = Integer.valueOf(fields[1]);
				count += collisionCount;
			}
			reader.close();
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return count;
	}
	
	public double detectionCompletionTime(Map<Integer, List<Double>> detectionInfo, int objectCount) {
		double missionCompletionTime = 0;
		// Find latest of the first 2 for all these
		for (int i = 0; i < objectCount; i++) {
			List<Double> res = detectionInfo.get(i);
			if (res != null) {
				Collections.sort(res);

				if (res.size() < 2) {
					// If less than 1 detection/verification per object, the mission
					// was never completed

					// TODO: mission end time (defined) instead of MAX_VALUE here?

					return Double.MAX_VALUE;
				} else {
					missionCompletionTime = Math.max(missionCompletionTime, res.get(0));
					missionCompletionTime = Math.max(missionCompletionTime, res.get(1));
				}
			} else {
				// If one object was never detected, the mission was not completed
				return Double.MAX_VALUE;
			}
		}

		return missionCompletionTime;
	}
	
	public void registerDetectionAtTime(Map<Integer, List<Double>> detectionInfo, double time, int label) {
		if (!detectionInfo.containsKey(label)) {
			detectionInfo.put(label, new ArrayList<Double>());
		}
		detectionInfo.get(label).add(time);
	}

	public int missedDetections(Map<Integer, List<Double>> detectionInfo, int objectCount) {
		int missedTotal = 0;
		int foundTotal = 0;
		for (int i = 0; i < objectCount; i++) {
			List<Double> res = detectionInfo.get(i);
			if (res != null) {
				foundTotal += detectionInfo.get(i).size();
				foundTotal = Math.min(foundTotal, DETECTIONS_PER_OBJECT_EXPECTED);
				missedTotal += DETECTIONS_PER_OBJECT_EXPECTED - foundTotal;
			} else {
				// If no result for this object, add the number of detections intended
				missedTotal += DETECTIONS_PER_OBJECT_EXPECTED;
			}
		}
		return missedTotal;
	}
	
	public Map<Metrics,Object> readMetricsFromLogFiles(String logFileDir) throws InvalidMetrics {
		
		int objectsInMission = 0;
		if (metricState.containsKey(MetricStateKeys.BENIGN_OBJECTS_IN_MISSION)) {
			objectsInMission = (int)metricState.get(MetricStateKeys.OBJECTS_IN_MISSION);
		}
		
		// Read the goal result file here - process the given goals
		// Write it out to a common result file - with the fault info
		File f = new File(logFileDir + "/goalLog.log");
		File pf = new File(logFileDir + "/objectPositions.log");
		File robotDistFile = new File(logFileDir + "/robotDistances.log");
		File obstacleFile = new File(logFileDir + "/obstacleCollisions.log");
		File energyFile = new File(logFileDir + "/robotEnergyAtEnd.log");
		
		int detections = 0;
		int missedDetections = 0;
		int avoidanceViolations = 0;
		int maxObjectNum = 0;
		int checkDetectionCount = 0;
		
		int outsideRegionViolations = 0;

		double firstFaultTime = Double.MAX_VALUE;
		
		Map<Metrics,Object> metricResults = new HashMap<Metrics,Object>();

		// The map entry stores as a pair the number of detections and the latest time
		Map<Integer, List<Double>> detectionInfo = new HashMap<Integer, List<Double>>();

		Scanner reader;
		try {
			reader = new Scanner(f);
			while (reader.hasNextLine()) {
				String line = reader.nextLine();
				String[] fields = line.split(",");
				String goalClass = fields[0];
				if (goalClass.equals("atlasdsl.DiscoverObjects")) {
					double time = Double.parseDouble(fields[1]);
					String robot = fields[2];
					int num = Integer.parseInt(fields[3]);
					checkDetectionCount++;

					if (num > maxObjectNum) {
						maxObjectNum = num;
					}

					registerDetectionAtTime(detectionInfo, time, num);
				}

				if (goalClass.equals("atlasdsl.AvoidOthers")) {
					avoidanceViolations += 1;
					double time = Double.parseDouble(fields[2]);
					if (time < firstFaultTime) {
						firstFaultTime = time;
					}
				}
				
				if (goalClass.equals("atlasdsl.StayInRegion")) {
					int count = Integer.parseInt(fields[1]);
					outsideRegionViolations = Math.max(count,outsideRegionViolations);
				}
			}

			missedDetections = Math.max(0, ((objectsInMission * DETECTIONS_PER_OBJECT_EXPECTED)
					- checkDetectionCount));
			reader.close();

		} catch (FileNotFoundException e1) {
			e1.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}

		// TODO: better way, get max_dist this from a region?
		double MAX_DIST = 1000.0;

		double missedDetectionFactor = 10.0;
		double avoidanceFactor = 10.0;

		double combinedDistMetric = missedDetections;
		double avoidanceMetric = avoidanceViolations;

		try {
			reader = new Scanner(pf);
			while (reader.hasNextLine()) {
				String line = reader.nextLine();
				String[] fields = line.split(",");
				String label = fields[0];
				double dist = Double.valueOf(fields[1]);
				double sensorWorkingDist = Double.valueOf(fields[2]);
				System.out.println("Robot dist to label " + label + "=" + dist + "\n");
				combinedDistMetric += dist;
			}

			System.out.println(
					"Total distance: " + combinedDistMetric + ",missedDetections = " + missedDetections + "\n");
			combinedDistMetric += (missedDetections * missedDetectionFactor);
			System.out.println("Output metric: " + combinedDistMetric);

			reader.close();

			reader = new Scanner(robotDistFile);
			while (reader.hasNextLine()) {
				String line = reader.nextLine();
				String[] fields = line.split(",");
				String label = fields[0];
				double dist = Double.valueOf(fields[1]);
				avoidanceMetric += dist;
			}

			avoidanceMetric += (avoidanceViolations * avoidanceFactor);
			reader.close();

			double detectionCompletionTime = detectionCompletionTime(detectionInfo, objectsInMission);

			// Set the output metrics
			int metricID = 0;
			int constraintID = 0;
			List<String> names = new ArrayList<String>();

			if (metrics.contains(Metrics.TOTAL_ENERGY_AT_END) || metrics.contains(Metrics.MEAN_ENERGY_AT_END)) {
				Scanner energyReader;
				try {
					energyReader = new Scanner(energyFile);
					double energyTotal = 0.0;
					double count = 0;
					
					while (energyReader.hasNextLine()) {
						String line = energyReader.nextLine();
						String [] fields = line.split(",");
						String name = fields[0];
						Double energyOnRobot = Double.parseDouble(fields[1]);
						System.out.println("Robot " + name + " has energy " + energyOnRobot);
						energyTotal += energyOnRobot;
						count++;
					}
					
					if (metrics.contains(Metrics.TOTAL_ENERGY_AT_END)) {
						metricResults.put(Metrics.TOTAL_ENERGY_AT_END, energyTotal);
						//solution.setObjective(metricID++, energyTotal);
						//names.add("totalEnergy");
					}
					
					if (metrics.contains(Metrics.MEAN_ENERGY_AT_END)) {
						double meanEnergy = energyTotal / count;
						metricResults.put(Metrics.MEAN_ENERGY_AT_END, meanEnergy);
						//solution.setObjective(metricID++, meanEnergy);
						//names.add("meanEnergy");
					}	
					energyReader.close();
					
				} catch (FileNotFoundException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				} finally {
					
				}
			}
			
			if (metrics.contains(Metrics.PURE_MISSED_DETECTIONS)) {
				//solution.setObjective(metricID++, missedDetections);
				//names.add("missedDetections");
				metricResults.put(Metrics.PURE_MISSED_DETECTIONS, missedDetections);
			}
			
			if (metrics.contains(Metrics.COMBINED_MISSED_DETECTION_DIST_METRIC)) {
				//solution.setObjective(metricID++, combinedDistMetric);
				//names.add("combinedMissedDetectionDistMetric");
				metricResults.put(Metrics.COMBINED_MISSED_DETECTION_DIST_METRIC, combinedDistMetric);
			}
			
			if (metrics.contains(Metrics.OUTSIDE_REGION_COUNT)) {
				//solution.setObjective(metricID++, outsideRegionViolations);
				metricResults.put(Metrics.OUTSIDE_REGION_COUNT, outsideRegionViolations);
			}

			if (metrics.contains(Metrics.OBSTACLE_AVOIDANCE_METRIC)) {
					int obstacleCollisionCount = readObstacleFileObsCount(obstacleFile);
//					if (obstacleCollisionCount == 0) {
//						solution.setConstraint(constraintID++, -100);
//					} else {
//						solution.setConstraint(constraintID++, 0);
//					}

					//solution.setObjective(metricID++, obstacleCollisionCount);
					//names.add("obstacleCollisions");
					metricResults.put(Metrics.OBSTACLE_AVOIDANCE_METRIC, obstacleCollisionCount);
			}

			if (metrics.contains(Metrics.AVOIDANCE_METRIC)) {
				//solution.setObjective(metricID++, avoidanceMetric);
				//names.add("avoidanceMetric");
				metricResults.put(Metrics.AVOIDANCE_METRIC, avoidanceMetric);
			}

			if (metrics.contains(Metrics.DETECTION_COMPLETION_TIME)) {
				//solution.setObjective(metricID++, detectionCompletionTime);
				//names.add("detectionCompletionTime");
				metricResults.put(Metrics.DETECTION_COMPLETION_TIME,detectionCompletionTime);
			}

			String info = String.join(",", names);
		
			String logRes = metricResults.entrySet().stream().
					map(e -> e.getKey() + "=" + e.getValue()).
					collect(Collectors.joining(","));

			System.out.println(info + "\n");
			System.out.println(logRes + "\n");
			tempLog.write(logRes + "\n");
			tempLog.flush();
			
			
		} catch (FileNotFoundException e1) {
			e1.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		return metricResults;
	}

	public Metrics getMetricByID(int i) {
		return metrics.get(i);
	}
	
	public void setMetricState(String string, Object o) {
		metricState.put(string,o);
	}
	
	public void clearMetricState() {
		metricState.clear();
	}
}
