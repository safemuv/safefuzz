package carsspecific.ros.carsqueue;

import java.util.Set;

import javax.json.*;
import atlasdsl.*;
import atlassharedclasses.GPSPositionReading;
import atlassharedclasses.Point;
import carsspecific.ros.carsqueue.ROSTopicUpdate.ATLASTag;
import carsspecific.ros.connection.ROSConnection;

import edu.wpi.rail.jrosbridge.*;
import edu.wpi.rail.jrosbridge.callback.TopicCallback;
import edu.wpi.rail.jrosbridge.messages.Message;
import fuzzingengine.FuzzingConfig;
import fuzzingengine.FuzzingEngine;
import middleware.core.*;

public class ROSEventQueue extends CARSLinkEventQueue<ROSEvent> {

	private final boolean DEBUG_PRINT_RAW_MESSAGE = false;
	private Mission mission;
	private Ros ros;
	private static final long serialVersionUID = 1L;
	private FuzzingEngine fuzzEngine;

	public ROSEventQueue(ATLASCore core, Mission mission, int queueCapacity, FuzzingEngine fuzzEngine) {
		super(core, queueCapacity, '.');
		this.mission = mission;
		this.fuzzEngine = fuzzEngine;
	}

	public void run() {
		super.run();
	}

	public void registerAfter() {

	}

	public void handleEventSpecifically(ROSEvent e) {
		if (e instanceof ROSTopicUpdate) {
			ROSTopicUpdate rtu = (ROSTopicUpdate) e;
			if (rtu.tagEquals(ATLASTag.POSE)) {
				JsonObject j = rtu.getJSON();
				JsonObject pose = j.getJsonObject("pose");
				JsonObject pos = pose.getJsonObject("position");
				JsonNumber jx = pos.getJsonNumber("x");
				JsonNumber jy = pos.getJsonNumber("y");
				JsonNumber jz = pos.getJsonNumber("z");
				Point p = new Point(jx.doubleValue(), jy.doubleValue(), jz.doubleValue());
				System.out.println("ATLAS Point:" + p.toString());
				GPSPositionReading gps = new GPSPositionReading(p, 0.0, rtu.getVehicleName());
				core.notifyPositionUpdate(gps);
			}

			if (rtu.tagEquals(ATLASTag.VELOCITY)) {
				JsonObject j = rtu.getJSON();
				JsonObject tw = j.getJsonObject("twist");
				JsonObject linear = tw.getJsonObject("linear");
				JsonNumber jx = linear.getJsonNumber("x");
				JsonNumber jy = linear.getJsonNumber("y");
				JsonNumber jz = linear.getJsonNumber("z");
				Point vel = new Point(jx.doubleValue(), jy.doubleValue(), jz.doubleValue());
				System.out.println("Vel:" + vel.toString());
			}
		}
	}
	
	private void standardSubscribe(String fullTopicName, String rosType) {
		ROSEventQueue rosQueue = this;
		Topic t = new Topic(ros, fullTopicName, rosType);
		ATLASTag tag = ATLASTag.SIMULATOR_GENERAL;
		t.subscribe(new TopicCallback() {
			@Override
			public void handleMessage(Message message) {
				if (DEBUG_PRINT_RAW_MESSAGE) {
					System.out.println("From ROSbridge tagged: " + tag.toString() + ":" + message.toString());
				};
				
				ROSEvent rev = new ROSTopicUpdate(tag, fullTopicName, message, core.getTime(), rosType);
				rosQueue.add(rev);
			}
		});
		
	}

	private void standardSubscribeVehicle(String vehicleName, ATLASTag tag, String topicName, String rosType) {
		ROSEventQueue rosQueue = this;
		String topicNameFull = "/" + vehicleName + topicName;
		Topic t = new Topic(ros, topicNameFull, rosType);
		t.subscribe(new TopicCallback() {
			@Override
			public void handleMessage(Message message) {
				if (DEBUG_PRINT_RAW_MESSAGE) {
					System.out.println("From ROSbridge tagged: " + tag.toString() + ":" + message.toString());
				};
				
				ROSEvent rev = new ROSTopicUpdate(vehicleName, tag, topicName, message, core.getTime(), rosType);
				rosQueue.add(rev);
			}
		});
	}

	private void subscribeForStandardVehicleTopics(String vehicleName) {
		// We always require the position and velocity of vehicles for the middleware state
		String velTopicName = "/ual/velocity";
		String posTopicName = "/ual/pose";
		String velType = "geometry_msgs/TwistStamped";
		String posType = "geometry_msgs/PoseStamped";
		standardSubscribeVehicle(vehicleName, ATLASTag.VELOCITY, velTopicName, velType);
		standardSubscribeVehicle(vehicleName, ATLASTag.POSE, posTopicName, posType);
	}
	
	private void setupTopicForSubscription() {
		// TODO: create a topic and do the subscription here
	}
	
	private void subscribeForFuzzingTopics() {
		// TODO: need a connection from the fuzzing engine to this queue
		
		// Need the types as well for the keys
		// Also need to know if the topics are per-robot or not...
		Set<String> keys = fuzzEngine.getAllKeys();
		FuzzingConfig confs = fuzzEngine.getConfig();
		
		for (String k : keys) {
			
		}
	}
	
	private void subscribeForGoalTopics() {
		// TODO: add the messages from the mission
	}
	
	private void subscribeForSimulatorTopics() {
		// Need the simulator time...
		// standardSubscribe("/clock", "CLOCK_TYPE");
	}

	public void setup() {
		ros = ROSConnection.getConnection().getROS();
		subscribeForSimulatorTopics();
		
		// Iterate over all the robots in the DSL, set up subscriptions for position and
		// velocity
		for (Robot r : mission.getAllRobots()) {
			subscribeForStandardVehicleTopics(r.getName());
		}
	}

	public void close() {
		ros.disconnect();
	}
}