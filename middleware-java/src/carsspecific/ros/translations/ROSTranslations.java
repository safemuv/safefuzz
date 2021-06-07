package carsspecific.ros.translations;

import java.util.HashMap;
import java.util.List;

import javax.json.Json;
import javax.json.JsonObject;

import atlassharedclasses.Point;
import carsspecific.ros.connection.ROSConnection;
import edu.wpi.rail.jrosbridge.Ros;
import edu.wpi.rail.jrosbridge.Topic;
import edu.wpi.rail.jrosbridge.messages.Message;
import middleware.carstranslations.CARSTranslations;
import middleware.core.ActiveMQProducer;

public class ROSTranslations extends CARSTranslations {
	HashMap<String,ActiveMQProducer> producers;

	private Ros ros;
	
	public ROSTranslations() {
		ros = ROSConnection.getConnection().getROS();
	}
	
	public void setOutputProducers(HashMap<String,ActiveMQProducer> producers) {
		this.producers = producers;
	}
	
	public synchronized void sendCARSUpdate(String robotName, Object key, Object value) {
		System.out.println("ROSTranslations: sendCARSUpdate unimplemented");
	}
	
	public synchronized void vehicleStatusChange(String robotName, boolean newStatus) { 
		System.out.println("ROSTranslations: startRobot unimplemented");
	}

	public void setCoordinates(String robotName, List<Point> coords) {
		System.out.println("ROSTranslations: setCoordinates unimplemented");	
	}
	
	public void setVelocity(String robotName, Point newVel) {
		String topicName = "/" + robotName + "/ual/set_velocity";
		Topic echo = new Topic(ros, topicName, "geometry_msgs/TwistStamped");
		JsonObject velUpdate = Json.createObjectBuilder().add("header", Json.createObjectBuilder().add("seq", 0)
				.add("stamp", Json.createObjectBuilder().add("secs", 0).add("nsecs", 0)).add("frame_id", "odom"))
				.add("twist",
						Json.createObjectBuilder()
								.add("linear", Json.createObjectBuilder().add("x", newVel.getX()).add("y", newVel.getY()).add("z", newVel.getZ()))
								.add("angular", Json.createObjectBuilder().add("x", 0).add("y", 0).add("z", 0)))
				.build();
		Message velUpdate_m = new Message(velUpdate);
		echo.publish(velUpdate_m);
	}
	
	public void returnHome(String robotName) {
		System.out.println("ROSTranslations: setCoordinates unimplemented");			
	}

	public void setCoordinates(String robotName, List<Point> coords, int repeatCount) {
		
	}

	public void startVehicle(String robotName) {
		
	}

	public void stopVehicle(String robotName) {
		
	}
	
	public void sendBackJSON(String robotName, String localTopicName, JsonObject jo, String rosTypeName) {
		String topicName = "/" + robotName + localTopicName;
		Topic echo = new Topic(ros, topicName, rosTypeName);
		Message msg = new Message(jo);
		System.out.println("ROSTranslations.sendBackJSON: sending back: robotName=" + robotName + ",localTopicName=" + localTopicName + ",json=" + jo.toString());
		echo.publish(msg);
	}
	
	public void sendBackJSON(String topicName, JsonObject jo, String rosTypeName) {
		// TODO: should this Topic object be preserved - resource problem
		Topic echo = new Topic(ros, topicName, rosTypeName);
		Message msg = new Message(jo);
		System.out.println("ROSTranslations.sendBackJSON: sending back: topicName=" + topicName + ",json=" + jo.toString());
		echo.publish(msg);
	}
}
