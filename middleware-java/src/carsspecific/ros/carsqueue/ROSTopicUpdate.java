package carsspecific.ros.carsqueue;

import javax.json.JsonObject;
import edu.wpi.rail.jrosbridge.messages.Message;

public class ROSTopicUpdate extends ROSEvent {
	private String vehicleName;
	private String topicName;
	private JsonObject json;
	private Message message;
	private double time;
	private ATLASTag atlasTag;
	
	public enum ATLASTag {
		VELOCITY,
		POSE
	}
	
	public ROSTopicUpdate(String vehicleName, ATLASTag atlasTag, String topicName, Message message, double time) {
		this.vehicleName = vehicleName;
		this.topicName = topicName;
		this.message = message;
		this.time = time;
		this.atlasTag = atlasTag;
		this.json = message.toJsonObject();
	}
	
	public boolean tagEquals(ATLASTag t) {
		return atlasTag.equals(t);
	}
	
	public Message getMessage() {
		return message;
	}
	
	public JsonObject getJSON() {
		return json;
	}
	
	public String getVehicleName() {
		return vehicleName;
	}
	
	public String getTopicName() {
		return topicName;
	}
	
	public double getTime() {
		return time;
	}
}
