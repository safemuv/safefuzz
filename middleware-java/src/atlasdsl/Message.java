package atlasdsl;

import java.util.List;

public class Message {
	private String name;
	private Component from;
	private Component to;
	private double minFrequency;
	private double maxFrequency;
	private List<MessageField> fields;
	
	public Message(String name, Component from, Component to) {
		this.name = name;
		this.from = from;
		this.to = to;
	}
	
	public boolean isTo(Component c) {
		return (this.to == c);
	}
	
	public boolean isFrom(Component c) {
		return (this.from == c);
	}
	
	public Component getTo() {
		return to;
	}
	
	public Component getFrom() {
		return from;
	}
	
	public String getName() {
		return name;
	}
	
	public void setMinFrequency(double minFrequency) {
		this.minFrequency = minFrequency;
	}
	
	public void setMaxFrequency(double maxFrequency) {
		this.maxFrequency = maxFrequency;
	}
}
