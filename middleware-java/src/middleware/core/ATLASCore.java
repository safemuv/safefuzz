package middleware.core;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import atlasdsl.*;
import atlasdsl.faults.*;
import atlassharedclasses.*;
import faultgen.FaultGenerator;
import middleware.gui.GUITest;
import middleware.missionmonitor.*;

// This code will be combined with the simulator-specific code
// during code generation
public abstract class ATLASCore {
	protected ATLASEventQueue carsIncoming;
	protected ATLASEventQueue fromCI;
	
	//protected ATLASEventQueue fromFaultGen;
	// for now the fault generator is installed in the middleware process itself,
	// not communicating over ActiveMQ with it
	
	protected ActiveMQProducer outputToCI;
	private final int CI_QUEUE_CAPACITY = 100;
	protected Mission mission;
	protected MissionMonitor monitor;
	
	private GUITest gui;
	protected List<ATLASEventQueue> queues = new ArrayList<ATLASEventQueue>();
	protected List<FaultInstance> activeFaults = new ArrayList<FaultInstance>();
	private FaultGenerator faultGen;
	
	private double time = 0.0;
	
	public ATLASCore(Mission mission) {
		this.mission = mission;
		this.monitor = new MissionMonitor(this, mission);
		fromCI = new CIEventQueue(this, mission, CI_QUEUE_CAPACITY);
		queues.add(fromCI);
		faultGen = new FaultGenerator(this,mission);
		gui = new GUITest(mission, faultGen);
	}
	
	public synchronized void registerFault(FaultInstance fi) {
		activeFaults.add(fi);
		System.out.println("Fault added");
		Fault f = fi.getFault();
		f.immediateEffects(this);
	}
	
	public void clearFaults() {
		activeFaults.clear();
	}
    
    public ActiveMQProducer getCIProducer() {
    	return outputToCI;
    }
	
    public void runMiddleware()  {
		for (ATLASEventQueue q : queues) {
			// Since the GUI displays global status, it
			// needs to be updated following every event on any queue
			q.registerAfterHook(() -> gui.updateGUI());
			// Also after events, need to check for faults
			q.registerAfterHook(() -> faultGen.pollFaultsNow());
			q.registerAfterHook(() -> monitor.runStep());
			q.setup();
		}
		
		for (ATLASEventQueue q : queues) {
			new Thread(q).start();
		}
    }

	public double getTime() {
		return time;
	}
	
	public void updateTime(double time) throws CausalityException {
		if ((time < this.time)) {
			this.time = time;
		}
	}
	
	// This is used by active faults to inject their immediate effects
	// upon the low-level CARS simulation
	public void sendToCARS(Robot r, String key, String value) {
		CIEventQueue CIq = (CIEventQueue)fromCI;
		CIq.sendToCARS(r, key, value);
	}

	public List<FaultInstance> activeFaultsOfClass(Class class1) {
		return activeFaults.stream()
				.filter(f -> f.getClass() == class1)
				.collect(Collectors.toList());
	}
}