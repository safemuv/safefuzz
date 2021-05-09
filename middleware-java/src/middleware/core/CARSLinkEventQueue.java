package middleware.core;

import java.util.List;
import java.util.Optional;

import atlassharedclasses.ATLASObjectMapper;
import fuzzingengine.*;
import fuzzingengine.operations.*;
import middleware.carstranslations.CARSTranslations;

public abstract class CARSLinkEventQueue<E> extends ATLASEventQueue<E> implements Runnable {
	private static final long serialVersionUID = 1L;
	private static final boolean CHECK_FOR_QUEUE_EVENTS_BEFORE_ANY = true;
	//protected ATLASObjectMapper atlasOMapper;
	protected ATLASCore core;
	protected FuzzingEngine<E> fuzzingEngine;
	protected CARSTranslations cTrans;
	
	public CARSLinkEventQueue(ATLASCore core, int capacity, char progressChar) {
		super(core,capacity,progressChar);
		this.core = core;
		progressChar = '.';
		this.progressChar = progressChar;
		this.fuzzingEngine = core.getFuzzingEngine();
		this.cTrans = core.getCARSTranslator();
	}
	
	public abstract void handleEventSpecifically(E event);
	
	public void checkForQueuedEvents() {
		double time = core.getTime();
		List<FutureEvent<E>> delayedEvents = fuzzingEngine.pollEventsReady(time);
		for (FutureEvent<E> delayed : delayedEvents) {
			Optional<String> reflectBackAsName = delayed.getReflectBackName();
			E event = delayed.getEvent();
			if (reflectBackAsName.isPresent()) {
				reflectEventBack(event, reflectBackAsName.get());
			}
			
			handleEventSpecifically(event);
		}
	}
	
	//@ Potentially reflect it back to the CARS
	public void reflectEventBack(E event, String reflectBackName) {
		
		if (event instanceof CARSVariableUpdate) {
			CARSVariableUpdate varUpdate = (CARSVariableUpdate)event;  
			cTrans.sendBackEvent(varUpdate, reflectBackName);
		}
	}
	
	public void handleEvent(E event) {
		if (CHECK_FOR_QUEUE_EVENTS_BEFORE_ANY) {
			checkForQueuedEvents();
		}
		
		double time = core.getTime();
		
		// TODO: Before handling custom events, check for pending events that are now due!

		// Do the fuzzing specific parts of a CARS message
		// If the message is on the list to fuzz... alter it
		List<FuzzingOperation> ops = fuzzingEngine.shouldFuzzCARSEvent(event, time);
		Optional<E> modifiedEvent_o = Optional.of(event);
		Optional<String> reflectBackAsName = fuzzingEngine.shouldReflectBackToCARS(event);
		
		// Need to determine if ANY event in the chain requires enquining
		// For now, just assume false
		boolean shouldEnqueue = false;
		double enqueueTime = 0.0;
		
		// Make the transformations on the event here
		for (FuzzingOperation op : ops) {
			if (modifiedEvent_o.isPresent()) {
				modifiedEvent_o = fuzzingEngine.fuzzTransformEvent(modifiedEvent_o, op);
				shouldEnqueue = shouldEnqueue || op.shouldEnqueue();
				enqueueTime += op.enqueueTime();
			}
		}
		
		// How to handle the case where it should be enqueued?
			
		// The event may have been destroyed or deleted by the chain of fuzzing operations. If 
		// so, it will be ignored.
		if (modifiedEvent_o.isPresent()) {
			E modifiedEvent = modifiedEvent_o.get();
			
			if (shouldEnqueue) {
				double futureTime = time + enqueueTime;
				fuzzingEngine.addToQueue(modifiedEvent, futureTime, reflectBackAsName);
			} else {
				// No enqueuing, reflect back to CARS then handle immediately
				if (reflectBackAsName.isPresent()) {
					reflectEventBack(modifiedEvent, reflectBackAsName.get());
				}
				handleEventSpecifically(modifiedEvent);
			}
		}
	}
}
