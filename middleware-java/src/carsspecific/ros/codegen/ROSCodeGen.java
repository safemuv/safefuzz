package carsspecific.ros.codegen;

import carsspecific.ros.rosmapping.*;
import fuzzingengine.FuzzingEngine;
import middleware.atlascarsgenerator.*;
import middleware.atlascarsgenerator.carsmapping.*;

import java.util.Optional;

import atlasdsl.*;

public class ROSCodeGen extends CARSCodeGen {
	// TODO: how to specify the sensor behaviour
	public ROSCodeGen(Mission m, Optional<FuzzingEngine> fe_o) {
		super(m, fe_o);
	}
	
	public CARSSimulation convertDSL() throws ConversionFailed {
		CARSSimulation rossim = new ROSSimulation();
		return rossim;	
	}}
