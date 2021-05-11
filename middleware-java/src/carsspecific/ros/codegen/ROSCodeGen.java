package carsspecific.ros.codegen;

import fuzzingengine.FuzzingEngine;
import middleware.atlascarsgenerator.*;
import middleware.atlascarsgenerator.carsmapping.*;

import java.util.Optional;

import atlasdsl.*;
import carsspecific.ros.rosmapping.ROSSimulation;

public class ROSCodeGen extends CARSCodeGen {
	// TODO: how to specify the sensor behaviour
	public ROSCodeGen(Mission m, Optional<FuzzingEngine> fe_o) {
		super(m, fe_o);
	}
	
	public CARSSimulation convertDSL() throws ConversionFailed {
		CARSSimulation rossim = new ROSSimulation();
		return rossim;	
	}}
