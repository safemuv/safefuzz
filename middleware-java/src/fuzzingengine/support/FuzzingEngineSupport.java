package fuzzingengine.support;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import atlasdsl.Mission;
import fuzzingengine.FuzzingEngine;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.spec.GeneratedFuzzingSpec;

public class FuzzingEngineSupport {
	public static List<FuzzingKeySelectionRecord> loadFuzzingRecords(Mission mission, String csvFileName) {
		FuzzingEngine fe = GeneratedFuzzingSpec.createFuzzingEngine(mission, false);
		fe.setupFromFuzzingFile(csvFileName, mission);
		Map<String,FuzzingKeySelectionRecord> map = fe.getConfig().getKeyLookup();
		return new ArrayList<FuzzingKeySelectionRecord>(map.values());
	}
}