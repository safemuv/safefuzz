package fuzzingengine;

import javax.json.JsonObject;

public interface JSONLambda {
	JsonObject op(JsonObject input);
}
