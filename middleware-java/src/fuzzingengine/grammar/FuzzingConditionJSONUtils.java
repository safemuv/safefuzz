package fuzzingengine.grammar;

import java.io.StringReader;
import javax.json.*;

import it.units.malelab.jgea.representation.tree.Tree;

public class FuzzingConditionJSONUtils {
	public static JsonObject conditionNodeToJSON(Tree<String> node) {
		JsonObjectBuilder builder = Json.createObjectBuilder();
		// Add children
		builder.add("N", node.content());
		JsonArrayBuilder childrenBuilder = Json.createArrayBuilder();
		node.childStream().forEach(c -> {
			childrenBuilder.add(conditionNodeToJSON(c));
		});
		
		if (node.nChildren() > 0) {
			builder.add("C", childrenBuilder.build());
		}
		return builder.build();
	}
	
	public static JsonObject conditionToJSON(Tree<String> tree) {
		return conditionNodeToJSON(tree);
	}
	
	public static String conditionToJSONString(Tree<String> tree) {
		JsonObject o = conditionNodeToJSON(tree);
		String replaced = o.toString().replaceAll(",", "\\|");
		return replaced;
	}
	
	public static Tree<String> conditionFromJSONNode(JsonObject obj, Tree<String> parent) {
		JsonString n = (JsonString)obj.get("N");
		String name = n.getString();
		Tree<String> t = new Tree<String>(name, parent);
		JsonArray c = (JsonArray)obj.get("C");
		if (c != null) {
			for (int i = 0; i < c.size(); i++) {
				JsonObject v = (JsonObject)c.get(i);
				t.addChild(conditionFromJSONNode(v, t));
			}
		}
		return t;
	}
	
	public static Tree<String> conditionFromJSONString(String s) {
		String original = s.replaceAll("\\|", ",");
		JsonObject obj = Json.createReader(new StringReader(original)).readObject();
		return conditionFromJSONNode(obj,null);
	}
}
