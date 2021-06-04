package carsspecific.ros.codegen;

import fuzzingengine.FuzzingEngine;
import fuzzingengine.FuzzingKeySelectionRecord;
import fuzzingengine.YAMLExtras;
import fuzzingengine.operations.FuzzingOperation;
import fuzzingengine.operations.ValueFuzzingOperation;
import middleware.atlascarsgenerator.*;
import middleware.atlascarsgenerator.carsmapping.*;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.parsers.*;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerConfigurationException;
import javax.xml.transform.TransformerException;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.TransformerFactoryConfigurationError;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.dataformat.yaml.YAMLFactory;
import com.fasterxml.jackson.dataformat.yaml.YAMLGenerator.Feature;

import atlasdsl.*;
import carsspecific.ros.rosmapping.ROSSimulation;

public class ROSCodeGen extends CARSCodeGen {
	private static ObjectMapper obj = new ObjectMapper();

	public ROSCodeGen(Mission m, Optional<FuzzingEngine> fe_o) {
		super(m, fe_o);
	}
	
	public CARSSimulation convertDSL() throws ConversionFailed {
		CARSSimulation rossim = new ROSSimulation();
		transformAllFiles();
		return rossim;	
	}
	
	public static void processLaunchFiles(FuzzingEngine fe) {
		// Get the list of all keys not in the fuzzspec
		Set<String> specKeys = fe.getSpec().getRecords().keySet();
		Set<String> confKeys = fe.getAllKeys();
		
		// Remove all keys not listed in the config
		Set<String> removalKeys = specKeys.stream()
				.filter(k -> !(confKeys.contains(k)))
				.collect(Collectors.toSet());
		
		System.out.println("specKeys =" + specKeys);
		System.out.println("confKeys =" + confKeys);
		System.out.println("removalKeys =" + removalKeys);
		
		// TODO: get custom launch file path - for now hardcoded path
		try {
			removeRemapNodes("/tmp/prepare_sim.launch", removalKeys);
		} catch (ParserConfigurationException | SAXException | IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		System.out.println("processLaunchFiles done");
	}
	
	public static void removeRemapNodes(String filename, Set<String> remapKeys)
			throws ParserConfigurationException, FileNotFoundException, SAXException, IOException {
		DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
		dbf.setValidating(false);
		DocumentBuilder db = dbf.newDocumentBuilder();

		Document doc = db.parse(new FileInputStream(new File(filename)));

		NodeList elements = doc.getElementsByTagName("remap");
		
		for (int i = 0; i < elements.getLength(); i++) {
			Element e = (Element)elements.item(i);
			System.out.println("element=" + e + "-from=" + e.getAttribute("from"));
			
			boolean shouldRemove = remapKeys.contains(e.getAttribute("from")) || remapKeys.contains(e.getAttribute("from"));
			System.out.println("from=" + e.getAttribute("from"));
			System.out.println("to=" + e.getAttribute("to"));

			// remove the specific node
			if (shouldRemove) {
				System.out.println("Removing remap entry " + e);
				e.getParentNode().removeChild(e);
			} else {
				System.out.println("Remap entry " + e + " not found in key removal list");
			}
		}
		// Normalise the DOM tree, puts all text nodes in the
		// full depth of the sub-tree underneath this node
		doc.normalize();
		
		// Write it out
		Transformer tf;
		try {
			tf = TransformerFactory.newInstance().newTransformer();
			tf.setOutputProperty(OutputKeys.ENCODING, "UTF-8");
			tf.setOutputProperty(OutputKeys.INDENT, "yes");
			Writer out = new FileWriter(filename);
			tf.transform(new DOMSource(doc), new StreamResult(out));
			out.close();
		} catch (TransformerConfigurationException | TransformerFactoryConfigurationError e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (TransformerException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public static void transformFiles(FuzzingEngine fe, Set<FuzzingKeySelectionRecord> recs) {
		ObjectMapper mapper = new ObjectMapper(new YAMLFactory().disable(Feature.WRITE_DOC_START_MARKER));
		mapper.findAndRegisterModules();
		try {
			// Iterate over all the fuzzing key selection records
			for (FuzzingKeySelectionRecord r : recs) {
				// TODO: only those with keys on environmental components should be considered
				FuzzingOperation op = r.getOperation();
				String filename = fe.getFilenameForKey(r.getKey());
				
				Object internalSpec = r.getGroupNum();
				File f = new File(filename);
				JsonNode res = (JsonNode)mapper.readTree(new File(filename));
				String [] specFields = ((String)internalSpec).split("\\.");
				//System.out.println("Res class=" + res.getClass());
				
				// Only ValueFuzzingOperations can be applied here... it only makes sense for them
				// to be used, since others such as delay cannot be sensibly applied 
				if (op instanceof ValueFuzzingOperation) {
					ValueFuzzingOperation opV = (ValueFuzzingOperation)op;
					YAMLExtras.fuzzTransformYAML(obj, res, specFields, opV);
				}
				// Write back the modified file
				mapper.writeValue(f, res);
			}
		} catch (JsonProcessingException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public void transformAllFiles() {
		if (fe_o.isPresent()) {
			FuzzingEngine fe = fe_o.get();
			Set<FuzzingKeySelectionRecord> records = fe.getAllEnvironmentalKeys();
			System.out.println("transformAllFiles = " + records);
			transformFiles(fe, records);
			
			processLaunchFiles(fe);
		}
		
	}
}
