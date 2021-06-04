package test.fuzzingengine.parsingxml;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.StringWriter;
import java.io.Writer;
import java.util.ArrayList;
import java.util.List;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.xml.sax.SAXException;

public class TestXMLRemoval {

	public static void removeRemapNodes(String filename, List<String> remapKeys)
			throws ParserConfigurationException, FileNotFoundException, SAXException, IOException {
		DocumentBuilderFactory dbf = DocumentBuilderFactory.newInstance();
		dbf.setValidating(false);
		DocumentBuilder db = dbf.newDocumentBuilder();

		Document doc = db.parse(new FileInputStream(new File(filename)));

		// retrieve the element 'link'
		for (String k : remapKeys) {
			Element element = (Element) doc.getElementsByTagName("remap").item(0);
			boolean shouldRemove = element.getAttribute("from").equals(k) || element.getAttribute("to").equals(k);
			System.out.println("from=" + element.getAttribute("from"));
			System.out.println("to=" + element.getAttribute("to"));

			// remove the specific node
			if (shouldRemove) {
				System.out.println("Removing remap entry with key " + k);
				element.getParentNode().removeChild(element);
			} else {
				System.out.println("Remap entry with key " + k + " not in removal list");
			}
		}
		// Normalise the DOM tree, puts all text nodes in the
		// full depth of the sub-tree underneath this node
		doc.normalize();
		try {
			prettyPrint(doc);
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	public static void main(String[] args) throws Exception {
		List<String> removeKeys = new ArrayList<String>();
		removeKeys.add("ual/velocity");
		removeRemapNodes("/tmp/prepare_sim.launch", removeKeys);
	}

	public static final void prettyPrint(Document xml) throws Exception {
		Transformer tf = TransformerFactory.newInstance().newTransformer();
		tf.setOutputProperty(OutputKeys.ENCODING, "UTF-8");
		tf.setOutputProperty(OutputKeys.INDENT, "yes");
		Writer out = new StringWriter();
		tf.transform(new DOMSource(xml), new StreamResult(out));
		System.out.println(out.toString());
	}

}