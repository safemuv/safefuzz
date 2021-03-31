package test.modelloader;

import java.io.File;
import java.lang.reflect.InvocationTargetException;
import java.net.URISyntaxException;
import java.util.ArrayList;
import org.eclipse.epsilon.common.util.StringProperties;
import org.eclipse.epsilon.emc.emf.EmfModel;
import org.eclipse.epsilon.eol.exceptions.models.EolModelLoadingException;
import org.eclipse.epsilon.eol.models.IRelativePathResolver;

public class GenerateModelsLoaderTest  {
	public void run(String modelFilePath) throws InvocationTargetException, InterruptedException, EolModelLoadingException {
			EmfModel model = new EmfModel();
			model.setName("Source");
			
			System.out.println("Model " + modelFilePath + ": starting load");
			ArrayList<String> metamodelFiles = new ArrayList<String>();
			metamodelFiles.add(new File("ecore-metamodels-combined/atlas.ecore").getAbsolutePath());
			
			model.setMetamodelFiles(metamodelFiles);
			model.setModelFile(new File(modelFilePath).getAbsolutePath());
			model.load();
			System.out.println("Model " + modelFilePath + ": loaded OK!");
			model.close();
	}
	
	protected EmfModel createEmfModel(String name, String model, 
			String metamodel, boolean readOnLoad, boolean storeOnDisposal) 
					throws EolModelLoadingException, URISyntaxException {
		EmfModel emfModel = new EmfModel();
		StringProperties properties = new StringProperties();
		properties.put(EmfModel.PROPERTY_NAME, name);
		properties.put(EmfModel.PROPERTY_FILE_BASED_METAMODEL_URI,
				getFileURI(metamodel).toString());
		properties.put(EmfModel.PROPERTY_MODEL_URI, 
				getFileURI(model).toString());
		properties.put(EmfModel.PROPERTY_READONLOAD, readOnLoad + "");
		properties.put(EmfModel.PROPERTY_STOREONDISPOSAL, 
				storeOnDisposal + "");
		emfModel.load(properties, (IRelativePathResolver) null);
		return emfModel;
	}
	
	protected java.net.URI getFileURI(String fileName) throws URISyntaxException {
		if (GenerateModelsLoaderTest.class.getResource(fileName)==null) {
			System.out.println("getresource returned null for fileName " + fileName);
		}
		
		java.net.URI binUri = GenerateModelsLoaderTest.class.getResource(fileName).toURI();
		return binUri.toString().contains("bin") ? new java.net.URI(binUri.toString().replaceAll("bin", "src")) : binUri;
	}
	
}
