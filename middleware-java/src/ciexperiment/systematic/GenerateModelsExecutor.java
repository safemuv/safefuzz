package ciexperiment.systematic;

import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.InvocationTargetException;
import java.net.URISyntaxException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.UUID;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EPackage;
import org.eclipse.emf.ecore.EStructuralFeature;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.emf.ecore.resource.impl.ResourceSetImpl;
import org.eclipse.emf.ecore.xmi.impl.XMIResourceFactoryImpl;
import org.eclipse.epsilon.common.util.StringProperties;
import org.eclipse.epsilon.emc.emf.EmfModel;
import org.eclipse.epsilon.eol.exceptions.EolRuntimeException;
import org.eclipse.epsilon.eol.exceptions.models.EolModelElementTypeNotFoundException;
import org.eclipse.epsilon.eol.exceptions.models.EolModelLoadingException;
import org.eclipse.epsilon.eol.models.IRelativePathResolver;

public class GenerateModelsExecutor {

	public void transformModel(String modelFilePath) throws InvocationTargetException, InterruptedException {
		try {
			//registerMMs();		
			EmfModel sourceModel = new EmfModel();
			sourceModel.setName("Source");
			
			System.out.println("Model " + modelFilePath + ": starting load");
			ArrayList<String> metamodelFiles = new ArrayList<String>();
			metamodelFiles.add(new File("ecore-metamodels-combined/atlas.ecore").getAbsolutePath());
			sourceModel.setMetamodelFiles(metamodelFiles);
			sourceModel.setModelFile(new File(modelFilePath).getAbsolutePath());
			sourceModel.load();
			System.out.println("Model " + modelFilePath + ": loaded OK!");
						
			// This is based on one version of a Mission MM where Robots extend a class called "MutableComponent".
			// This is just an example. If you need to use it with your updated MM, change the models in the "models" folder
			// In order to run you need to run this as a new Eclipse Application, create a project, create the model you
			// want to mutate, right click on it and select "SESAME --> Generate Models". The code located in here will be executed.
			
			// !!! HERE WE WRITE THE LOGIC FOR MUTATION OF MODELS !!!
			
			//System.out.println(sourceModel.allContents());
			//System.out.println(sourceModel.getAllOfKind("MutableComponent"));
			
			
			// Copy Model
			UUID uuid = UUID.randomUUID();
			Path copied = Paths.get(modelFilePath + "-" + uuid.toString() + ".model");
			Path originalPath = Paths.get(modelFilePath);
		    try {
				Files.copy(originalPath, copied, StandardCopyOption.REPLACE_EXISTING);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		    
		    System.out.println("New path for model = " + copied);
		    
		    EmfModel targetModel =  new EmfModel();
			ArrayList<String> metamodelFilesTarget = new ArrayList<String>();
			metamodelFilesTarget.add(new File("ecore-metamodels-combined/atlas.ecore").getAbsolutePath());
			targetModel.setMetamodelFiles(metamodelFiles);
			targetModel.setName("Target");
			targetModel.setModelFile(copied.toString());
			targetModel.load();
		    
		    // Simple queries to get objects by type and their attributes
			Collection<EObject> mutableObjects = new ArrayList<EObject>();
			mutableObjects = sourceModel.getAllOfKind("Robot");
			for (EObject obj : mutableObjects) {
				EStructuralFeature nameAttr = obj.eClass().getEStructuralFeature("name");
				System.out.println("FOUND ROBOT NAME = " + obj.eGet(nameAttr));
			}
			
			// Testing removing robot frank from the model
			Collection<EObject> elementsToBeRemoved = new ArrayList<EObject>();
			elementsToBeRemoved = targetModel.getAllOfKind("Robot");
			for (EObject eObj : elementsToBeRemoved) {
				EStructuralFeature nameAttr = eObj.eClass().getEStructuralFeature("name");
				if (eObj.eGet(nameAttr).equals("frank")) {
					System.out.println("Removing frank");
					targetModel.deleteElement((Object)eObj);
					break;
				}
			}
			
			// !!! At the end of the mutation for the model to be saved we need to call dispose(). Otherwise it is not saved.
			targetModel.dispose();
			sourceModel.close();
			targetModel.close();
		
			// !!! END OF MUTATION LOGIC !!!
			

		} catch (EolModelLoadingException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (EolModelElementTypeNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (EolRuntimeException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} finally {
		}
	}
	
//	public void executeEGL(String modelFilePath) {
//		
//		//emf source (your mission model)
//		EmfModel sourceModelForEGL = createAndLoadAnEmfModel(
//				"http://www.github.com/jrharbin-york/atlas-middleware/dsl/mission,http://www.github.com/jrharbin-york/atlas-middleware/dsl/region,http://www.github.com/jrharbin-york/atlas-middleware/dsl/message,http://www.github.com/jrharbin-york/atlas-middleware/dsl/fuzzing,http://www.github.com/jrharbin-york/atlas-middleware/dsl/faults,http://www.github.com/jrharbin-york/atlas-middleware/dsl/components",
//				modelFilePath, "Source", "true", "true");
//
//		//egl factory and module
//		EglFileGeneratingTemplateFactory factory = new EglFileGeneratingTemplateFactory();
//		EglTemplateFactoryModuleAdapter eglModule = new EglTemplateFactoryModuleAdapter(factory);
//		eglModule.getContext().getModelRepository().addModel(sourceModelForEGL);
//
//		// Point to where the EGL file is located
//		// TODO: replace the egl file path 
//		String rawLocation = "/home/atlas/atlas/atlas-middleware/atlas-models/genmission.egl";
//		java.net.URI EglFile = new java.net.URI(rawLocation);
//
//		EglFileGeneratingTemplate template = (EglFileGeneratingTemplate) factory.load(EglFile);
//		template.process();
//		
//		// Set the target file, ie. where the results will be generated to.
//		// TODO: replace the path for the Java generated DSL here
//		File target = new File(modelFilePath + "GeneratedFile.txt");
//		target.createNewFile();
//		template.generate(target.toURI().toString());
//		// !!! END OF EGL EXECUTION FROM JAVA !!!
//	}
	
	private void registerMMs() {
		ResourceSet resSet = new ResourceSetImpl();
		resSet.getResourceFactoryRegistry().getExtensionToFactoryMap().put("model", new XMIResourceFactoryImpl());
		resSet.getResourceFactoryRegistry().getExtensionToFactoryMap().put("ecore", new XMIResourceFactoryImpl());
		InputStream inputStream = GenerateModelsExecutor.class
				.getResourceAsStream("/models/mission.ecore");

		Resource resource = resSet.createResource(
				URI.createURI("http://www.github.com/jrharbin-york/atlas-middleware/dsl/mission"),
				"org.eclipse.emf.ecore");
		if (resource == null) {
			System.out.println("Resource is null");
			System.out.println("ResSet="+resSet.toString());
		}
		
		try {
			resource.load(inputStream, new HashMap<>());
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		for (EObject o : resource.getContents()) {
			if (o instanceof EPackage) {
				EPackage.Registry.INSTANCE.put(((EPackage) o).getNsURI(), o);
			}
		}

		Resource resource2 = resSet.createResource(URI.createURI("http://www.github.com/jrharbin-york/atlas-middleware/dsl/components"),
				"org.eclipse.emf.ecore");
		InputStream inputStream2 = GenerateModelsExecutor.class
				.getResourceAsStream("/models/components.ecore");
		try {
			resource2.load(inputStream2, new HashMap<>());
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		for (EObject o : resource2.getContents()) {
			if (o instanceof EPackage) {
				EPackage.Registry.INSTANCE.put(((EPackage) o).getNsURI(), o);
			}
		}
		
		Resource resource3 = resSet.createResource(URI.createURI("http://www.github.com/jrharbin-york/atlas-middleware/dsl/faults"),
				"org.eclipse.emf.ecore");
		InputStream inputStream3 = GenerateModelsExecutor.class
				.getResourceAsStream("/models/faults.ecore");
		try {
			resource3.load(inputStream3, new HashMap<>());
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		for (EObject o : resource3.getContents()) {
			if (o instanceof EPackage) {
				EPackage.Registry.INSTANCE.put(((EPackage) o).getNsURI(), o);
			}
		}
		
		Resource resource4 = resSet.createResource(URI.createURI("http://www.github.com/jrharbin-york/atlas-middleware/dsl/fuzzing"),
				"org.eclipse.emf.ecore");
		InputStream inputStream4 = GenerateModelsExecutor.class
				.getResourceAsStream("/models/fuzzing.ecore");
		try {
			resource4.load(inputStream4, new HashMap<>());
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		for (EObject o : resource4.getContents()) {
			if (o instanceof EPackage) {
				EPackage.Registry.INSTANCE.put(((EPackage) o).getNsURI(), o);
			}
		}
		
		Resource resource5 = resSet.createResource(URI.createURI("http://www.github.com/jrharbin-york/atlas-middleware/dsl/message"),
				"org.eclipse.emf.ecore");
		InputStream inputStream5 = GenerateModelsExecutor.class
				.getResourceAsStream("/models/message.ecore");
		try {
			resource5.load(inputStream5, new HashMap<>());
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		for (EObject o : resource5.getContents()) {
			if (o instanceof EPackage) {
				EPackage.Registry.INSTANCE.put(((EPackage) o).getNsURI(), o);
			}
		}
		
		Resource resource6 = resSet.createResource(URI.createURI("http://www.github.com/jrharbin-york/atlas-middleware/dsl/region"),
				"org.eclipse.emf.ecore");
		InputStream inputStream6 = GenerateModelsExecutor.class
				.getResourceAsStream("/models/region.ecore");
		try {
			resource6.load(inputStream6, new HashMap<>());
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		for (EObject o : resource6.getContents()) {
			if (o instanceof EPackage) {
				EPackage.Registry.INSTANCE.put(((EPackage) o).getNsURI(), o);
			}
		}
		//System.out.println(theFile.getRawLocation().toOSString());
	}

	private EmfModel createAndLoadAnEmfModel(String metamodelURI, String modelFile, String modelName, String readOnLoad,
			String storeOnDisposal) throws EolModelLoadingException {
		EmfModel theModel = new EmfModel();
		StringProperties properties = new StringProperties();
		properties.put(EmfModel.PROPERTY_METAMODEL_URI, metamodelURI);
		properties.put(EmfModel.PROPERTY_MODEL_FILE, modelFile);
		properties.put(EmfModel.PROPERTY_NAME, modelName);
		properties.put(EmfModel.PROPERTY_READONLOAD, readOnLoad);
		properties.put(EmfModel.PROPERTY_STOREONDISPOSAL, storeOnDisposal);
		//
		properties.put(EmfModel.PROPERTY_CACHED, "false");
		//
		theModel.load(properties, (IRelativePathResolver) null);
		return theModel;
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
		if (GenerateModelsExecutor.class.getResource(fileName)==null) {
			System.out.println("getresource returned null for fileName " + fileName);
		}
		
		java.net.URI binUri = GenerateModelsExecutor.class.getResource(fileName).toURI();
		return binUri.toString().contains("bin") ? new java.net.URI(binUri.toString().replaceAll("bin", "src")) : binUri;
	}
	
}
