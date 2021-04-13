package ciexperiment.systematic.backup;

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
import java.util.Optional;
import java.util.Random;
import java.util.UUID;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.EPackage;
import org.eclipse.emf.ecore.EStructuralFeature;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.resource.ResourceSet;
import org.eclipse.emf.ecore.resource.impl.ResourceSetImpl;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.emf.ecore.xmi.impl.XMIResourceFactoryImpl;
import org.eclipse.epsilon.common.util.StringProperties;
import org.eclipse.epsilon.egl.EglFileGeneratingTemplate;
import org.eclipse.epsilon.egl.EglFileGeneratingTemplateFactory;
import org.eclipse.epsilon.egl.EglTemplateFactoryModuleAdapter;
import org.eclipse.epsilon.egl.exceptions.EglRuntimeException;
import org.eclipse.epsilon.emc.emf.EmfModel;
import org.eclipse.epsilon.eol.exceptions.EolRuntimeException;
import org.eclipse.epsilon.eol.exceptions.models.EolModelElementTypeNotFoundException;
import org.eclipse.epsilon.eol.exceptions.models.EolModelLoadingException;
import org.eclipse.epsilon.eol.models.IRelativePathResolver;

public class GenerateModelsExecutor {
	private final String METAMODEL_FILE = "ecore-metamodels-combined/atlas.ecore";
	private final boolean DEBUG_MODEL_TRANSFORMS = true;
	Random rng;
	
	public GenerateModelsExecutor() {
		rng = new Random();
	}

	private void removeRobot(EmfModel targetModel, String robotName) throws EolModelElementTypeNotFoundException {
		Collection<EObject> elementsToBeRemoved = new ArrayList<EObject>();
		elementsToBeRemoved = targetModel.getAllOfKind("Robot");
		for (EObject eObj : elementsToBeRemoved) {
			EStructuralFeature nameAttr = eObj.eClass().getEStructuralFeature("name");
			if (eObj.eGet(nameAttr).equals(robotName)) {
				if (DEBUG_MODEL_TRANSFORMS) {
					System.out.println("Removing " + robotName);
				}
				EcoreUtil.delete(eObj, true);
				break;
			}
		}
	}
	
	/** Returns the string giving the name of the transformed model */
	public Optional<String> transformModel(String modelFilePath) throws InvocationTargetException, InterruptedException {
		try {
			EmfModel sourceModel = this.loadModel(METAMODEL_FILE, modelFilePath, "Source");
			
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
		    EmfModel targetModel = loadModel(METAMODEL_FILE, copied.toString(), "Target");
		    // Simple queries to get objects by type and their attributes
			Collection<EObject> mutableObjects = new ArrayList<EObject>();
			mutableObjects = sourceModel.getAllOfKind("Robot");
			for (EObject obj : mutableObjects) {
				EStructuralFeature nameAttr = obj.eClass().getEStructuralFeature("name");
				System.out.println("FOUND ROBOT NAME = " + obj.eGet(nameAttr));
			}
			
			// REMOVAL CODE START
			// Testing removing a randomly chosen robot from the model
			double test = rng.nextDouble();
			System.out.println("testval = " + test);
			if (test < 0.25) {
				removeRobot(targetModel, "frank");
			} else if (test < 0.5) {
				removeRobot(targetModel, "ella");
			} else if (test < 0.75) {
				removeRobot(targetModel, "gilda");
			} else {
				removeRobot(targetModel, "henry");
			}
			// REMOVAL CODE END
			
			targetModel.dispose();
			sourceModel.close();
			targetModel.close();
			
			return Optional.of(copied.toString());			
		} catch (EolModelLoadingException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (EolModelElementTypeNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (EolRuntimeException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		return Optional.empty();
	}
	
	private EmfModel loadModel(String metaModelFilePath, String modelFilePath, String modelName) throws EolModelLoadingException {
	    EmfModel model =  new EmfModel();
		ArrayList<String> metamodelFilesTarget = new ArrayList<String>();
		metamodelFilesTarget.add(new File(metaModelFilePath).getAbsolutePath());
		model.setMetamodelFiles(metamodelFilesTarget);
		model.setName("Target");
		model.setModelFile(modelFilePath);
		model.setStoredOnDisposal(true);
		model.load();
		return model;
	}
	
	public void executeEGL(String sourceModelFile, String eglOutput) throws EglRuntimeException, URISyntaxException, IOException, EolModelLoadingException {
		EmfModel sourceModelForEGL = loadModel(METAMODEL_FILE, sourceModelFile, "Target");
		//egl factory and module
		EglFileGeneratingTemplateFactory factory = new EglFileGeneratingTemplateFactory();
		EglTemplateFactoryModuleAdapter eglModule = new EglTemplateFactoryModuleAdapter(factory);
		eglModule.getContext().getModelRepository().addModel(sourceModelForEGL);

		//String rawLocation = "ecore-metamodels/genmission.egl";
		//java.net.URI EglFile = new java.net.URI(new File(rawLocation).getAbsolutePath());
		java.net.URI EglFile = new java.net.URI("file:///home/atlas/atlas/atlas-middleware/middleware-java/ecore-metamodels/genmission.egl");
		System.out.println(EglFile);
		
		EglFileGeneratingTemplate template = (EglFileGeneratingTemplate)factory.load(EglFile);
		template.process();
				
		// Set the target file, ie. where the results will be generated to.
		// TODO: replace the path for the Java generated DSL here
		File target = new File(eglOutput);
		target.createNewFile();
		template.generate(target.toURI().toString());
	}
	
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
