package test.modelloader;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.net.URISyntaxException;
import java.util.Optional;

import org.eclipse.epsilon.egl.exceptions.EglRuntimeException;
import org.eclipse.epsilon.eol.exceptions.models.EolModelLoadingException;

import ciexperiment.systematic.backup.GenerateModelsExecutor;

public class ModelCopyGeneratorTest {
	public static void main(String[] args) throws FileNotFoundException, InterruptedException {
		GenerateModelsExecutor modelLoader = new GenerateModelsExecutor();
		try {
			modelLoader.transformModel("test-experiment-models/mission-empty-robot.model");
			Optional<String> file_opt = modelLoader.transformModel("test-experiment-models/mission-3robot.model");
			
			if (file_opt.isPresent()) {
				String file = file_opt.get();
				modelLoader.executeEGL(file, "/tmp/testout.java");
			}
			
			System.out.println("Models all transformed successfully");
		} catch (InvocationTargetException | InterruptedException e) {
			e.printStackTrace();
		} catch (EolModelLoadingException e) {
			e.printStackTrace();
		} catch (EglRuntimeException e) {
			e.printStackTrace();
		} catch (URISyntaxException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}
