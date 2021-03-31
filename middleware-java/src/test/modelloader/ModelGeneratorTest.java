package test.modelloader;

import java.io.FileNotFoundException;
import java.lang.reflect.InvocationTargetException;

import org.eclipse.epsilon.eol.exceptions.models.EolModelLoadingException;

public class ModelGeneratorTest {
	public static void main(String[] args) throws FileNotFoundException, InterruptedException {
		
		GenerateModelsLoaderTest modelLoader = new GenerateModelsLoaderTest();
		try {
			modelLoader.run("test-experiment-models/mission-empty.model");
			modelLoader.run("test-experiment-models/mission-working-goal-with-no-rect.model");
			modelLoader.run("test-experiment-models/mission-goal-with-rect.model");
			modelLoader.run("test-experiment-models/mission-test.model");
			modelLoader.run("test-experiment-models/mission-1robot-empty.model");		
			modelLoader.run("test-experiment-models/mission-3robot.model");
			System.out.println("Models all loaded successfully");
		} catch (InvocationTargetException | InterruptedException e) {
			e.printStackTrace();
		} catch (EolModelLoadingException e) {
			e.printStackTrace();
		}
	}
}