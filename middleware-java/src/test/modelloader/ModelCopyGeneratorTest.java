package test.modelloader;

import java.io.FileNotFoundException;
import java.lang.reflect.InvocationTargetException;
import ciexperiment.systematic.GenerateModelsExecutor;

public class ModelCopyGeneratorTest {
	public static void main(String[] args) throws FileNotFoundException, InterruptedException {
		GenerateModelsExecutor modelLoader = new GenerateModelsExecutor();
		try {
			modelLoader.transformModel("test-experiment-models/mission-3robot.model");
			System.out.println("Models all transformed successfully");
		} catch (InvocationTargetException | InterruptedException e) {
			e.printStackTrace();
		}
	}
}
