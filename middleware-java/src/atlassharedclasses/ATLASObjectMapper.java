package atlassharedclasses;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

public class ATLASObjectMapper {
	public enum FormatErrorType {
		JSON_PROCESSING_EXCEPTION,
		INVALID_CLASS,
		REGEX_MATCH_FAILED
	}
	
	public class ATLASFormatError extends Exception {
		private static final long serialVersionUID = 1L;
		private FormatErrorType et;
		ATLASFormatError(FormatErrorType et) {
			this.et = et;
		}
	}
	
	private Pattern msgScanner = Pattern.compile("([^,]+),(.+)");
	private ObjectMapper objMapper = new ObjectMapper();
	
	public String serialise(Object msg) throws JsonProcessingException {
		String className = msg.getClass().getName();
		return className + "," + objMapper.writeValueAsString(msg);
	}
	
	// TODO: need some way to tell what the type is?
	public ATLASSharedResult deserialise(String msgText) throws ATLASFormatError {
		Matcher m = msgScanner.matcher(msgText);
		if (m.find()) {
			String className = m.group(1);
			String objText = m.group(2);
			try {
				Class<?> msgClass = Class.forName(className);
				Object msg = objMapper.readValue(objText, msgClass);
				return new ATLASSharedResult(msg, msgClass);
			} catch (JsonProcessingException e) {
				e.printStackTrace();
				throw new ATLASFormatError(FormatErrorType.JSON_PROCESSING_EXCEPTION);
			} catch (ClassNotFoundException e) {
				e.printStackTrace();
				throw new ATLASFormatError(FormatErrorType.INVALID_CLASS);
			}
		} else {
			throw new ATLASFormatError(FormatErrorType.REGEX_MATCH_FAILED);
		}
	}
}
