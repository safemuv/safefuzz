package utils;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.Writer;

public class MyInputStreamSink implements Runnable {
    private InputStream m_in;
    private String m_streamName;

    MyInputStreamSink( InputStream in, String streamName ) {
        m_in = in;
        m_streamName = streamName;
    }

    public void run() {
        BufferedReader reader = null;
        Writer writer = null;
        try {
            reader = new BufferedReader( new InputStreamReader( m_in ) );

            for ( String line = null; ((line = reader.readLine()) != null); ) {
            }
        } catch (IOException e) {
            System.out.println("Unexpected I/O exception reading from process.");
        }
        finally {
            try {
                if ( null != reader ) reader.close();
            }
            catch ( java.io.IOException e ) {
                System.out.println("Unexpected I/O exception closing a stream.");
            }
        }
    }
}