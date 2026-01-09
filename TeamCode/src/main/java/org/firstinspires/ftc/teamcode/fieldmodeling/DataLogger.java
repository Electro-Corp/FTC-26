package org.firstinspires.ftc.teamcode.fieldmodeling;

import com.google.gson.JsonParser;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.Reader;
import java.io.Writer;
import java.util.ArrayList;

public class DataLogger {

    Writer logOut;

    private String fileName;

    private static final String readName = "/sdcard/LogParams-FINAL.txt";

    public DataLogger(){
        fileName = "/sdcard/LogParams-" + System.currentTimeMillis() + ".txt";
    }

    public void write(FieldDataPoints points){
        try{
            logOut = new FileWriter(fileName);
        } catch (Exception e){
            throw new RuntimeException(e);
        }

        try {
            logOut.write(points.toJSON().toString());
            logOut.close();
        } catch (Exception e){
            throw new RuntimeException(e);
        }
    }

    public static FieldDataPoints read(){
        Reader reader = null;
        try {
            reader = new FileReader(readName);
        } catch(Exception e){
            new RuntimeException(e);
        }

        return new FieldDataPoints(new JsonParser().parse(reader).getAsJsonObject());
    }

}
