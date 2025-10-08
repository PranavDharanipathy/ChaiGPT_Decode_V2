package org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileWriter;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.Map;

public class DataSaver_JSON {

    private final Gson gson = new GsonBuilder().setPrettyPrinting().create();

    private BufferedWriter dataWriter;
    private BufferedReader dataReader;

    private File internallyStoredFile;

    /// @param fileName - must be "whatever file name".jsonl
    public DataSaver_JSON(String fileName) {

        internallyStoredFile = AppUtil.getInstance().getSettingsFile(fileName);

        try {
            dataWriter = new BufferedWriter(new FileWriter(internallyStoredFile, true));
            dataReader = new BufferedReader(new FileReader(internallyStoredFile));

        }
        catch (IOException ignored) {}
    }

    /// Adds to JSON
    public synchronized void addData(String header, Object data) {

        Map<String, Object> entry = new LinkedHashMap<>();
        entry.put(header, data);

        try {
            dataWriter.write(gson.toJson(entry));
            dataWriter.newLine();
        }
        catch (IOException ignored) {}
    }

    /// Reads data from JSON
    public String[] retrieveData() {

        String[] data = gson.fromJson(dataReader, String[].class);

        //want a list/array type value
        ArrayList<String> allData = new ArrayList<>(Arrays.asList(data));
        return allData.toArray(new String[0]);
    }


    public void close() {

        try {

            dataWriter.close();
            dataReader.close();
        }
        catch (IOException ignored) {}
    }

}
