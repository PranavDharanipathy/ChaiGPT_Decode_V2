package org.firstinspires.ftc.teamcode.EnhancedFunctions_SELECTED;

import android.content.Context;
import android.content.SharedPreferences;
import android.content.SharedPreferences.Editor;

/// The XML files are stored on the Driver Hub.
public final class SimpleDataFile_XML {

    private final SharedPreferences sharedPreferences;
    private final Context context;

    public SimpleDataFile_XML(String FILE_NAME, Context context) { //XML file is created with name of FILE_NAME

        this.context = context;
        sharedPreferences = this.context.getSharedPreferences(FILE_NAME, Context.MODE_PRIVATE);
    }

    public void saveData(String key, int value) { //save to XML file

        Editor editor = sharedPreferences.edit();
        editor.putInt(key, value);
        editor.apply();
    }

    public int loadData(String key, int defaultValue) { //load from XML file
        return sharedPreferences.getInt(key, defaultValue);
    }

    public void remove(String key) { // deletes the key-value pair for the given key in the XML
        sharedPreferences.edit().remove(key).apply();
    }

    public void clearAll() { // clears all data from XML file
        sharedPreferences.edit().clear().apply();
    }

    public void deleteDataFile(String FILE_NAME) { // deletes the XML file
        context.deleteSharedPreferences(FILE_NAME);
    }

}