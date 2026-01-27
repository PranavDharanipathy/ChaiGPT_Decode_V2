package org.firstinspires.ftc.teamcode.util.TelemetryUtils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Arrays;

public class TelemetryO {

    private Telemetry telemetry;

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public TelemetryO(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public TelemetryO addDash() {
        return new TelemetryO(new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry()));
    }

    private ArrayList<TelemetryMode> enabledModes = new ArrayList<>(Arrays.asList(TelemetryMode.getAll()));

    public void setTelemetryModes(TelemetryMode... modes) {

        enabledModes.clear();

        for (TelemetryMode mode : modes) {
            if (!enabledModes.contains(mode)) enabledModes.add(mode);
        }
    }

    public TelemetryMode[] getTelemetryModesAsArray() {
        return enabledModes.toArray(new TelemetryMode[0]);
    }

    public ArrayList<TelemetryMode> getTelemetryModes() {
        return enabledModes;
    }

    public void addData(TelemetryMode mode, String caption, String format, Object... args) {
        if (enabledModes.contains(mode)) telemetry.addData(caption, format, args);
    }
    public void addData(String caption, String format, Object... args) {
        telemetry.addData(caption, format, args);
    }

    public void addData(TelemetryMode mode, String caption, Object value) {
        if (enabledModes.contains(mode)) telemetry.addData(caption, value);
    }
    public void addData(String caption, Object value) {
        telemetry.addData(caption, value);
    }

    public void clear() {
        telemetry.clear();
    }

    public void clearAll() {
        telemetry.clearAll();
    }

    public void addAction(TelemetryMode mode, Runnable action) {
        if (enabledModes.contains(mode)) telemetry.addAction(action);
    }

    public void speak(TelemetryMode mode, String text) {
        if (enabledModes.contains(mode)) telemetry.speak(text);
    }
    public void speak(String text) {
        telemetry.speak(text);
    }

    public void speak(TelemetryMode mode, String text, String languageCode, String countryCode) {
        if (enabledModes.contains(mode)) telemetry.speak(text, languageCode, countryCode);
    }
    public void speak(String text, String languageCode, String countryCode) {
        telemetry.speak(text, languageCode, countryCode);
    }

    /// This method doesn't care about mode
    public void addLine() {
        telemetry.addLine();
    }

    public void addLine(TelemetryMode mode, String lineCaption) {
        if (enabledModes.contains(mode)) telemetry.addLine(lineCaption);
    }
    public void addLine(String lineCaption) {
        telemetry.addLine(lineCaption);
    }

    public void update() {
        telemetry.update();
    }

}
