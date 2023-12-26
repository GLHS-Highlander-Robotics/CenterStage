package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class SensorColor {
    private NormalizedColorSensor colorSensor;
    private NormalizedRGBA colors;
    private float gain = 2;
    final float[] hsvValues = new float[3];

    public SensorColor(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        colors = colorSensor.getNormalizedColors();
        colorSensor.setGain(gain);
    }

    public void update() {
        colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
    }

    public float getHue() {
        return 0;
    }

    public float getSaturation() {
        return 0;
    }

    public float getValue() {
        return 0;
    }

    public float getRed() {
        return 0;
    }

    public float getBlue() {
        return 0;
    }

    public float getGreen() {
        return 0;
    }

    public float getAlpha() {
        return 0;
    }

    public float getGain() {
        return 0;
    }
}
