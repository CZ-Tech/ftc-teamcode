package org.firstinspires.ftc.teamcode.common.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class LedColor {
    public static Gamepad.LedEffect RGBEffect = new Gamepad.LedEffect.Builder()
            .addStep(1, 0, 0, 500)
            .addStep(0, 1, 0, 500)
            .addStep(0, 0, 1, 500)
            .build();

    public static Gamepad.LedEffect breakRed = new Gamepad.LedEffect.Builder()
            .addStep(1, 0, 0, 100)
            .addStep(0, 0, 0, 100)
            .setRepeating(true)
            .build();

    public static Gamepad.LedEffect pinkEffect = new Gamepad.LedEffect.Builder()
            .addStep(255,0,246,Gamepad.LED_DURATION_CONTINUOUS)
            .build();
    public static Gamepad.LedEffect stopLED = new Gamepad.LedEffect.Builder()
            .addStep(0, 0, 0, Gamepad.LED_DURATION_CONTINUOUS)
            .build();
}
