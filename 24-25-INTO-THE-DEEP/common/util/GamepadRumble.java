package org.firstinspires.ftc.teamcode.common.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadRumble {
    public static Gamepad.RumbleEffect fullPowerRumble = new Gamepad.RumbleEffect.Builder()
            .addStep(1, 1, 500)
            .build();

    public static Gamepad.RumbleEffect halfPowerRumble = new Gamepad.RumbleEffect.Builder()
            .addStep(0.5, 0.5, 500)
            .build();

    public static Gamepad.RumbleEffect halfContinousPowerRumble = new Gamepad.RumbleEffect.Builder()
            .addStep(0.5, 0.5, Gamepad.RUMBLE_DURATION_CONTINUOUS)
            .build();
}
