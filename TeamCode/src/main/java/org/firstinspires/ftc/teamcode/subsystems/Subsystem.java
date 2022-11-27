package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotMain;

public abstract class Subsystem {

    public abstract void subsystemInit(HardwareMap hardwareMap);
    public abstract void loop();
    public abstract void stop();

}
