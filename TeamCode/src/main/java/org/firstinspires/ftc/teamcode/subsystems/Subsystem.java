package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMain;

public abstract class Subsystem {

    public abstract void subsystemInit(HardwareMap hardwareMap);
    public abstract void loop(Telemetry telemetry);
    public abstract void stop();

}
