package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotContainer {
    public final RobotHardware hardware;

    public RobotContainer(HardwareMap hwMap) {
        hardware = new RobotHardware(hwMap);
    }
}

