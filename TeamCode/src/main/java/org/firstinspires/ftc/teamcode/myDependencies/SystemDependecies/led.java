package org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;

public class led {
    private static RevBlinkinLedDriver l;
    public static void initialize(){
        l = RobotSystem.hardwareMap.get(RevBlinkinLedDriver.class, "leds");
        l.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
    }

    public static void setGreen(){
        l.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }
    public static void setRed(){
        l.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    }
}
