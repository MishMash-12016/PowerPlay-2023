package org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;

public class led {
    private static RevBlinkinLedDriver l;
    public static void initialize(){
        l = RobotSystem.hardwareMap.get(RevBlinkinLedDriver.class, "leds");
        l.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
    }

    public static void green(){
        l.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }
    public static void red(){
        l.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
    }
    public static void white(){
        l.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }
    public static void blue(){
        l.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }
    public static void black(){
        l.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }
}
