package org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;
import org.opencv.core.Scalar;

public class led {
    private static RevBlinkinLedDriver l;
    public static void reset(){

    }
    public static void setColor(){
        l.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }
    public static class Color {
        public static final Scalar example = new Scalar(0, 0, 0);
    }
}
