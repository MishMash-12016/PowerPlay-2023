package org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.myDependencies.System;

public class puffer {
    // region SERVOS
    private static Servo pufferServo;
    private static Servo rightServo ;
    private static Servo leftServo  ;
    // endregion

    // region CONSTANTS
    private static final double inPosition  = 0   ;
    private static final double midPosition = 0.2 ;
    private static final double outPosition = 0.75;

    private static final double grabPosition    = 0.14;
    private static final double releasePosition = 0   ;
    // endregion

    // region INITIALIZATION
    public static void initialize(){
        pufferServo = System.hardwareMap.servo.get("puffer");

        release();

        rightServo = System.hardwareMap.servo.get("placerRight");
        leftServo  = System.hardwareMap.servo.get("placerLeft" );

        rightServo.setDirection(Servo.Direction.REVERSE);
    }
    // endregion

    // region FUNCTIONALITY
    public static void grab(){
        pufferServo.setPosition(grabPosition);
    }
    public static void release(){
        pufferServo.setPosition(releasePosition);
    }

    public static void goToOut(){
        setPosition(outPosition);
    }
    public static void goToMid(){
        setPosition(midPosition);
    }
    public static void goToIn(){
        setPosition(inPosition);
    }
    // endregion

    // region PRIVATE FUNCTIONALITY
    private static void setPosition(double position){
        rightServo.setPosition(position);
        leftServo .setPosition(position);
    }
    // endregion
}
