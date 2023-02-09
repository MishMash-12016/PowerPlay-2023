package org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;

public class puffer {
    // region SERVOS
    private static Servo pufferServo;
    private static Servo rightServo ;
    private static Servo leftServo  ;
    // endregion

    // region CONSTANTS
    private static final double inPosition  = 0.16;
    private static final double midPosition = 0.5 ;
    private static final double outPosition = 0.95;

    private static final double autonomousOutPosition = 0.88;
    private static final double autonomousInPosition = 0.16;

    private static final double grabPosition    = 0.3;
    private static final double releasePosition = 0.2;
    // endregion

    // region INITIALIZATION
    public static void initialize(){
        pufferServo = RobotSystem.hardwareMap.servo.get("puffer");

        release();

        rightServo = RobotSystem.hardwareMap.servo.get("pufferRight");
        leftServo  = RobotSystem.hardwareMap.servo.get("pufferLeft" );

        rightServo.setDirection(Servo.Direction.REVERSE);

        goToIn();
    }
    // endregion

    // region FUNCTIONALITY
    public static void grab(){
        led.green();
        pufferServo.setPosition(grabPosition);
    }
    public static void release(){
        led.red();
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

    public static void autonomousGoToOut(){
        setPosition(autonomousOutPosition);
    }
    public static void autonomousGoToIn(){
        setPosition(autonomousInPosition);
    }

    public static boolean isGrabbing() {
        return pufferServo.getPosition() == grabPosition;
    }
    // endregion

    // region PRIVATE FUNCTIONALITY
    private static void setPosition(double position){
        rightServo.setPosition(position);
        leftServo .setPosition(position);
    }
    // endregion
}
