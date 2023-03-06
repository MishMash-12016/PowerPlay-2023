package org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;

public class arm {
    // region SERVOS
    private static Servo rightServo;
    private static Servo leftServo ;
    // endregion

    // region SENSOR
    private static DigitalChannel isArmOut;
    // endregion

    // region CONSTANTS
    public static final double outPosition = 0.04;
    public static final double inPosition  = 0.55 ;
    // endregion

    // region INITIALIZATION
    public static void initialize(){
        // region SERVOS
        rightServo = RobotSystem.hardwareMap.servo.get("armRight");
        leftServo  = RobotSystem.hardwareMap.servo.get("armLeft") ;

        leftServo.setDirection(Servo.Direction.REVERSE);

        goToRelativePosition(0);
        // endregion

        // region SENSOR
        isArmOut = RobotSystem.hardwareMap.get(DigitalChannel.class, "armIsOutSensor");
        isArmOut.setMode(DigitalChannel.Mode.INPUT);
        // endregion
    }
    // endregion

    // region FUNCTIONALITY
    public static void goToRelativePosition(double position){
        setPosition(inPosition + Math.abs(outPosition - inPosition) * position);
    }

    public static boolean isOut(){
        return isArmOut.getState();
    }
    // endregion

    // region PRIVATE FUNCTIONALITY
    private static void setPosition(double position){
        rightServo.setPosition(position);
        leftServo .setPosition(position);
    }
    // endregion
}
