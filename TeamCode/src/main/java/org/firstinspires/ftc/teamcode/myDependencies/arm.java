package org.firstinspires.ftc.teamcode.myDependencies;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class arm {
    // region SERVOS
    private static Servo rightServo;
    private static Servo leftServo ;
    // endregion

    // region SENSOR
    private static DigitalChannel isArmOut;
    // endregion

    // region CONSTANTS
    public static final double outPosition = 0.89;
    public static final double inPosition  = 0.5 ;
    // endregion

    // region INITIALIZATION
    public static void initialize(){
        // region SERVOS
        rightServo = System.hardwareMap.servo.get("armRight");
        leftServo  = System.hardwareMap.servo.get("armLeft") ;

        leftServo.setDirection(Servo.Direction.REVERSE);
        // endregion

        // region SENSOR
        isArmOut = System.hardwareMap.get(DigitalChannel.class, "armSensor");
        isArmOut.setMode(DigitalChannel.Mode.INPUT);
        // endregion
    }
    // endregion

    // region FUNCTIONALITY
    public static void setPosition(double position){
        rightServo.setPosition(position);
        leftServo .setPosition(position);
    }
    public static boolean isIn(){
        return !isArmOut.getState();
    }
    // endregion
}
