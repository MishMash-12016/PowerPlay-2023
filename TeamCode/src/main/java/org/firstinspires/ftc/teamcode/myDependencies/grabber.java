package org.firstinspires.ftc.teamcode.myDependencies;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class grabber {
    // region SERVOS
    private static Servo grabberServo ;
    private static Servo rightServo   ;
    private static Servo leftServo    ;
    // endregion

    // region SENSORS
    private static DistanceSensor distanceFromConeSensor;
    private static DigitalChannel isOutSensor;
    // endregion

    // region CONSTANTS
    private static final double middlePosition = 0.26;
    private static final double inPosition     = 0.09;
    //                                             low -  -  -  -  -  -  - high
    private static final double[] pilePositions = {0.76, 0.72, 0.69, 0.65, 0.61};


    private static final double grabPosition = 0.61;
    private static final double releasePosition = 0.36;

    private static final double coneDistanceCatchTrigger = 8.5;
    // endregion

    // region INITIALIZATION
    public static void initialize(){
        // region SERVOS
        grabberServo = System.hardwareMap.servo.get("grabber");

        release();

        rightServo = System.hardwareMap.servo.get("placerRight");
        leftServo  = System.hardwareMap.servo.get("placerLeft" );

        rightServo.setDirection(Servo.Direction.REVERSE);

        setPosition(inPosition);
        // endregion

        // region SENSOR
        isOutSensor = System.hardwareMap.get(DigitalChannel.class, "grabberOutSensor");
        isOutSensor.setMode(DigitalChannel.Mode.INPUT);

        distanceFromConeSensor = System.hardwareMap.get(DistanceSensor.class, "grabberDistanceSensor");
        // endregion
    }
    // endregion

    // region FUNCTIONALITY
    public static void release() { grabberServo.setPosition(releasePosition); }
    public static void grab()    { grabberServo.setPosition(grabPosition   ); }

    public static void goToCone(int coneHeight){ setPosition(pilePositions[coneHeight]); }
    public static void goToMid()            { setPosition(middlePosition); }
    public static void goToIn()             { setPosition(inPosition    ); }

    public static boolean coneIsInRange(){
        return coneDistanceCatchTrigger > distanceFromConeSensor.getDistance(DistanceUnit.CM);
    }
    public static boolean isOut() {
        return isOutSensor.getState();
    }

    public static double distanceReading(){
        return distanceFromConeSensor.getDistance(DistanceUnit.CM);
    }
    // endregion

    // region PRIVATE FUNCTIONALITY
    private static void setPosition(double position){
        rightServo.setPosition(position);
        leftServo .setPosition(position);
    }
    // endregion
}
