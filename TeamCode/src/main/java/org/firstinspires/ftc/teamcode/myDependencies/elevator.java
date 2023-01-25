package org.firstinspires.ftc.teamcode.myDependencies;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class elevator {
    // region MOTORS
    private static DcMotorEx motorRight;
    private static DcMotorEx motorLeft ;
    // endregion

    // region CONSTANTS
    public static final int highPosition   = 19500;
    public static final int middlePosition = 12000;
    public static final int lowPosition    = 5000 ;
    public static final int bottomPosition = 0    ;
    // endregion

    // region VARIABLES
    public static double position;

    private static double power;
    private static boolean isControllerActive;
    // endregion

    // region INITIALIZATION
    public static void initialize() {
        motorLeft = (DcMotorEx) System.hardwareMap.dcMotor.get("elevatorLeft");
        motorRight = (DcMotorEx) System.hardwareMap.dcMotor.get("elevatorRight");

        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        reset();
    }

    public static void reset(){
        position = 0;
        power    = 0;
        isControllerActive = false;
    }
    // endregion

    // region FUNCTIONALITY
    Thread controller = new Thread(() -> {
        reset();
        isControllerActive = true;
        while (isControllerActive && !System.isStopRequested){
            power = calculatePower();
            if (power >  1) power =  1;
            if (power < -1) power = -1;

            // make the elevator weaker when coming down to compensate for gravity
            if (power < 0) power /= 2;

            // set the elevator power into the elevator motors
            setPower(power);
        }
    });
    // endregion

    // region PRIVATE FUNCTIONALITY
    private static void setPower(double power){
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }
    private static double calculatePower(){
        return (position - motorLeft.getCurrentPosition()) / 2300.0;
    }
    // endregion
}
