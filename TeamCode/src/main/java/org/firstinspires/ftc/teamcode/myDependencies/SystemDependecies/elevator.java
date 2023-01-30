package org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;

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

    public static final int marginOfError = 400;
    // endregion

    // region SENSOR
    public static DigitalChannel isUpSensor;
    // endregion

    // region VARIABLES
    public static double wantedPosition;

    private static double motorPower;
    // endregion

    // region INITIALIZATION
    public static void initialize() {
        // region MOTORS
        motorLeft = (DcMotorEx) RobotSystem.hardwareMap.dcMotor.get("elevatorLeft");
        motorRight = (DcMotorEx) RobotSystem.hardwareMap.dcMotor.get("elevatorRight");

        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // endregion

        // region SENSOR
        isUpSensor = RobotSystem.hardwareMap.get(DigitalChannel.class, "elevatorIsUpSensor");
        isUpSensor.setMode(DigitalChannel.Mode.INPUT);
        // endregion

        elevator.reset();
    }

    public static void reset(){
        motorPower = 0;
        wantedPosition = 0;
    }
    // endregion

    // region FUNCTIONALITY
    public static Thread controller = new Thread(() -> {
        elevator.reset();
        while (!RobotSystem.isStopRequested && !elevator.controller.isInterrupted()){
            motorPower = calculatePower();

            if (motorPower >  1) motorPower =  1;
            if (motorPower < -1) motorPower = -1;

            // make the elevator weaker when coming down to compensate for gravity
            if (motorPower < 0) motorPower /= 2;

            // set the elevator power into the elevator motors
            setMotorPower(motorPower);
        }
    });
    public static boolean isUp(){
        return isUpSensor.getState();
    }
    public static boolean reachedWantedPosition(){
        return marginOfError < Math.abs(wantedPosition - getCurrentPosition());
    }
    // endregion

    // region PRIVATE FUNCTIONALITY
    private static void setMotorPower(double motorPower){
        motorLeft.setPower(motorPower);
        motorRight.setPower(motorPower);
    }
    private static double calculatePower(){
        return (wantedPosition - motorLeft.getCurrentPosition()) / 2300.0;
    }
    private static double getCurrentPosition() {
        return motorLeft.getCurrentPosition();
    }
    // endregion

}
