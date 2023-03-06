package org.firstinspires.ftc.teamcode.developerTools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;

@TeleOp(group = "developer tools")
public class SensorTester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DistanceSensor distanceFromConeSensor = hardwareMap.get(DistanceSensor.class, "grabberDistanceToConeSensor");

        DigitalChannel isArmOut           = hardwareMap.get(DigitalChannel.class, "armIsOutSensor"    );
        DigitalChannel isElevatorUpSensor = hardwareMap.get(DigitalChannel.class, "elevatorIsUpSensor");
        DigitalChannel isGrabberOutSensor = hardwareMap.get(DigitalChannel.class, "grabberIsOutSensor");

        isArmOut          .setMode(DigitalChannel.Mode.INPUT);
        isElevatorUpSensor.setMode(DigitalChannel.Mode.INPUT);
        isGrabberOutSensor.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();
        if (isStopRequested()) return;
        resetRuntime();

        while (opModeIsActive()){
            telemetry.addData("distanceFromConeSensor", distanceFromConeSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("isArmOutSensor"        , isArmOut.getState());
            telemetry.addData("isElevatorUpSensor"    , isElevatorUpSensor.getState());
            telemetry.addData("isGrabberOutSensor"    , isGrabberOutSensor.getState());
            telemetry.update();
        }
    }
}
