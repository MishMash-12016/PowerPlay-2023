package org.firstinspires.ftc.teamcode.opModes.autonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;

@Autonomous
public class newAutonomous extends LinearOpMode {

    public void runOpMode(){
        RobotSystem.initializeAll(hardwareMap, telemetry, gamepad1, gamepad2);
        RobotSystem.auto.initialize();

        waitForStart();

        try {

            RobotSystem.auto.follow(RobotSystem.auto.trajectories.startToScore);
            RobotSystem.auto.score();
            RobotSystem.auto.cycle(5);

            RobotSystem.auto.follow(RobotSystem.auto.trajectories.scoreToPark1);

        }catch (InterruptedException e){
            RobotSystem.auto.terminate();
        }
    }
}
