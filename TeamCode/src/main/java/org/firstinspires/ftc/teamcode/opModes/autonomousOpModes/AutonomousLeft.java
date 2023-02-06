package org.firstinspires.ftc.teamcode.opModes.autonomousOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.myDependencies.RobotSystem;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.puffer;

@Autonomous
public class AutonomousLeft extends LinearOpMode {
    @Override
    public void runOpMode(){
        RobotSystem.regularAuto.initializeAll(hardwareMap, telemetry, gamepad1, gamepad2);

        waitForStart();
        RobotSystem.regularAuto.getLatestDetection();
        RobotSystem.regularAuto.closeCamera();

        new Thread(() -> {
            while (opModeIsActive()){}
            RobotSystem.regularAuto.terminate();
        }).start();

        RobotSystem.startAllAutonomousControllers();
        try {
            RobotSystem.regularAuto.follow(RobotSystem.regularAuto.trajectories.startToScore);
            puffer.grab();
            puffer.goToMid();
            RobotSystem.await(RobotSystem.regularAuto::isStationary);
            RobotSystem.regularAuto.cycle(5);

            switch (RobotSystem.regularAuto.detection){
                case (1):{
                    RobotSystem.regularAuto.follow(RobotSystem.regularAuto.trajectories.scoreToPark1);
                    break;
                }
                case (2):{
                    RobotSystem.regularAuto.follow(RobotSystem.regularAuto.trajectories.scoreToPark2);
                    break;
                }
                case (3):{
                    RobotSystem.regularAuto.follow(RobotSystem.regularAuto.trajectories.scoreToPark3);
                    break;
                }
            }
        } catch (InterruptedException e){}

        while (opModeIsActive() && !RobotSystem.isStopRequested){}

        RobotSystem.regularAuto.terminate();
    }
}
