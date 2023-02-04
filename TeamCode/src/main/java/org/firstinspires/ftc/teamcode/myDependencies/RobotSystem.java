package org.firstinspires.ftc.teamcode.myDependencies;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.myDependencies.SystemDependecies.*;
import org.firstinspires.ftc.teamcode.myDependencies.oldFiles.PipeLine;
import org.firstinspires.ftc.teamcode.roadRunnerDependencies.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunnerDependencies.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.Callable;

public class RobotSystem {
    // region SYSTEM CONSTANTS
    public static final InterruptedException hardStopRequest = new InterruptedException("stop everything !");
    public static final InterruptedException softStopRequest = new InterruptedException("stop running safely");

    public static Gamepad gamepad1;
    public static Gamepad gamepad2;
    public static Telemetry telemetry;
    public static HardwareMap hardwareMap;
    // endregion

    // region SYSTEM VARIABLES
    public static boolean isStopRequested;
    public static int currentConeHeight;
    // endregion

    // region INITIALIZATION
    public static void initializeAll(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2){
        RobotSystem.initialize(hardwareMap, telemetry, gamepad1, gamepad2);

        arm.initialize();
        puffer.initialize();
        grabber.initialize();
        elevator.initialize();
        driveTrain.initialize();
    }
    public static void resetAll(){
        RobotSystem.reset();

        led.reset();
        elevator.reset();
        driveTrain.reset();
    }

    public static void initialize(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2){
        RobotSystem.reset();

        RobotSystem.hardwareMap = hardwareMap;
        RobotSystem.telemetry   = telemetry  ;
        RobotSystem.gamepad1    = gamepad1   ;
        RobotSystem.gamepad2    = gamepad2   ;
    }
    public static void reset(){
        isStopRequested = false;
        currentConeHeight = 0;
    }
    // endregion

    // region SYSTEM FUNCTIONALITY
    public static void startAllTeleOpControllers(){
        RobotSystem.cycleController.start();
        driveTrain.controller.start();
        elevator.controller.start();
    }
    public static void startAllAutonomousControllers(){
        elevator.controller.start();

    }

    public static void terminate(){
        resetAll();
        isStopRequested = true;

        telemetry.update();
    }
    // endregion

    // region GENERAL FUNCTIONALITY
    private static final ElapsedTime timePassed = new ElapsedTime();
    public static void sleep(int wantedSleepTime) throws InterruptedException{
        timePassed.reset();
        while(wantedSleepTime > timePassed.milliseconds()){
            if (isStopRequested) {
                throw hardStopRequest;
            }
            Thread.sleep(20);
        }
    }
    public static void await(Callable<Boolean> requirement) throws InterruptedException{
        try{
            while (!requirement.call() && !isStopRequested) {
            }
        }catch (Exception e){
            throw softStopRequest;
        }
        if (isStopRequested) {
            throw hardStopRequest;
        }
    }

    public static class FirstPress{
        public static boolean a = true;
        public static boolean A(){
            if (gamepad1.a){
                if(a){
                    a = false;
                    return true;
                }
            } else a = true;
            return false;
        }

        public static boolean x = true;
        public static boolean X(){
            if (gamepad1.x){
                if(x){
                    x = false;
                    return true;
                }
            } else x = true;
            return false;
        }

        public static boolean y = true;
        public static boolean Y(){
            if (gamepad1.y){
                if(y){
                    y = false;
                    return true;
                }
            } else y = true;
            return false;
        }

        public static boolean b = true;
        public static boolean B(){
            if (gamepad1.b){
                if(b){
                    b = false;
                    return true;
                }
            } else b = true;
            return false;
        }
    }
    // endregion

    // region DRIVER CONTROLLED FUNCTIONS
    private static final Thread cycleController = new Thread(() -> {
        while (!isStopRequested && !RobotSystem.cycleController.isInterrupted()) {
            if (!manual.asyncCollect.isAlive() && (gamepad1.left_trigger > 0 || gamepad1.left_bumper)) {
                manual.collect();
            }

            if      (FirstPress.A()) { manual.score(elevator.highPosition); }
            else if (FirstPress.X()) { manual.score(elevator.middlePosition); }
            else if (FirstPress.Y()) { manual.score(elevator.lowPosition); }
            else if (FirstPress.B()) { manual.asyncScore.interrupt(); }
        }
    });

    public static class manual{
        public static void collect() {
            asyncCollect.start();
        }
        public static void score(int height) {
            elevator.setWantedPosition(height);
            if (!manual.asyncScore.isAlive()){
                asyncScore.start();
            }
        }

        public static final Thread asyncCollect = new Thread(() -> {
            try {
                driveTrain.slowMode();
                grabber.goToCone(currentConeHeight);
                grabber.release();
                sleep(200);
                while (!RobotSystem.manual.asyncCollect.isInterrupted() && !isStopRequested && (gamepad1.left_trigger > 0 || gamepad1.left_bumper)) {
                    arm.goToRelativePosition(gamepad1.left_trigger);

                    if (grabber.coneIsInRange()) {
                        if (!grabber.isGrabbing()) {
                            grabber.grab();
                        }
                    } else {
                        if (grabber.isGrabbing()) {
                            grabber.release();
                        }
                    }
                }

                driveTrain.fastMode();

                if (isStopRequested){
                    throw hardStopRequest;
                }

                if (RobotSystem.manual.asyncCollect.isInterrupted()){
                    throw softStopRequest;
                }

                grabber.goToMid();

                arm.goToRelativePosition(0);

                await(() -> !arm.isOut() && !elevator.isUp());

                grabber.goToIn();

                await(() -> !grabber.isOut());

                grabber.release();

                if (grabber.hasCone()) {
                    puffer.grab();
                    puffer.goToMid();
                }

            }catch (Exception e){
                if (e == softStopRequest){
                    arm.goToRelativePosition(0);
                    grabber.release();
                    grabber.goToIn();
                } else if (e != hardStopRequest){
                    telemetry.addData("stopped manual collect completely", e);
                    telemetry.update();
                }
            }
        });
        public static final Thread asyncScore = new Thread(()-> {
            try {
                await(elevator::almostReachedWantedPosition);

                puffer.goToOut();

                sleep(275);
                await(() -> (gamepad1.right_trigger > 0 || RobotSystem.manual.asyncScore.isInterrupted()));

                if (manual.asyncScore.isInterrupted()){
                    throw softStopRequest;
                }

                puffer.release();

                await(() -> gamepad1.right_trigger == 0 || RobotSystem.manual.asyncScore.isInterrupted());

                throw softStopRequest;
            }catch (InterruptedException e1){
                if (e1.equals(softStopRequest)){
                    try {
                        puffer.goToIn();

                        sleep(200);

                        elevator.setWantedPosition(elevator.bottomPosition);

                        await(() -> !elevator.isUp());

                        sleep(300);
                        if (!grabber.isOut() && grabber.hasCone()){
                            puffer.goToMid();
                        }
                        else {
                            puffer.release();
                        }

                    } catch (Exception e2){
                        telemetry.addData("stopped manual score from safely stopping", e2);
                        telemetry.update();
                    }
                }
            } catch (Exception e){
                telemetry.addData("stopped manual score completely", e);
                telemetry.update();
            }
        });
    }
    // endregion

    // region AUTONOMOUS FUNCTIONS
    public static class auto {
        public static class safeAuto {
            // region INITIALIZATION
            public static void initializeAll(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2){
                RobotSystem.initialize(hardwareMap, telemetry, gamepad1, gamepad2);
                initialize();

                arm.initialize();
                puffer.initialize();
                grabber.initialize();
                elevator.initialize();
            }

            public static void initialize(){
                initializeCamera();

                // region INITIALIZE DRIVE
                drive = new SampleMecanumDrive(hardwareMap);
                drive.setPoseEstimate(positions.start);
                // endregion

                // region CREATE TRAJECTORIES
                // region START TO SCORE
                trajectories.startToScore = drive.trajectorySequenceBuilder(positions.start)
                        .setTangent(positions.startToScoreStartingTangent)
                        .splineToConstantHeading(positions.startToScoreTemp1, Math.abs(-90))
                        .splineToConstantHeading(positions.startToScoreTemp2, Math.abs(-90))
                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(positions.startToScoreTemp3, Math.toRadians(180))
                        .build();
                // endregion

                // region SCORE TO COLLECT
                trajectories.scoreToCollect = drive.trajectorySequenceBuilder(positions.score)
                        .setTangent(0)
                        .splineToSplineHeading(positions.collect, Math.toRadians(0))
                        .build();
                // endregion

                // region COLLECT TO SCORE
                trajectories.collectToScore = drive.trajectorySequenceBuilder(positions.collect)
                        .setTangent(Math.toRadians(180))
                        .splineToSplineHeading(positions.score, Math.toRadians(180))
                        .build();
                // endregion

                // region SCORE TO PARK1
                trajectories.scoreToPark1 = drive.trajectorySequenceBuilder(positions.score)
                        .setTangent(Math.toRadians(0))
                        .splineToConstantHeading(positions.park1, Math.toRadians(0))
                        .build();
                // endregion

                // region SCORE TO PARK2
                trajectories.scoreToPark2 = drive.trajectorySequenceBuilder(positions.score)
                        .splineToConstantHeading(positions.park2, Math.toRadians(0))
                        .build();
                // endregion

                // region SCORE TO PARK3
                trajectories.scoreToPark3 = drive.trajectorySequenceBuilder(positions.score)
                        .splineToConstantHeading(positions.park3, Math.toRadians(180))
                        .build();
                // endregion
                // endregion
            }
            // endregion

            // region MOVEMENT
            public static class trajectories{
                public static TrajectorySequence startToScore;
                public static TrajectorySequence scoreToCollect;
                public static TrajectorySequence collectToScore;
                public static TrajectorySequence scoreToPark1;
                public static TrajectorySequence scoreToPark2;
                public static TrajectorySequence scoreToPark3;
            }
            private static class positions{
                public static final double startToScoreStartingTangent = Math.toRadians(-40);

                public static final Pose2d start   = new Pose2d(32.8, 58, Math.toRadians(-90));
                public static final Pose2d score   = new Pose2d(24, 12, Math.toRadians(-90));
                public static final Pose2d collect = new Pose2d(36, 12, Math.toRadians(180));
                public static final Vector2d park1 = new Vector2d(65, 12);
                public static final Vector2d park2 = new Vector2d(36, 12);
                public static final Vector2d park3 = new Vector2d(12, 12);

                public static final Vector2d startToScoreTemp1 = new Vector2d(36, 48);
                public static final Vector2d startToScoreTemp2 = new Vector2d(36, 12);
                public static final Vector2d startToScoreTemp3 = new Vector2d(score.getX(), score.getY());

            }
            // endregion

            // region FUNCTIONALITY
            public static void cycle(int cycleAmount) throws InterruptedException{
                for (int coneHeight = 4; coneHeight > 4 - cycleAmount; coneHeight--) {
                    goToCollect();

                    follow(trajectories.scoreToCollect);
                    collect(coneHeight);

                    follow(trajectories.collectToScore);
                    goToScore();
                    score();
                }
                finishScoring();
            }

            public static void goToCollect() throws InterruptedException{
                finishScoring();
                await(() -> !elevator.isUp());
            }
            public static void collect(int coneNum) throws InterruptedException{
                await(safeAuto::isStationary);
                grabber.goToCone(coneNum);
                grabber.release();
                arm.goToRelativePosition(1);
                sleep(500);
                await(grabber::coneIsInRange);
                grabber.grab();
                await(grabber::hasCone);
                grabber.goToMid();
                sleep(500);
                arm.goToRelativePosition(0);
                await(() -> !arm.isOut());
            }
            public static void goToScore() throws InterruptedException{
                grabber.goToIn();
                await(() -> !grabber.isOut());
                grabber.release();
                puffer.grab();
                puffer.goToMid();
            }
            public static void score() throws InterruptedException{
                elevator.wantedPosition = elevator.highPosition;
                await(elevator::almostReachedWantedPosition);
                puffer.goToOut();
                sleep(500);
                await(safeAuto::isStationary);
                puffer.release();
                sleep(500);
            }
            public static void finishScoring() throws InterruptedException{
                puffer.goToIn();
                sleep(500);
                elevator.wantedPosition = elevator.bottomPosition;
            }
            // endregion

            // region DRIVE
            private static SampleMecanumDrive drive;
            // endregion

            // region CAMERA
            private static OpenCvWebcam webcam;
            public static void initializeCamera(){
                webcam = OpenCvCameraFactory.getInstance().createWebcam(
                        hardwareMap.get(WebcamName.class, "cam"),
                        hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
                webcam.setPipeline(new PipeLine(false));

                webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    }

                    @Override
                    public void onError(int errorCode) {}
                });
            }
            // endregion

            // region GLOBAL FUNCTIONALITY
            public static boolean isStationary(){
                return !drive.isBusy();
            }
            public static void follow(TrajectorySequence sequence){
                drive.followTrajectorySequenceAsync(sequence);
                followThread.start();
            }
            private static final Thread followThread = new Thread(() -> {
                while (!isStationary() && !isStopRequested){
                    drive.update();
                }
            });
            public static void terminate(){
                RobotSystem.terminate();
            }
            // endregion
        }

        public static class regularAuto {
            // region INITIALIZATION
            public static void initializeAll(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2){
                RobotSystem.initialize(hardwareMap, telemetry, gamepad1, gamepad2);
                initialize();

                arm.initialize();
                puffer.initialize();
                grabber.initialize();
                elevator.initialize();
            }

            public static void initialize(){
                // region INITIALIZE CAMERA
                webcam = OpenCvCameraFactory.getInstance().createWebcam(
                        hardwareMap.get(WebcamName.class, "cam"),
                        hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
                webcam.setPipeline(new PipeLine(false));

                webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                    }

                    @Override
                    public void onError(int errorCode) {}
                });

                // endregion

                // region INITIALIZE DRIVE
                drive = new SampleMecanumDrive(hardwareMap);
                drive.setPoseEstimate(positions.start);
                // endregion

                // region CREATE TRAJECTORIES
                // region START TO SCORE
                trajectories.startToScore = drive.trajectorySequenceBuilder(positions.start)
                        .setTangent(positions.startToScoreStartingTangent)
                        .splineToSplineHeading(positions.startToScoreTemp1, Math.toRadians(-90.0))
                        .splineToSplineHeading(positions.score, Math.toRadians(-100.0))
                        .build();
                // endregion


                // region SCORE TO PARKING 1
                trajectories.scoreToPark1 = drive.trajectorySequenceBuilder(positions.score)
                        .setTangent(Math.toRadians(90.0))
                        .splineToLinearHeading(positions.scoreToParking1Temp1, Math.toRadians(-90.0))
                        .splineToLinearHeading(positions.parking1, Math.toRadians(0.0)).build();
                // endregion

                // region SCORE TO PARKING 2
                trajectories.scoreToPark2 = drive.trajectorySequenceBuilder(positions.score)
                        .setTangent(Math.toRadians(90.0))
                        .splineToLinearHeading(positions.parking2, Math.toRadians(-90.0)).build();
                // endregion

                // region SCORE TO PARKING 3
                trajectories.scoreToPark3 = drive.trajectorySequenceBuilder(positions.score)
                        .setTangent(Math.toRadians(90.0))
                        .splineToLinearHeading(positions.scoreToParking3Temp1, Math.toRadians(-90.0))
                        .splineToLinearHeading(positions.parking3, Math.toRadians(0.0)).build();
                // endregion
                // endregion
            }

            public static void terminate(){
                RobotSystem.terminate();
            }
            // endregion

            // region MOVEMENT
            private static SampleMecanumDrive drive;
            public static class trajectories{
                public static TrajectorySequence startToScore;
                public static TrajectorySequence scoreToPark3;
                public static TrajectorySequence scoreToPark2;
                public static TrajectorySequence scoreToPark1;
            }
            private static class positions{
                public static final double startToScoreStartingTangent = Math.toRadians(-40);
                public static final Pose2d start = new Pose2d(30.7, 61.4, Math.toRadians(-90.0));
                public static final Pose2d score = new Pose2d(30.0, 12.0, Math.toRadians(-140.0));
                public static final Pose2d parking2 = new Pose2d(36.0, 24.0, Math.toRadians(90.0));
                public static final Pose2d parking1 = new Pose2d(48.0, 36.0, Math.toRadians(0.0));
                public static final Pose2d parking3 = new Pose2d(24.0, 36.0, Math.toRadians(180.0));

                public static final Pose2d scoreToParking1Temp1 = new Pose2d(36.0, 24.0, Math.toRadians(90.0));
                public static final Pose2d scoreToParking3Temp1 = new Pose2d(36.0, 24.0, Math.toRadians(90.0));
                public static final Pose2d startToScoreTemp1    = new Pose2d(30.0, 48.0, Math.toRadians(-90.0));
            }

            public static void follow(TrajectorySequence sequence){
                drive.followTrajectorySequenceAsync(sequence);
                followThread.start();
            }
            private static final Thread followThread = new Thread(() -> {
                while (!isStationary() && !isStopRequested){
                    drive.update();
                }
            });
            // endregion

            // region CYCLE
            public static void cycle(int cycleAmount) throws InterruptedException{
                for (int coneHeight = 4; coneHeight > 5 - cycleAmount; coneHeight--) {
                    scoreAndPrepare(coneHeight);
                    collect();
                }
                score();
            }

            public static void score() throws InterruptedException{
                elevator.wantedPosition = elevator.highPosition;
                await(elevator::almostReachedWantedPosition);
                puffer.goToOut();
                sleep(500);
                puffer.release();
                sleep(500);
                puffer.goToIn();
                sleep(500);
                elevator.wantedPosition = elevator.bottomPosition;
            }
            public static void scoreAndPrepare(int coneHeight) throws InterruptedException{
                elevator.wantedPosition = elevator.highPosition;
                sleep(500);
                prepareForCollect(coneHeight);
                await(elevator::almostReachedWantedPosition);
                puffer.goToOut();
                sleep(500);
                puffer.release();
                sleep(500);
                puffer.goToIn();
                sleep(500);
                elevator.wantedPosition = elevator.bottomPosition;
            }
            public static void prepareForCollect(int cone){
                grabber.goToCone(cone);
                arm.goToRelativePosition(0.8);
                grabber.release();
            }
            public static void collect() throws InterruptedException{
                arm.goToRelativePosition(1);
                sleep(500);
                await(grabber::coneIsInRange);
                grabber.grab();
                await(grabber::hasCone);
                grabber.goToMid();
                sleep(500);
                arm.goToRelativePosition(0);
                await(() -> !arm.isOut() && !elevator.isUp());
                grabber.goToIn();
                await(() -> !grabber.isOut());
                grabber.release();
                puffer.grab();
                puffer.goToMid();
            }

            public static boolean isStationary(){
                return !drive.isBusy();
            }
            // endregion

            // region CAMERA
            private static OpenCvWebcam webcam;
            // endregion
        }
    }
    // endregion
}
