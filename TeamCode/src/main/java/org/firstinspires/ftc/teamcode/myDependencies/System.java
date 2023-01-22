package org.firstinspires.ftc.teamcode.myDependencies;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.myDependencies.oldFiles.ElevatorPositions;
import org.firstinspires.ftc.teamcode.myDependencies.oldFiles.Gamepad;
import org.firstinspires.ftc.teamcode.myDependencies.oldFiles.Vector;

public class System {
    /** SYSTEM FUNCTIONALITY */
    public Thread elevatorController;
    public Thread driveController;
    public Thread gamepadController;

    /** GENERAL FUNCTIONALITY */
    public void sleep(int time){}

    /** GAME PAD FUNCTIONALITY IMPORTER */
    static class gamepad{
        public static boolean a = false;
        public static boolean b = false;
        public static boolean y = false;
        public static boolean x = false;

        public static boolean dpadUp    = false;
        public static boolean dpadLeft  = false;
        public static boolean dpadDown  = false;
        public static boolean dpadRight = false;

        public static boolean rightBumper = false;
        public static boolean leftBumper  = false;


        public static Vector2d leftJoystick  = new Vector2d(0, 0);
        public static Vector2d rightJoystick = new Vector2d(0, 0);


        public static double rightTrigger = 0;
        public static double leftTrigger  = 0;

    }

    /** DRIVER CONTROLLED FUNCTIONS */
    static class manual{
        public Thread score;
        public Thread collect;
    }

    /** AUTONOMOUS FUNCTIONS */
    static class auto{
        public Thread autoCycle;

        private void score(boolean prepareForNext, int nextConeNum){}
        private void collect(int coneNum){}
    }


}
