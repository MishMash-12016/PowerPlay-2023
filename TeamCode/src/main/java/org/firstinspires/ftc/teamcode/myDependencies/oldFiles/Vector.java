package org.firstinspires.ftc.teamcode.myDependencies.oldFiles;

public class Vector {
    public double x;
    public double y;

    public Vector() {
    }

    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getAngle() {
        double a = Math.PI * 2;
        if (y < 0) a += Math.PI + Math.atan(-x / y);
        else if (y == 0) {
            if (x < 0) a += Math.PI * 0.5;
            else a += Math.PI * 1.5;
        } else {
            a += Math.atan(-x / y);
        }
        a = Math.PI * 2 - a % (Math.PI * 2);

        return a;
    }

    public double getLength() {
        return Math.sqrt(Math.pow(this.x, 2) + Math.pow(this.y, 2));
    }

    public void setAngle(double a) {
        double l = this.getLength();
        this.x = Math.sin(a) * l;
        this.y = Math.cos(a) * l;
    }

    public void setLength(double r) {
        double a = this.getAngle();
        this.x = Math.sin(a) * r;
        this.y = Math.cos(a) * r;
    }

    public void addAngle(double angle) {
        this.setAngle(this.getAngle() + angle);
    }
}