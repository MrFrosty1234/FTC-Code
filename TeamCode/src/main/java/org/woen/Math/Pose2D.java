package org.woen.Math;



public class Pose2D {



        public double x;

        public double y;
        public double h;

    public Pose2D(double x, double y, double h){
        this.x = x ;
        this.y = y ;
        this.h = h;
    }

    public Pose2D(){
        this.x = 0;
        this.h = 0;
        this.y = 0;
    }

    public void rotateVector(double angle){
        double radians = Math.toRadians(angle);

        double y1 = Math.cos(radians) * y + Math.sin(radians) * x;
        double x1 = Math.cos(radians) * x - Math.sin(radians) * y;

        x = x1;
        y = y1;
    }

    public void minusVector(Pose2D pos){
        x -= pos.x;
        y -= pos.y;
    }

    public void vectorPlus(Pose2D pos) {
        x += pos.x;
        y += pos.y;
    }

    public void plusVector(Pose2D pos){
        x += pos.x;
        y += pos.y;
    }

    public Pose2D minusPos(Pose2D pos){
        x -= pos.x;
        y -= pos.y;
        h -= pos.h;
        return new Pose2D(x, y , h);
    }
    public void  plusPos(Pose2D pos){
        x += pos.x;
        y += pos.y;
        h += pos.h;
    }

    public double getLength(){
        return Math.sqrt(x*x+y*y) ;
    }

    public static double length(Pose2D s, Pose2D e){
        return Math.sqrt((s.x-e.x)*(s.x-e.x) - (s.y-e.y)*(s.y-e.y));
    }

}
