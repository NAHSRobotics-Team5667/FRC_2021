package frc.robot.utils;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

public class MecanumOdometry {
    private Pose2d robotPose = new Pose2d();

    private double leftDistance = 0;
    private double rightDistance = 0;
    private double heading = 0;

    private double trackWidthInches;

    /**
     * @param trackWidthInches distance between the two same-side tracking wheels.
     * @param initialPose initial pose of the robot.
     */
    public MecanumOdometry(double trackWidthInches, Pose2d initialPose) {
        this.robotPose = initialPose;
        this.trackWidthInches = trackWidthInches;
        this.heading = initialPose.getRotation().getDegrees();
    }

    /**
     * Updates robot position on the field given robot left, right, and strafe values. If encoders are flipped, inversion 
     * should be accounted for in parameters.
     * 
     * @param leftDistance inches recorded by left encoder wheel.
     * @param rightDistance inches recorded by right encoder wheel.
     * @param perpendicularDistance avg. inches recorded by strafe odometers.
     */
    public void update(double leftDistance, double rightDistance, double perpendicularDistance) {
        double deltaTheta = ((leftDistance - this.leftDistance) + (rightDistance - this.rightDistance)) / trackWidthInches;
        double deltaCenter = ((leftDistance - this.leftDistance) + (rightDistance - this.rightDistance)) / 2;

        double deltaX = (deltaCenter * Math.cos(this.heading + (deltaTheta / 2))) - (perpendicularDistance * Math.sin(this.heading + (deltaTheta / 2)));
        double deltaY = (deltaCenter * Math.sin(this.heading + (deltaTheta / 2))) + (perpendicularDistance * Math.cos(this.heading + (deltaTheta / 2)));

        robotPose = new Pose2d(
            robotPose.getX() + deltaX,
            robotPose.getY() + deltaY,
            new Rotation2d(Math.toRadians(heading))
        );

        this.leftDistance = leftDistance;
        this.rightDistance = rightDistance;
        heading = heading + deltaTheta;
    }

    /**
     * @return the estimated position of the robot based on odometry tracking wheels.
     */
    public Pose2d getPoseEstimate() {
        return robotPose;
    }
}