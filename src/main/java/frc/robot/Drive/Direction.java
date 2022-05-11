package frc.robot.Drive;

public enum Direction {
    FORWARD(1), 
    BACKWARD(-1);

    private final double direction;

    private Direction(double direction) {
        this.direction = direction;
    }

    public double get() {
        return this.direction;
    }
}