package org.firstinspires.ftc.teamcode.subsystems.Vision;

public class AllianceDetector {
    // Configurables
    public double fieldCenterY = 0.0;


    public double thresholdMeters = 0.3;
    public int maxSamples = 5;

    // Estado
    private Alliance alliance = Alliance.UNKNOW;
    private int redCount = 0;
    private int blueCount = 0;
    private int samples = 0;

    public AllianceDetector() {}
    public AllianceDetector(int maxSamples) { this.maxSamples = maxSamples; }

    /**
     * Llamar en cada loop con la última pose corregida (vp).
     * Devuelve true cuando la alianza quedó decidida.
     */
    public boolean processPose(VisionIO.Pose2dSimple vp) {
        if (alliance != Alliance.UNKNOW) return true;
        if (vp == null) return false;

        Alliance candidate = determineByX(vp);

        switch (candidate) {
            case RED:
                redCount++;
                break;
            case BLUE:
                blueCount++;
                break;
            case UNKNOW:
            default:
                break;
        }

        samples++;

        if (samples >= maxSamples) {
            if (redCount > blueCount) alliance = Alliance.RED;
            else if (blueCount > redCount) alliance = Alliance.BLUE;
            else alliance = Alliance.UNKNOW; // empate o sin lecturas válidas
            return alliance != Alliance.UNKNOW;
        }

        return false;
    }

    private Alliance determineByX(VisionIO.Pose2dSimple vp) {
        if (vp == null) return Alliance.UNKNOW;
        if (vp.y < fieldCenterY - thresholdMeters) return Alliance.BLUE;
        if (vp.y > fieldCenterY + thresholdMeters) return Alliance.RED;

        return Alliance.UNKNOW;
    }

    public Alliance getAlliance() { return alliance; }
    public int getRedCount() { return redCount; }
    public int getBlueCount() { return blueCount; }
    public int getSamples() { return samples; }

    public void reset() {
        alliance = Alliance.UNKNOW;
        redCount = 0;
        blueCount = 0;
        samples = 0;
    }
}