using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class JointMotor
{
    public enum ActivationMode { direct, oscillator }
    private float internalTime;


    public void ActivateDirectJoint(ConfigurableJoint j, Vector3 rotationSignal, float forceMax, float forceSignal)
    {
        Vector3 constrainedRotation = Vector3.zero;
        float minRotation = j.lowAngularXLimit.limit;
        float maxRotation = j.highAngularXLimit.limit;

        minRotation = j.lowAngularXLimit.limit;
        maxRotation = j.highAngularXLimit.limit;
        constrainedRotation.x = DirectSignal(minRotation, maxRotation, rotationSignal.x);

        minRotation = -j.angularYLimit.limit;
        maxRotation = j.angularYLimit.limit;
        constrainedRotation.y = DirectSignal(minRotation, maxRotation, rotationSignal.y);


        minRotation = -j.angularZLimit.limit;
        maxRotation = j.angularZLimit.limit;
        constrainedRotation.z = DirectSignal(minRotation, maxRotation, rotationSignal.z);

        float forceOutput = DirectSignal(minRotation, maxRotation, forceSignal);

        float maximumForce = forceMax * forceSignal;
        //SetJointDriveMaximumForce(j, maximumForce);
    }

    //signals will vary between -1, 1
    public float DirectSignal(float min, float max, float amplitudeSignal)
    {
        return Mathf.Lerp(min, max, Mathf.InverseLerp(-1, 1, amplitudeSignal));
    }

    public float OscillateSignal(float min, float max, float phaseSignal, float frequencySignal,  float amplitudeSignal, float biasSignal)
    {
        //remap signals to needed ranges
        float frequency = Mathf.InverseLerp(-1f, 1, frequencySignal)*6f+1f;
        float amplitude = Mathf.InverseLerp(-1, 1, amplitudeSignal);
        float bias  = Mathf.InverseLerp(-1, 1, biasSignal);
        float phase = Mathf.InverseLerp(-1, 1, phaseSignal) * 6.28f;

        internalTime += frequency * Time.deltaTime;

        return Mathf.Lerp(min, max, Mathf.Clamp(Mathf.Sin((internalTime + phase)) * amplitude + bias, 0, 1));
    }
}
