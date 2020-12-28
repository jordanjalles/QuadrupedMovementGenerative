using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SignalTester : MonoBehaviour
{
    [Range(-1, 1)]
    public float directSignal;
    [Range(-1, 1)]
    public float phaseSignal;

    [Range(-1, 1)]
    public float amplitudeSignal;
    [Range(-1, 1)]
    public float frequencySignal;
    [Range(-1, 1)]
    public float biasSignal;
    

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        //transform.rotation = Quaternion.Euler((new Vector3(0f, 0f, JointMotor.DirectSignal(90, -90, directSignal))));
        transform.rotation = Quaternion.Euler((new Vector3(0f, 0f, JointMotor.OscillateSignal(90, -90, phaseSignal,  frequencySignal,  amplitudeSignal,  biasSignal))));
    }
}
