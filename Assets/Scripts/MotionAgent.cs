using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
public class MotionAgent : Agent
{ 
    public Transform target;
    public float positionSpringPower = 100;
    public float positionDamperMultiplier = 2; //how much stronger should the position damper be than max spring force
    public float maxSpringForce = 200;
    
    private float initialMaxSpringForce;
    private Rigidbody core;

    public BodyGenerator body;

    public float minUpVectorDot = 0.9f;
    public float distanceToTouchTarget = 1f;
    public float targetDistanceRange = 18f;

    public enum RewardMode {SeekTarget, StandUp};
    public RewardMode rewardMode;
    
    public bool immortal = false;
    
    public bool randomizeMaxSpringForce = false;
    //public bool randomizeLegScales = false;
    
    public float frontScale = 0.75f;
    public float backScale = 0.75f;
    
    private float efficiencyRollingAverage = 0;
    private float efficiencyRollingAverageDepth = 10; 
    
    private Vector3 velocityRollingAverage = Vector3.zero;
    private float velocityRollingAverageDepth = 20;
    
    private bool touchingGroundOtherThanFeet = false;
    
    private float forceUsedPercent = 0;

    public bool stateBasedModelSwitching = false;

    public Unity.Barracuda.NNModel seekTargetModel; 
    public Unity.Barracuda.NNModel standUpModel;
    void Start()
    {
        core = body.chest;
        this.initialMaxSpringForce = maxSpringForce;

        foreach (ConfigurableJoint cj in body.motorJoints)
        {
            SetJointDriveMaximumForce(cj, maxSpringForce);
        }
    }

    private void Update()
    {
        if (stateBasedModelSwitching)
        {
            if (rewardMode == RewardMode.StandUp)
            {
                if (FallenDownConditions() == false)
                {
                    
                    rewardMode = RewardMode.SeekTarget;
                    this.SetModel("AdvancedQuadrupedBehavior", seekTargetModel);
                    
                }
            }else if (rewardMode == RewardMode.SeekTarget)
            {
                if (FallenDownConditions() == true)
                {
                    
                    rewardMode = RewardMode.StandUp;
                    this.SetModel("AdvancedQuadrupedBehavior", standUpModel);
                    
                }
            }
        }
    }

    public void FixedUpdate()
    {
        CheckTouchingGroundOtherThanFeet();
    }

    //Used to reset a training scenario
    //randomize goal location
    //reset position and orientation of agent
    //etc...
    public override void OnEpisodeBegin()
    {
        SetUpTrainingMode();

        if (randomizeMaxSpringForce)
        {
            this.maxSpringForce = this.initialMaxSpringForce * RandomGaussian();
            //Debug.Log(this.maxSpringForce / this.initialMaxSpringForce);
        }
    }


    private void SetUpTrainingMode()
    {
        if (rewardMode == RewardMode.StandUp)
        {
            if (target.gameObject.activeSelf)
            {
                target.gameObject.SetActive(false);
            }
            ResetLocation();
            //flip around to random rotation
            this.core.MoveRotation(Quaternion.Euler(new Vector3(0f, Random.value * 360, 90 + Random.value * 180)));
        }

        if (rewardMode == RewardMode.SeekTarget)
        {
            if (!target.gameObject.activeSelf)
            {
                target.gameObject.SetActive(true);
            }

            if (FallenDownConditions())
            {
                ResetLocation();
                //face random direction
                this.core.MoveRotation(Quaternion.Euler(new Vector3(0, Random.value * 360, 0)));
            }

            while (DistanceToTarget() < distanceToTouchTarget * 3)
            {
                RandomizeTargetLocation();
            }
        }

    }

    private void ResetLocation()
    {
        core.transform.localPosition = new Vector3(0, 2f, 0);
        core.transform.rotation = Quaternion.identity;
        core.velocity = Vector3.zero;
        core.angularVelocity = Vector3.zero;
    }

    private void RandomizeTargetLocation()
    {
        target.localPosition = new Vector3(Random.value * targetDistanceRange - targetDistanceRange/2, 0, Random.value * targetDistanceRange - targetDistanceRange/2);
    }

    private bool FallenDownConditions()
    {
        return (touchingGroundOtherThanFeet || Vector3.Dot(core.transform.up, Vector3.up) < minUpVectorDot);
    }

    //total observation size 32
    public override void CollectObservations(VectorSensor sensor)
    {
        Debug.Log(body.sensedSegments.Count + " SensedSegments");
        
        //target position vec3
        sensor.AddObservation(core.transform.InverseTransformPoint(target.position).normalized);

        //limb positions vec3 * 12
        //limb rotation vec3 * 12
        //limb touchign ground bool * 12
        foreach (Rigidbody seg in body.sensedSegments)
        {
            //segment attributes relative to core
            sensor.AddObservation(core.transform.InverseTransformPoint(seg.position));
            sensor.AddObservation(seg.transform.localRotation);
            sensor.AddObservation(core.transform.InverseTransformDirection(seg.velocity));
            sensor.AddObservation(core.transform.InverseTransformDirection(seg.angularVelocity));
            sensor.AddObservation(seg.GetComponent<TouchingGround>().touching);
        }
        
        sensor.AddObservation(this.maxSpringForce);
        /*
        sensor.AddObservation(frontScale);
        sensor.AddObservation(backScale);
        */
        sensor.
    }

    //execute the actions given from the brain
    //update rewards
    public override void OnActionReceived(float[] vectorAction)
    {
        Vector3 controlSignal = Vector3.zero;
        float minRotation;
        float maxRotation;

        this.forceUsedPercent = 0f;

        for (int i = 0; i < body.motorJoints.Count; i++) {
            ConfigurableJoint cJoint = body.motorJoints[i];

            minRotation = cJoint.lowAngularXLimit.limit;
            maxRotation = cJoint.highAngularXLimit.limit;
            controlSignal.x = Mathf.Lerp(minRotation, maxRotation, Mathf.InverseLerp(-1, 1, vectorAction[(i * 4)]));

            minRotation = -cJoint.angularYLimit.limit;
            maxRotation = cJoint.angularYLimit.limit;
            controlSignal.y = Mathf.Lerp(minRotation, maxRotation, Mathf.InverseLerp(-1, 1, vectorAction[(i * 4) + 1]));

            minRotation = -cJoint.angularZLimit.limit;
            maxRotation = cJoint.angularZLimit.limit;
            controlSignal.z = Mathf.Lerp(minRotation, maxRotation, Mathf.InverseLerp(-1, 1, vectorAction[(i * 4) + 2]));

            float forceSignal = Mathf.InverseLerp(-1, 1, vectorAction[i * 4 + 3]);
            forceUsedPercent += forceSignal / body.motorJoints.Count;

            float maximumForce = maxSpringForce * forceSignal;
            SetJointDriveMaximumForce(cJoint, maximumForce);

            cJoint.targetRotation = Quaternion.Euler(controlSignal);
        }

        if (rewardMode == RewardMode.SeekTarget)
        {
            SeekTargetReward();
        }

        if (rewardMode == RewardMode.StandUp)
        {
            StandUpReward();
        }
    }

    private void StandUpReward()
    {

        if (FallenDownConditions() == false)
        {
            AddReward(10f);
            if (!immortal)
            {
                EndEpisode();
            }
        }

        float reward = 0.0f;
        float upVectorSignal = Mathf.InverseLerp(-1, 1, (Vector3.Dot(core.transform.up, Vector3.up))) * 0.2f;
        reward += Mathf.Pow(upVectorSignal, 4);
        AddReward(reward);
    }

    private void SeekTargetReward()
    {

        if (FallenDownConditions())
        {
            SetReward(-1.0f);
            if (!immortal)
            {
                EndEpisode();
            }
        }

        float efficiency = (1 - forceUsedPercent);
        efficiencyRollingAverage = (efficiencyRollingAverage + (efficiency / efficiencyRollingAverageDepth)) / (1 + 1f / efficiencyRollingAverageDepth);
        //Debug.Log("e:" + efficiency + " era" + efficiencyRollingAverage.ToString());

        velocityRollingAverage = (velocityRollingAverage + (core.velocity / velocityRollingAverageDepth)) / (1 + 1f / velocityRollingAverageDepth);
        float velocityDeltaFromAverage = (velocityRollingAverage - core.velocity).magnitude;
        float movementSmoothness = 1 - (float)System.Math.Tanh(velocityDeltaFromAverage * 2f);


        //0 is away from target, 1 is towards target
        float facingTarget = Mathf.InverseLerp(-1, 1, GetFacingTarget());
        //float facingTargetPow2 = Mathf.Pow(facingTarget, 2);

        float velocityTowardsTarget = core.velocity.magnitude * Vector3.Dot(core.velocity.normalized, (target.position - core.transform.position).normalized);
        //velocity towards target limit 1
        float vttl1 = Mathf.Max((float)System.Math.Tanh(velocityTowardsTarget * 0.25f), 0);
        Debug.Log(Mathf.Pow(vttl1, 2));
        DrawRedGreen(Mathf.Pow(vttl1, 2));

        float levelHorizon = Mathf.InverseLerp(minUpVectorDot, 1f, Vector3.Dot(core.transform.up, Vector3.up));

        float reward = 1;

        reward *= facingTarget;
        reward *= Mathf.Pow(vttl1, 2);
        reward *= efficiencyRollingAverage;
        reward *= movementSmoothness;
        //reward *= levelHorizon;

        //DrawRedGreen(vttl1);
        //Debug.Log("vttl1: " + vttl1);


        AddReward(reward); //reward for moving towards goal
        //Debug.Log("reward:" + reward);

        //Debug.Log(distanceToTarget);
        if (DistanceToTarget() < distanceToTouchTarget)
        {
            SetReward(1.0f);
            EndEpisode();
        }
    }
    private float DistanceToTarget()
    {
        return Vector3.Distance(core.transform.localPosition, target.localPosition);
    }
    private void DrawRedGreen(float reward)
    {

        //Debug.Log("draw reward: "+ transform.GetComponentsInChildren<Renderer>().ToString());
        foreach (Renderer rend in transform.GetComponentsInChildren<Renderer>())
        {
            Color c;
            if (reward < 0)
            {
                c = Color.Lerp(Color.red, Color.white, Mathf.InverseLerp(-1, 0, reward));
            }
            else
            {
                c = Color.Lerp(Color.white, Color.green, Mathf.InverseLerp(0, 1, reward));
            }

            if (rend.material.name == "BodyMatV2 (Instance)") { 
                //rend.material.SetColor("_EmissionColor", c);
                rend.material.SetColor("_BaseColor", c);
            }
        }


    }

    private void SetJointDriveMaximumForce(ConfigurableJoint cJoint, float newMaximumForce)
    {
        JointDrive jDrive = new JointDrive();
        jDrive.maximumForce = newMaximumForce;
        jDrive.positionSpring = this.positionSpringPower;
        jDrive.positionDamper = this.maxSpringForce * positionDamperMultiplier;
        cJoint.slerpDrive = jDrive;
    }

    public override void Heuristic(float[] actionsOut)
    {
        for (int i = 0; i < actionsOut.Length; i++)
        {
            float v = Mathf.InverseLerp(-1, 1, Input.GetAxis("Horizontal"));
            float h = Mathf.InverseLerp(-1, 1, Input.GetAxis("Vertical"));
            actionsOut[i] = (i % 2 == 0) ? v : h;
        }
    }

    public float GetFacingTarget()
    {
        return Vector3.Dot(core.transform.forward, (core.transform.position - target.position).normalized);
    }

    public float RandomGaussian()
    {
        //average of three random values approximates gaussian sample
        return (Random.value + Random.value + Random.value) / 3f;
    }

    private bool CheckTouchingGroundOtherThanFeet()
    {
        touchingGroundOtherThanFeet = false;
        foreach (Rigidbody seg in body.sensedSegments)
        {
            if (seg.name != "lower")
            {
                touchingGroundOtherThanFeet = true;
            }
        }
        return touchingGroundOtherThanFeet;
    }
}