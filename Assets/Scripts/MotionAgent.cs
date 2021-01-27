using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
public class MotionAgent : Agent
{ 
    

    //remove once replaced with body methods
    public float positionSpringPower = 100;
    public float positionDamperMultiplier = 2; //how much stronger should the position damper be than max spring force
    public float maxSpringForce = 200;
    
    private Rigidbody core;

    public BodyGenerator body;

    [Header("Environment settings")]
    [SerializeField]
    private Transform target;
    [SerializeField]
    private float minUpVectorDot = 0.9f;
    [SerializeField]
    private float distanceToTouchTarget = 3f;
    [SerializeField]
    private float targetDistanceRange = 18f;

    
    public enum RewardMode {SeekTarget, StandUp, StayStanding};
    [Header("Training settings")]
    [SerializeField]
    private RewardMode rewardMode;
    [SerializeField]
    private bool randomizeLegScales = false;
    [SerializeField]
    private float minLegScale;
    [SerializeField]
    private float maxLegScale;
    [SerializeField]
    private bool randomizeGrossScale = false;
    [SerializeField]
    private float minGrossScale;
    [SerializeField]
    private float maxGrossScale;

    [Header("Demonstration settings")]
    [SerializeField]
    private bool immortalMode = false;
    [SerializeField]
    private bool stateBasedModelSwitching = false;
    public Unity.Barracuda.NNModel seekTargetModel;
    public Unity.Barracuda.NNModel standUpModel;

    //Used for reward calculations
    private bool successfullEpisode = true;
    private bool slashedRewards = false;
    private float efficiencyRollingAverage = 0;
    private float efficiencyRollingAverageDepth = 10; 
    private Vector3 velocityRollingAverage = Vector3.zero;
    private float velocityRollingAverageDepth = 20;
    private float velocityTowardsTargetRollingAverage;
    private float vttraDepth = 50;

    private bool touchingGroundOtherThanFeet = false;
    private float forceUsedPercent = 0;
    private float[] previousVectorActions;

    [Header("Stay Standing Training Settings")]
    
    [SerializeField]
    private float impulseStartForce;
    private float numImpulses = 0;
    [SerializeField]
    private float timeBetweenImpulses = 3;
    private float prevImpulseTime = 0;




    void Start()
    {
        core = body.chest;

        ResetLocation();
        RandomizeTargetLocation();

    }

    private void Update()
    {
        //todo: add stay standing intermediary with timer

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


        //only change leg sizes if the episode was successful - counteracts the law of the jungle problem
        // mercy switch the unsuccessful bodies some percentage of the time
        bool mercy = (Random.value < 0.2f);
        
        if (randomizeGrossScale && (successfullEpisode || mercy))
        {
            //need to do gross scaling first because it overwrites leg rescaling
            RandomizeGrossScale();
        }
        if (randomizeLegScales && (successfullEpisode || mercy))
        {
            RandomizeLegScales();
        }
        SetUpTrainingMode();

        successfullEpisode = false;
        slashedRewards = false;
    }

    private void RandomizeLegScales()
    {
        
        float frontScale = Mathf.Lerp(minLegScale, maxLegScale, Random.value);
        float backScale = Mathf.Lerp(minLegScale, maxLegScale, Random.value);
        
        for (int i = 0; i < body.limbs.Count; i++)
        {
            if (i < body.limbs.Count / 2)
            {
                body.ScaleSegment(body.limbs[i], frontScale);
            }
            else
            {
                body.ScaleSegment(body.limbs[i], backScale);
            }
        }
    }
    private void RandomizeGrossScale()
    {
        float newScale = Mathf.Lerp(minGrossScale, maxGrossScale, Random.value);
        body.ApplyNewBaseScale(newScale);
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
            body.MoveChest(new Vector3(0, body.baseScale * 2, 0));
            //flip around to random rotation
            core.transform.rotation = (Quaternion.Euler(new Vector3(180f, Random.value * 360, (Random.value - 0.5f) * 180f)));

        }

        if (rewardMode == RewardMode.SeekTarget)
        {
            if (!target.gameObject.activeSelf)
            {
                target.gameObject.SetActive(true);

            }

            if (FallenDownConditions() || core.isKinematic)
            {
                ResetLocation();
                //face random direction
                this.transform.rotation = (Quaternion.Euler(new Vector3(0, Random.value * 360, 0)));
            }

            while (DistanceToTarget() < distanceToTouchTarget * 2)
            {
                RandomizeTargetLocation();
            }
        }
        if (rewardMode == RewardMode.StayStanding)
        {
            ResetLocation();
            RandomizeTargetLocation();
            numImpulses = 0;

            //face random direction
            this.transform.rotation = (Quaternion.Euler(new Vector3(0, Random.value * 360, 0)));

            if (!target.gameObject.activeSelf)
            {
                target.gameObject.SetActive(true);
               
            }

            while (DistanceToTarget() < distanceToTouchTarget * 2)
            {
                RandomizeTargetLocation();
            }
        }

    }

    private void ResetLocation()
    {
        
        body.MoveChest(new Vector3(0, body.baseScale*2, 0));
        core.transform.localRotation = (Quaternion.identity);
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
    public override void CollectObservations(VectorSensor sensor)
    {
        int totalObservations = 0;
        
        //target direction vec3
        sensor.AddObservation(core.transform.InverseTransformPoint(target.position).normalized);
        totalObservations += 3;


        sensor.AddObservation(core.velocity);
        totalObservations += 3;

        sensor.AddObservation(core.angularVelocity);
        totalObservations += 3;

        //limb positions vec3 * 12
        //limb rotation vec3 * 12
        //limb touchign ground bool * 12
        foreach (Rigidbody seg in body.sensedSegments)
        {
            //segment attributes relative to core
            sensor.AddObservation(core.transform.InverseTransformPoint(seg.transform.position));
            totalObservations += 3;

            sensor.AddObservation(seg.transform.localRotation);
            totalObservations += 3;

            sensor.AddObservation(core.transform.InverseTransformDirection(seg.velocity));
            totalObservations += 3;

            sensor.AddObservation(core.transform.InverseTransformDirection(seg.angularVelocity));
            totalObservations += 3;

            sensor.AddObservation(seg.GetComponent<TouchingGround>().touching);
            totalObservations += 1;
        }


    }

    //execute the actions given from the brain
    //update rewards
    public override void OnActionReceived(float[] vectorAction)
    {
        Vector3 controlSignal = Vector3.zero;
        float forceSignal;

        this.forceUsedPercent = 0f;


        float averageSignalDiff = DifferenceBetweenSignalVectors(previousVectorActions, vectorAction)/vectorAction.Length;
        float actionStability = 1 - (averageSignalDiff / 2);
        

        this.previousVectorActions = vectorAction;

        //replacement drive loop with body native functions
        for (int i = 0; i < body.motorJoints.Count; i++)
        {
            controlSignal.x = Mathf.InverseLerp(-1, 1, vectorAction[(i * 4)]);
            controlSignal.y = Mathf.InverseLerp(-1, 1, vectorAction[(i * 4) + 1]);
            controlSignal.z = Mathf.InverseLerp(-1, 1, vectorAction[(i * 4) + 2]);
            forceSignal = Mathf.InverseLerp(-1, 1, vectorAction[i * 4 + 3]);

            forceUsedPercent += forceSignal / body.motorJoints.Count;
            body.DriveJoint(body.motorJoints[i], controlSignal, forceSignal);
        }
        
        if (rewardMode == RewardMode.SeekTarget)
        {
            SeekTargetReward();
        }

        if (rewardMode == RewardMode.StandUp)
        {
            StandUpReward();
        }        
        
        if (rewardMode == RewardMode.StayStanding)
        {
            StayStandingReward();
        }
    }

    private void StandUpReward()
    {

        float reward = 1f;



        if (FallenDownConditions() == false)
        {
            float stepsRemaining = MaxStep - StepCount;
            //SetReward((stepsRemaining / ((float)StepCount+1f)));

            SetReward(100f * (stepsRemaining / MaxStep));
            if (!immortalMode)
            {
                successfullEpisode = true;
                EndEpisode();
            }
        }

        //still rougly gettingup


        float upRightNess = (Vector3.Dot(core.transform.up, Vector3.up));

        reward *= Mathf.Pow(upRightNess, 4) * 0.01f * Mathf.Sign(upRightNess);
        reward *= CheckNumberOfFeetTouchingGround() + 1;

        AddReward(reward);
    }

    private void SeekTargetReward()
    {
        

        if (FallenDownConditions())
        {
            SetReward(-10f);

            if (!immortalMode)
            {
                successfullEpisode = false;
                EndEpisode();
                return;
                
            }
            //Debug.Log(GetCumulativeReward());
        }

        if (DistanceToTarget() < distanceToTouchTarget)
        {
            //reward for getting target multiplies current reward amount by ratio of time remaining.
            //that's so we don't punish the quick runners

            float stepsRemaining = MaxStep - StepCount;
            AddReward((GetCumulativeReward() * (stepsRemaining / StepCount)));

            successfullEpisode = true;
            EndEpisode();
            //return;
        }


        float efficiency = (1 - forceUsedPercent);
        efficiencyRollingAverage = (efficiencyRollingAverage + (efficiency / efficiencyRollingAverageDepth)) / (1 + 1f / efficiencyRollingAverageDepth);

        /*
        velocityRollingAverage = (velocityRollingAverage + (core.velocity / velocityRollingAverageDepth)) / (1 + 1f / velocityRollingAverageDepth);
        float velocityDeltaFromAverage = (velocityRollingAverage - core.velocity).magnitude;
        float movementSmoothness = 1 - (float)System.Math.Tanh(velocityDeltaFromAverage * 2f);
        */

        //0 is away from target, 1 is towards target
        float facingTarget = Mathf.InverseLerp(-1, 1, GetFacingTarget());
       

        float velocityTowardsTarget = core.velocity.magnitude * Vector3.Dot(core.velocity.normalized, (target.position - core.transform.position).normalized);
        //velocityTowardsTarget = Mathf.Max(velocityTowardsTarget, 0);

        velocityTowardsTargetRollingAverage = (velocityTowardsTargetRollingAverage + (velocityTowardsTarget / vttraDepth))/(1 + 1f/ vttraDepth);
        velocityTowardsTargetRollingAverage = Mathf.Max(velocityTowardsTargetRollingAverage, 0);

        float forwardVelocity = (-core.transform.InverseTransformDirection(core.velocity).z);

        float levelHorizon = Mathf.InverseLerp(minUpVectorDot, 1f, Vector3.Dot(core.transform.up, Vector3.up));

        float rewardTierSize = 1.0f;
        float reward = 1;

        //tiered reward factor inclusion
        reward *= Mathf.Pow(facingTarget,2);
        reward *= forwardVelocity;
        
        if (reward >= 0) reward *= efficiencyRollingAverage;

        //if (GetCumulativeReward() >= 0 * rewardTierSize) reward *= movementSmoothness;

        AddReward(reward); //reward for moving towards goal

        DrawRedGreen(reward);
        
        if (reward < 0)
        {
            Debug.LogWarning("Reward somehow less than zero: " + reward);
        }


    }
    private void StayStandingReward()
    {

        
        if (CheckTouchingGroundOtherThanFeet())
        {
            SetReward(-0.1f);
            DrawRedGreen(-1f);
            if (GetCumulativeReward() < -10)
            {
                EndEpisode();
            }
        }
        else
        {
            if (Time.time - prevImpulseTime > timeBetweenImpulses)
            {
                numImpulses += 1;
                prevImpulseTime = Time.time;
                Vector3 forceDirection = new Vector3(Random.value - 0.5f, Random.value - 0.5f, Random.value - 0.5f).normalized;
                Vector3 force = forceDirection * impulseStartForce * numImpulses;
                Debug.Log(force);

                core.AddForce(force, ForceMode.Impulse);
            }

            float uprightness = Vector3.Dot(core.transform.up, Vector3.up);

            float efficiency = (1 - forceUsedPercent);

            //0 is away from target, 1 is towards target
            float facingTarget = Mathf.InverseLerp(-1, 1, GetFacingTarget());

            float stillness = 1f - (float)System.Math.Tanh(core.velocity.magnitude);

            float reward = 1;

            reward *= uprightness;
            reward *= efficiency;
            reward *= facingTarget;
            reward *= facingTarget;
            reward *= stillness;

            SetReward(reward); //reward for moving towards goal

            DrawRedGreen(reward);
        }



    }
    private float DistanceToTarget()
    {
        return Vector3.Distance(core.transform.position, target.position);
    }

    private float DifferenceBetweenSignalVectors(float[] vectorAction1, float[] vectorAction2)
    {
        float diff = 0f;

        if (vectorAction1 != null && vectorAction2 != null)
        {
            for( int i = 0; i < vectorAction1.Length; i++)
            {
                diff += Mathf.Abs(vectorAction1[i] - vectorAction2[i]);
            }
        }
        return diff;
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

    public override void Heuristic(float[] actionsOut)
    {
        for (int i = 0; i < actionsOut.Length; i++)
        {

            float v = Input.GetAxis("Horizontal");
            float h = Input.GetAxis("Vertical");
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
            if (seg.GetComponent<TouchingGround>().touching == true)
                {
                if (seg.name != "lower")
                {
                    touchingGroundOtherThanFeet = true;
                }
            }
        }
        return touchingGroundOtherThanFeet;
    }

    private int CheckNumberOfFeetTouchingGround()
    {
        int feetTouchingGround = 0;
        foreach (Rigidbody seg in body.sensedSegments)
        {
            if (seg.GetComponent<TouchingGround>().touching == true)
            {
                if (seg.name == "lower")
                {
                    feetTouchingGround += 1;
                }
            }
        }
        return feetTouchingGround;
    }
}