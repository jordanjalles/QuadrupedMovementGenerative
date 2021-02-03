using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MotionAgentController : MonoBehaviour
{

    [SerializeField]
    private MotionAgent motionAgent;

    [SerializeField]
    private Camera mainCamera;

    [SerializeField]
    private float turnSpeed;

    private Transform virtualTarget;
    private Transform virtualTargetPivot;

    private float lastTippedOverTime;
    private float minGetUpTime = 1f;


    void Awake()
    {
        
    }

    void Start()
    {
        SetUpVirtualTarget();
    }

    void Update()
    {
        if (Input.GetAxis("Vertical") > 0)
        {
            UpdateModelState("forward");
        }
        else
        {
            UpdateModelState("stay");
        }

        //update target position
        UpdateRig(Input.GetAxis("Horizontal"));

    }

    void UpdateModelState(string input)
    {

        if (motionAgent.CheckTippedOver()) { 
            motionAgent.ChangeMode(MotionAgent.RewardMode.StandUp);
            lastTippedOverTime = Time.time;
        }
        else
        {
            if (Time.time - lastTippedOverTime > motionAgent.standUpSuccessSeconds)
            {
                if (input == "forward")
                {
                    motionAgent.ChangeMode(MotionAgent.RewardMode.SeekTarget);
                }
                if (input == "stay")
                {
                    motionAgent.ChangeMode(MotionAgent.RewardMode.StayStanding);
                }
            }
        }



    }



    void UpdateRig(float horizontal)
    {
        virtualTargetPivot.transform.Rotate(0, horizontal*turnSpeed, 0);

        virtualTargetPivot.position = Vector3.Lerp(virtualTargetPivot.position, motionAgent.core.transform.position, 0.1f);
        mainCamera.transform.localPosition = new Vector3(0, 1, -3f);
        virtualTarget.localPosition = new Vector3(0, 0, 5f);

        mainCamera.transform.LookAt(virtualTarget);
    }

    void SetUpVirtualTarget()
    {
        Debug.Log("yo");
        virtualTarget = new GameObject().transform;
        virtualTarget.name = "virtualTarget";
        virtualTarget.gameObject.SetActive(true);
        

        virtualTargetPivot = new GameObject().transform;
        virtualTargetPivot.name = "virtualTargetPivot";
        //virtualTargetPivot.position = motionAgent.core.transform.position;

        virtualTarget.parent = virtualTargetPivot;

        
        mainCamera.transform.parent = virtualTargetPivot;



        motionAgent.target = virtualTarget;
    }
}
