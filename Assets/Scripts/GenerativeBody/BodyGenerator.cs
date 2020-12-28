using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class BodyGenerator : MonoBehaviour
{
    
    public float baseScale = 1f;
    public float density = 1f;
    public float highAngularXLimitDefault = 60f;
    public float lowAngularXLimitDefault = 60f;
    public float angularYimitDefault = 30f ;
    public float angularZimitDefault = 10f;

    private float spawnTime;
    private float minJointSettlingTime = 0.5f; //seconds
    private bool jointsSettled = false;

    public GameObject limbSegmentPrefab;
    public GameObject jointPrefab;
    public GameObject bodyPrefab;

    [HideInInspector]
    public Rigidbody chest;
    [HideInInspector]
    public Rigidbody hips;

    [HideInInspector]
    public List<Rigidbody> limbs;
    [HideInInspector]
    public List<Rigidbody> torso;
    [HideInInspector]
    public List<ConfigurableJoint> motorJoints = new List<ConfigurableJoint>();
    [HideInInspector]
    public List<Rigidbody> sensedSegments = new List<Rigidbody>();


    public virtual void Awake()
    {
        spawnTime = Time.time;
        SetUpBody();
        SetAllMassesByVolume();
        StoreAllMotorJoints();
    }

    protected virtual void SetUpBody()
    {
        //children will impliment me
    }

    private void FixedUpdate()
    {
        if (Time.time - spawnTime < minJointSettlingTime || !jointsSettled)
        {
            jointsSettled = true;
            foreach (Rigidbody rBody in this.GetComponentsInChildren<Rigidbody>())
            {
                //if any body parts are still flailing
                if (rBody.velocity.magnitude > 1)
                {
                    jointsSettled = false;
                }
                
                rBody.velocity = Vector3.zero;
                rBody.angularVelocity = Vector3.zero;
                LocalizeRigidbody(rBody, baseScale*5);
            }

            //Debug.Log(jointsSettled);
            chest.isKinematic = true;
            chest.MovePosition(transform.position);
            chest.MoveRotation(transform.rotation);
        }
        else
        {
            chest.isKinematic = false;
        }
    }

    private void LocalizeRigidbody(Rigidbody r, float limit)
    {
        if (r.transform.localPosition.magnitude > limit)
        {
            r.MovePosition(this.transform.position);
        }
    }

    protected void ConnectAngledLimb(Rigidbody segment, Rigidbody connectedBody, Vector3 angles, Vector3 anchor)
    {
        segment.transform.rotation = Quaternion.Euler(angles);
        ConfigurableJoint joint = ConnectWithJoint(segment, connectedBody, "ball");
        joint.connectedAnchor = anchor;
    }

    protected void StoreAllMotorJoints()
    {
        motorJoints = new List<ConfigurableJoint>();

        ConfigurableJoint joint;

        foreach (Rigidbody limb in limbs)
        {
            joint = limb.GetComponentInChildren<ConfigurableJoint>();
            if (joint != null)
            {
                motorJoints.Add(joint);
            }
        }
        foreach (Rigidbody seg in torso)
        {
            joint = seg.GetComponentInChildren<ConfigurableJoint>();
            if (joint != null)
            {
                motorJoints.Add(joint);
            }
        }
    }

    protected List<Rigidbody> CreateOneJointTorso()
    {
        chest = PrefabBodySegment(bodyPrefab);
        chest.name = "chest";
        chest.transform.localScale = new Vector3(baseScale * 2f, baseScale * 1f, baseScale * 1.5f);

        hips = PrefabBodySegment(bodyPrefab);
        hips.name = "hips";
        hips.transform.localScale = new Vector3(baseScale * 2f, baseScale * 1f, baseScale * 1.5f);

        ConfigurableJoint chestXhips = ConnectWithJoint(chest, hips, "hinge");
        SetJointXLimits(chestXhips, -30, 10);

        List<Rigidbody> torso = new List<Rigidbody>();

        torso.Add(chest);
        torso.Add(hips);


        this.torso.Add(chest);
        this.torso.Add(hips);

        sensedSegments.Add(chest);
        sensedSegments.Add(hips);

        return torso;


    }


    protected List<Rigidbody>CreateTwoJointLimb()
    {
        Rigidbody upperRbody = PrefabBodySegment(limbSegmentPrefab);
        Rigidbody middleRbody = PrefabBodySegment(limbSegmentPrefab);
        Rigidbody lowerRbody = PrefabBodySegment(limbSegmentPrefab);

        upperRbody.name = "upper";
        middleRbody.name = "middle";
        lowerRbody.name = "lower";

        upperRbody.transform.localScale = new Vector3(baseScale * 0.5f, baseScale * 0.5f, baseScale);
        middleRbody.transform.localScale = new Vector3(baseScale * 0.3f, baseScale * 0.3f, baseScale);
        lowerRbody.transform.localScale = new Vector3(baseScale * 0.25f, baseScale * 0.25f, baseScale);

        middleRbody.transform.rotation = Quaternion.Euler(new Vector3(-60f, 0f, 0f));
        lowerRbody.transform.rotation = Quaternion.Euler(new Vector3(60f, 0f, 0f));

        ConnectWithJoint(middleRbody, upperRbody, "hinge");
        ConnectWithJoint(lowerRbody, middleRbody, "hinge");

        List<Rigidbody> limb = new List<Rigidbody>();

        limb.Add(upperRbody);
        limb.Add(middleRbody);
        limb.Add(lowerRbody);
        
        limbs.Add(upperRbody);
        limbs.Add(middleRbody);
        limbs.Add(lowerRbody);

        sensedSegments.Add(upperRbody);
        sensedSegments.Add(middleRbody);
        sensedSegments.Add(lowerRbody);

        return  limb;
    }

    protected Rigidbody primitiveBodySegment()
    {
        GameObject segment = GameObject.CreatePrimitive(PrimitiveType.Cube);
        
        Rigidbody segmentRbody = segment.AddComponent<Rigidbody>();
        segment.transform.parent = this.transform;
        segment.transform.localPosition = Vector3.zero;
        segment.AddComponent(System.Type.GetType("TouchingGround" + ",Assembly-CSharp"));

        return segmentRbody;
    }



    protected Rigidbody PrefabBodySegment(GameObject prefab)
    {
        GameObject segment = Instantiate(prefab, Vector3.zero, Quaternion.identity) ;

        Rigidbody segmentRbody = segment.AddComponent<Rigidbody>();
        segment.transform.parent = this.transform;
        segment.transform.localPosition = Vector3.zero;
        segment.AddComponent(System.Type.GetType("TouchingGround" + ",Assembly-CSharp"));

        return segmentRbody;
    }

    protected ConfigurableJoint ConnectWithJoint(Rigidbody body, Rigidbody connectedBody, string jointType)
    {
        ConfigurableJoint joint = body.gameObject.AddComponent<ConfigurableJoint>() as ConfigurableJoint;

        joint.connectedBody = connectedBody;

        //default anchors
        joint.autoConfigureConnectedAnchor = false;
        joint.connectedAnchor = new Vector3(0f, 0f, -0.5f);
        joint.anchor = new Vector3(0f, 0f, 0.5f);

        joint.rotationDriveMode = RotationDriveMode.Slerp;

        if (jointType == "ball")
        {
            SetUpBallJoint(joint);
        }

        if (jointType == "hinge")
        {
            SetUpHingeJoint(joint);
        }

        return joint;

    }

    protected void SetUpBallJoint(ConfigurableJoint j)
    {
        //ball joint
        j.xMotion = ConfigurableJointMotion.Locked;
        j.yMotion = ConfigurableJointMotion.Locked;
        j.zMotion = ConfigurableJointMotion.Locked;
        j.angularXMotion = ConfigurableJointMotion.Limited;
        j.angularYMotion = ConfigurableJointMotion.Limited;
        j.angularZMotion = ConfigurableJointMotion.Limited;

        SetJointAllLimits(j, lowAngularXLimitDefault, highAngularXLimitDefault, angularYimitDefault, angularZimitDefault);
    }


    protected void SetUpHingeJoint(ConfigurableJoint j)
    {
        //ball joint
        j.xMotion = ConfigurableJointMotion.Locked;
        j.yMotion = ConfigurableJointMotion.Locked;
        j.zMotion = ConfigurableJointMotion.Locked;
        j.angularXMotion = ConfigurableJointMotion.Limited;
        j.angularYMotion = ConfigurableJointMotion.Locked;
        j.angularZMotion = ConfigurableJointMotion.Locked;

        SetJointXLimits(j, lowAngularXLimitDefault, highAngularXLimitDefault);
    }

    protected void SetJointXLimits(ConfigurableJoint j, float lowXlimit, float highXlimit)
    {
        //can't set joint limits directly, so we need to create new objects for replacement
        SoftJointLimit lowAngularXLimit = new SoftJointLimit();
        lowAngularXLimit.limit = lowXlimit;
        SoftJointLimit highAngularXLimit = new SoftJointLimit();
        highAngularXLimit.limit = highXlimit;

        //replace limits
        j.lowAngularXLimit = lowAngularXLimit;
        j.highAngularXLimit = highAngularXLimit;
    }

    protected void SetJointAllLimits(ConfigurableJoint j, float lowXlimit, float highXlimit, float yLimit, float zLimit)
    {
        SetJointXLimits(j, lowXlimit, highXlimit);

        //can't set joint limits directly, so we need to create new objects for replacement
        SoftJointLimit angularYLimit = new SoftJointLimit();
        angularYLimit.limit = yLimit;
        SoftJointLimit angularZLimit = new SoftJointLimit();
        angularZLimit.limit = zLimit;

        //replace limits
        j.angularYLimit = angularYLimit;
        j.angularZLimit = angularZLimit;

        
    }


    protected void SetAllMassesByVolume()
    {
        foreach (Rigidbody rb in GetComponentsInChildren<Rigidbody>())
        {
            rb.mass = rb.transform.localScale.x* rb.transform.localScale.y * rb.transform.localScale.z * density;
        }
    }


}
