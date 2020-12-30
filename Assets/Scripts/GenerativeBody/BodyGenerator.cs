using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class BodyGenerator : MonoBehaviour
{
    [Header("Physical attributes")]
    public float baseScale = 1f;
    public float density = 1f;

    [Header("Joint drive settigs")]
    public float positionSpringPower = 3000;
    public float positionDamperMultiplier = 2; //how much stronger should the position damper be than max spring force
    public float maxSpringForce = 30;
    public bool scaleForceByCrossSection = false;

    [Header("Joint limit defaults")]
    public float ballHighAngularXLimitDefault = 60f;
    public float ballLowAngularXLimitDefault = 60f;
    public float ballAngularYimitDefault = 30f;
    public float ballAngularZimitDefault = 10f;

    public float kneeAngularXLimitDefault = 30f;
    public float ankleAngularXLimitDefault = 20f;
    public float torsoAngularXLimitDefault = 10f;




    private float moveTime;
    private float minJointSettlingTime = 0.5f; //seconds
    private bool jointsSettled = false;

    [Header("Limb prefabs")]
    public GameObject limbSegmentPrefab;
    public GameObject jointPrefab;
    public GameObject bodyPrefab;
    public GameObject headPrefab;
    public GameObject tailPrefab;

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

    [Header("Default body part scales")]
    [SerializeField]
    protected Dictionary<string, Vector3> defaultScales = new Dictionary<string, Vector3>()
    {
        { "upper" ,  new Vector3(0.5f, 0.5f, 1f) },
        { "middle" , new Vector3(0.3f, 0.3f, 1f) },
        { "lower" , new Vector3(0.2f, 0.2f, 1f) },
        { "chest" , new Vector3(1.5f, 1f, 1.5f) },
        { "hips" , new Vector3(1.5f, 1f, 1.5f) }
    };

    public virtual void Awake()
    {
        moveTime = Time.time;
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
        if (Time.time - moveTime < minJointSettlingTime || !jointsSettled)
        {
            jointsSettled = true;
            foreach (Rigidbody rBody in this.GetComponentsInChildren<Rigidbody>())
            {
                //if any body parts are still flailing
                if (rBody.velocity.magnitude > 3)
                {
                    jointsSettled = false;
                }

                rBody.velocity = Vector3.zero;
                rBody.angularVelocity = Vector3.zero;
                LocalizeRigidbody(rBody, baseScale * 5);


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

    public void MoveChest(Vector3 position)
    {
        chest.MovePosition(position);
        moveTime = Time.time;
        jointsSettled = false;
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
        ScaleSegment(chest, 1.0f);

        hips = PrefabBodySegment(bodyPrefab);
        hips.name = "hips";
        ScaleSegment(hips, 1.0f);

        ConfigurableJoint chestXhips = ConnectWithJoint(chest, hips, "hinge");
        SetJointXLimits(chestXhips, -torsoAngularXLimitDefault, torsoAngularXLimitDefault);

        List<Rigidbody> torso = new List<Rigidbody>();

        torso.Add(chest);
        torso.Add(hips);

        this.torso.Add(chest);
        this.torso.Add(hips);

        sensedSegments.Add(chest);
        sensedSegments.Add(hips);

        return torso;
    }

    public void ScaleSegment(Rigidbody segment, float relativeScale)
    {
        ConfigurableJoint cJoint = segment.GetComponent<ConfigurableJoint>();
        Vector3 originalAnchor = Vector3.zero;
        Vector3 originalConnectedBodyAnchor = Vector3.zero;

        //store joint anchors
        if (cJoint != null)
        {
            originalAnchor = cJoint.anchor;
            originalConnectedBodyAnchor = cJoint.connectedAnchor;
        }
        if (defaultScales.ContainsKey(segment.name))
        {
            segment.transform.localScale = defaultScales[segment.name] * baseScale * relativeScale;
            SetMassByVolume(segment);
        }
        else
        {
            Debug.LogWarning("Attempting to scale segment type without default scale: " +segment.name );
        }

        //reset joint anchors after scaling
        if (cJoint != null)
        {
            cJoint.anchor = originalAnchor;
            cJoint.connectedAnchor = originalConnectedBodyAnchor;
        }
    }

    public void ApplyDefaultScales()
    {
        foreach (Rigidbody r in GetComponentsInChildren<Rigidbody>())
        {
            ScaleSegment(r, 1.0f);
        }
    }


    protected List<Rigidbody> CreateTwoJointLimb()
    {
        Rigidbody upperRbody = PrefabBodySegment(limbSegmentPrefab);
        Rigidbody middleRbody = PrefabBodySegment(limbSegmentPrefab);
        Rigidbody lowerRbody = PrefabBodySegment(limbSegmentPrefab);

        upperRbody.name = "upper";
        middleRbody.name = "middle";
        lowerRbody.name = "lower";

        ScaleSegment(upperRbody, 1.0f);
        ScaleSegment(middleRbody, 1.0f);
        ScaleSegment(lowerRbody, 1.0f);

        middleRbody.transform.rotation = Quaternion.Euler(new Vector3(-45f, 0f, 0f));
        lowerRbody.transform.rotation = Quaternion.Euler(new Vector3(45f, 0f, 0f));

        ConfigurableJoint knee = ConnectWithJoint(middleRbody, upperRbody, "hinge");
        ConfigurableJoint ankle = ConnectWithJoint(lowerRbody, middleRbody, "hinge");

        SetJointXLimits(knee, -kneeAngularXLimitDefault, kneeAngularXLimitDefault);
        SetJointXLimits(ankle, -ankleAngularXLimitDefault, ankleAngularXLimitDefault);

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

        SetJointAllLimits(j, ballLowAngularXLimitDefault, ballHighAngularXLimitDefault, ballAngularYimitDefault, ballAngularZimitDefault);
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

        SetJointXLimits(j, ballLowAngularXLimitDefault, ballHighAngularXLimitDefault);
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
    public void DriveJoint(ConfigurableJoint cJoint, Vector3 direction, float effort)
    {
        Vector3 constrainedDirection = Vector3.zero;
        constrainedDirection.x = Mathf.Lerp(cJoint.lowAngularXLimit.limit, cJoint.highAngularXLimit.limit, direction.x);
        constrainedDirection.y = Mathf.Lerp(-cJoint.angularYLimit.limit, cJoint.angularYLimit.limit, direction.y);
        constrainedDirection.z = Mathf.Lerp(-cJoint.angularZLimit.limit, cJoint.angularZLimit.limit, direction.z);

        cJoint.targetRotation = Quaternion.Euler(constrainedDirection);
        
        float newForce = effort * maxSpringForce;

        if (scaleForceByCrossSection)
        {
            newForce *= baseScale * baseScale;
        }

        SetJointDriveMaximumForce(cJoint, newForce);
    }


    public void SetJointDriveMaximumForce(ConfigurableJoint cJoint, float newMaximumForce)
    {
        JointDrive jDrive = new JointDrive();
        jDrive.maximumForce = newMaximumForce;
        jDrive.positionSpring = this.positionSpringPower;
        jDrive.positionDamper = this.maxSpringForce * positionDamperMultiplier;
        if (scaleForceByCrossSection)
        {
            jDrive.positionDamper *= baseScale * baseScale;
        }
        cJoint.slerpDrive = jDrive;
    }

    protected void SetMassByVolume(Rigidbody rb)
    {
        rb.mass = rb.transform.localScale.x * rb.transform.localScale.y * rb.transform.localScale.z * density;
    }


    protected void SetAllMassesByVolume()
    {
        foreach (Rigidbody rb in GetComponentsInChildren<Rigidbody>())
        {
            SetMassByVolume(rb);
        }
    }


}
