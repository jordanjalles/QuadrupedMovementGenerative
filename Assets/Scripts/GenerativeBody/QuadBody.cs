using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class QuadBody : BodyGenerator
{

    [HideInInspector]
    public List<Rigidbody> frontLimbs;
    [HideInInspector]
    public List<Rigidbody> backLimbs;
    [HideInInspector]
    public Rigidbody head;
    [HideInInspector]
    public Rigidbody tail;

    /*
    [Range(0.5f, 1f)]
    public float backLegsSizeMin;
    [Range(1f, 2f)]
    public float backLegsSizeMax;
    */

    protected override void SetUpBody()
    {
        CreateOneJointTorso();

        Rigidbody limbFL = CreateTwoJointLimb()[0];
        Rigidbody limbFR = CreateTwoJointLimb()[0];
        Rigidbody limbBL = CreateTwoJointLimb()[0];
        Rigidbody limbBR = CreateTwoJointLimb()[0];

        ConnectAngledLimb(limbFL, hips, new Vector3(-90f, 0f, 0f), new Vector3(0.4f, 0f, 0.4f));
        ConnectAngledLimb(limbFR, hips, new Vector3(-90f, 0f, 0f), new Vector3(-0.4f, 0f, 0.4f));

        ConnectAngledLimb(limbBL, chest, new Vector3(-90f, 0f, 0f), new Vector3(0.4f, 0f, -0.4f));
        ConnectAngledLimb(limbBR, chest, new Vector3(-90f, 0f, 0f), new Vector3(-0.4f, 0f, -0.4f));

        
        head = PrefabBodySegment(headPrefab);
        Joint neck = ConnectWithJoint(head, chest,  "fixed");
        head.drag = 0f;
        head.angularDrag = 0f;
        neck.connectedMassScale = 0.00001f;
        

        
        tail = PrefabBodySegment(tailPrefab);
        Joint tailJoint = ConnectWithJoint(hips, tail,  "fixed");
        tail.drag = 0f;
        tail.angularDrag = 0f;
        tailJoint.massScale = 0.00001f;
        

    }
}
