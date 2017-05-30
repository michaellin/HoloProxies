using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;

using HoloProxies.Objects;

public class unitTests : MonoBehaviour {

	// Use this for initialization
	void Start () {
        objectPose pose1 = new objectPose();
        Assert.AreEqual( pose1.getH(), new Matrix4x4() );
        Assert.AreEqual( pose1.getInvH(), new Matrix4x4() );
        Matrix4x4 testM = new Matrix4x4();
        
    }
	
	// Update is called once per frame
	void Update () {
		
	}
}
