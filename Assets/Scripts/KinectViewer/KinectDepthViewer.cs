using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KinectDepthViewer : MonoBehaviour {

    public GameObject Tracker;
    private trackingManager _TrackerManager;

    // Use this for initialization
    void Start () {
        gameObject.GetComponent<Renderer>().material.SetTextureScale( "_MainTex", new Vector2( -1, 1 ) );
        _TrackerManager = Tracker.GetComponent<trackingManager>();
    }
	
	// Update is called once per frame
	void Update () {
        gameObject.GetComponent<Renderer>().material.mainTexture = _TrackerManager.getDepthTexture();
    }
}
