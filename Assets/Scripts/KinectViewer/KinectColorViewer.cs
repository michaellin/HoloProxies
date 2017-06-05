using System.Collections;
using System.Collections.Generic;
using Windows.Kinect;
using System.IO;
using UnityEngine;

public class KinectColorViewer : MonoBehaviour {

	public GameObject Tracker;
	private trackingManager _TrackerManager;
    private Texture2D tex;
    private ColorSpacePoint[] colorPoints;
    private int width, height;

    // Use this for initialization
    void Start () {
		gameObject.GetComponent<Renderer>().material.SetTextureScale("_MainTex", new Vector2(-1, 1));
        _TrackerManager = Tracker.GetComponent<trackingManager>();
        width = _TrackerManager.getFrameWidth();
        height = _TrackerManager.getFrameHeight();
    }

    // Update is called once per frame
    void Update () {
        tex = _TrackerManager.getColorTexture();
        // colorPoints = _TrackerManager.getColorPoints();
        gameObject.GetComponent<Renderer>().material.mainTexture = tex;
	}
}
