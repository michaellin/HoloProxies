using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class KinectColorViewer : MonoBehaviour {

	public GameObject Tracker;
	private trackingManager _TrackerManager;

	// Use this for initialization
	void Start () {
		_TrackerManager = trackingManager.GetComponent<trackingManager>();
		gameObject.GetComponent<Renderer>().material.SetTextureScale("_MainTex", new Vector2(-1, 1));
	}
	
	// Update is called once per frame
	void Update () {

		if (trackingManager == null) {
			return;
		}
		if (_TrackerManager == null) {
			return;
		}

		gameObject.GetComponent<Renderer>().material.mainTexture = _TrackerManager.frame.ColorTexture;
		
	}
}
