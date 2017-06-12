using System.Collections;
using System.Collections.Generic;
using Windows.Kinect;
using System.IO;
using UnityEngine;

public class HistogramViewer : MonoBehaviour
{

    public GameObject Tracker;
    private trackingManager _TrackerManager;
    private Texture2D tex;
    private ColorSpacePoint[] colorPoints;

    // Use this for initialization
    void Start()
    {
        gameObject.GetComponent<Renderer>().material.SetTextureScale( "_MainTex", new Vector2( -1, 1 ) );
        _TrackerManager = Tracker.GetComponent<trackingManager>();
    }

    // Update is called once per frame
    void Update()
    {
        gameObject.GetComponent<Renderer>().material.mainTexture = _TrackerManager.getHistogramTexture(); ;
    }
}
