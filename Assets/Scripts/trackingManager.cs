using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using HoloProxies.Engine;
using HoloProxies.Utils;

/// <summary>
/// The trackingManager implements the main loop that starts off the main engine and runs the process frame function.
/// It also takes user input to start tracking, etc.
/// File from: demo.cpp in Start() and UIEngine.cpp in Update()
/// </summary>
public class trackingManager : MonoBehaviour
{

    private const int histogramNBins = 16;
    private const int numTrackingObj = 1;

    private coreEngine engine;
    private int[] engineSettings = { histogramNBins, numTrackingObj };
    private MultiSourceManager msm;

    public GameObject sourceManager;

    // Use this for initialization
    void Start()
    {
        // Get the source manager from which we get out RGBD pixels.
        msm = sourceManager.GetComponent<MultiSourceManager>();
        // Start off the main engine
        engine = new coreEngine( engineSettings, new Vector2i( msm.DepthWidth, msm.DepthHeight ), new Vector2i( msm.ColorWidth, msm.ColorHeight ) );

        float[] poses = { 0.0f, 0.0f, 0.8f, -Mathf.PI, 0, 0 };
        engine.trackingState.setHFromParam( poses, 0 );

    }

    // Update is called once per frame
    void Update()
    {

    }
}
