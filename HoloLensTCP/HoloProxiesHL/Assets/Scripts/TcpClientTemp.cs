using System.Collections.Generic;
using UnityEngine;
using UnityEngine.VR.WSA.Input;
using System.Globalization;

#if !UNITY_EDITOR   // Need this so that these libraries are only imported in the HoloLens
using Windows.Networking.Sockets;
using Windows.Storage.Streams;
using Windows.Networking;
using Windows.Foundation;
#endif

/// <summary>
/// Class for establishing remote tcp connection and streaming data.
/// </summary>
public class TcpClientTemp : MonoBehaviour
{
    // Set the IP address of the server
    public string ServerIP;

    // Port that you want to use
    public int ConnectionPort = 20602;

    public GameObject reference;

    public int interlock = 0;  // This variable is used to protect read wright issues. However, might be unnecessary

#if !UNITY_EDITOR

    /// <summary>
    /// Tracks the network connection to the remote machine were the server runs.
    /// </summary>
    private StreamSocket socketConnection;

    /// <summary>
    /// Flag to indicate whether socket is open for streaming data
    /// </summary>
    private bool socketOpen = false;

    /// <summary>
    /// If socket is open this will be the object used to send data
    /// </summary>
    private DataWriter socketDataWriter;

    /// <summary>
    /// If socket is open this will be the object used to receive data
    /// </summary>
    private DataReader socketDataReader;

    /// <summary>
    /// Flag to indicate whether a connection attempt has been deferred
    /// </summary>
    private bool deferredConnection;

    private ObjectAlign objAlg;

    private Matrix4x4 H;


    public void Start()
    {

        OpenConnection();
        H = new Matrix4x4();
        objAlg = reference.GetComponent<ObjectAlign>();

    }

    public void Update()
    {
        if (deferredConnection)
        {
            deferredConnection = false;
            Invoke("OpenConnection", 2); // Will call OpenConnection again after 2 seconds
        }
    }

    /// <summary>
    /// opens a socket connection as client.
    /// </summary>
    private void OpenConnection()
    {
        // Setup a connection to the server.
        HostName networkHost = new HostName(ServerIP.Trim());
        socketConnection = new StreamSocket();

        Debug.Log("Attempting to connect");

        // Connections are asynchronous. This callback function 'SocketOpenedHandler' will be called when the connection is established
        IAsyncAction outstandingAction = socketConnection.ConnectAsync(networkHost, ConnectionPort.ToString());
        AsyncActionCompletedHandler aach = new AsyncActionCompletedHandler(SocketOpenedHandler);
        outstandingAction.Completed = aach;
    }

    /// <summary>
    /// Called when a connection attempt complete, successfully or not.  
    /// </summary>
    /// <param name="asyncInfo">Data about the async operation.</param>
    /// <param name="status">The status of the operation.</param>
    public void SocketOpenedHandler(IAsyncAction asyncInfo, AsyncStatus status)
    {
        // Status completed is successful.
        if (status == AsyncStatus.Completed)
        {
            socketOpen = true;
            Debug.Log("Connected! Ready to send and receive data");
            //socketDataWriter = new DataWriter(socketConnection.OutputStream);
            socketDataReader = new DataReader(socketConnection.InputStream);
            socketDataReader.UnicodeEncoding = UnicodeEncoding.Utf8;
            socketDataReader.ByteOrder = ByteOrder.LittleEndian;

            //Begin reading data in the input stream async
            DataReaderLoadOperation outstandingRead = socketDataReader.LoadAsync(4*16); // Try to load 53 bytes
            AsyncOperationCompletedHandler<uint> aoch = new AsyncOperationCompletedHandler<uint>(DataReadHandler);
            outstandingRead.Completed = aoch;
        }
        else
        {
            Debug.Log("Failed to establish connection. Error Code: " + asyncInfo.ErrorCode);
            // In the failure case we'll requeue the data and wait before trying again.
            socketConnection.Dispose();
            // Setup a callback function that will retry connection after 2 seconds
            if (!socketOpen) // Redundant but to be safe
            {
                deferredConnection = true; // Defer the connection attempt
            }
        }
    }

    /// <summary>
    /// Called when receive data has completed.
    /// </summary>
    /// <param name="operation">Data about the async operation.</param>
    /// <param name="status">The status of the operation.</param>
    public void DataReadHandler(IAsyncOperation<uint> operation, AsyncStatus status)
    {
        // If we failed, requeue the data and set the deferral time.
        if (status == AsyncStatus.Error)
        {
            // didn't load data
            Debug.Log("Failed to load new data");
        }
        else
        {
            //Debug.Log("read something");
            // If we succeeded read into H. Order matters. Row major.
            //var a = "";
            //a += socketDataReader.ReadString(1);
            H.m00 = socketDataReader.ReadSingle();
            H.m01 = socketDataReader.ReadSingle();
            H.m02 = socketDataReader.ReadSingle();
            H.m03 = socketDataReader.ReadSingle();
            H.m10 = socketDataReader.ReadSingle();
            H.m11 = socketDataReader.ReadSingle();
            H.m12 = socketDataReader.ReadSingle();
            H.m13 = socketDataReader.ReadSingle();
            H.m20 = socketDataReader.ReadSingle();
            H.m21 = socketDataReader.ReadSingle();
            H.m22 = socketDataReader.ReadSingle();
            H.m23 = socketDataReader.ReadSingle();
            H.m30 = socketDataReader.ReadSingle();
            H.m31 = socketDataReader.ReadSingle();
            H.m32 = socketDataReader.ReadSingle();
            H.m33 = socketDataReader.ReadSingle();
            
            objAlg.updatePoseMatrix( H );
            
            interlock = 1;


            //restart reading data in the input stream async
            DataReaderLoadOperation outstandingRead = socketDataReader.LoadAsync(4*16);
            AsyncOperationCompletedHandler<uint> aoch = new AsyncOperationCompletedHandler<uint>(DataReadHandler);
            outstandingRead.Completed = aoch;
        }
    }
#endif

}