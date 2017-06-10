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
public class TcpClient : MonoBehaviour
{
    // Set the IP address of the server
    public string ServerIP;

    // Port that you want to use
    public int ConnectionPort = 20602;

    /// <summary>
    /// variables used to store interpolated polynomial coefficients
    /// </summary>
    public double ax = 1, bx = 0, cx = 0;
    public double ay = 1, by = 0, cy = 0;

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

    public void Start()
    {

        OpenConnection();
        ax = 0; bx = 0; cx = 0;
        ay = 0; by = 0; cy = 0;
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
            socketDataWriter = new DataWriter(socketConnection.OutputStream);
            socketDataReader = new DataReader(socketConnection.InputStream);
            socketDataReader.UnicodeEncoding = UnicodeEncoding.Utf8;
            socketDataReader.ByteOrder = ByteOrder.LittleEndian;

            //Begin reading data in the input stream async
            DataReaderLoadOperation outstandingRead = socketDataReader.LoadAsync(65); // Try to load 53 bytes
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
            Debug.Log("got something");
            // If we succeeded, clear the sending flag so we can send another mesh
            var receivedStrings = "";
            receivedStrings += socketDataReader.ReadString(65);  // Read the 53 bytes of polynomial coefficients data into a string
            // Process the received string
            string[] coeff = receivedStrings.Split(',');         // Split the string into each coefficient
            if (coeff.Length == 6)
            {
                ax = 1000*(double.Parse(coeff[0])-1);            // Substract the offset of 1 and scale from mm to meters
                bx = 1000*(double.Parse(coeff[1])-1);
                cx = 1000*(double.Parse(coeff[2])-1);
                ay = 1000*(double.Parse(coeff[3])-1);
                by = 1000*(double.Parse(coeff[4])-1);
                cy = 1000*(double.Parse(coeff[5])-1);
                interlock = 1;
            } else
            {
                Debug.Log("Received something else");
                Debug.Log(receivedStrings);
            }
        
            //restart reading data in the input stream async
            DataReaderLoadOperation outstandingRead = socketDataReader.LoadAsync(65);
            AsyncOperationCompletedHandler<uint> aoch = new AsyncOperationCompletedHandler<uint>(DataReadHandler);
            outstandingRead.Completed = aoch;
        }
    }
#endif
}