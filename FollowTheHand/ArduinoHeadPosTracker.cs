#if WindowsARpoise
using System;
using System.Globalization;
using System.IO.Ports;
using UnityEngine;

public class ArduinoHeadPosTracker : MonoBehaviour
{
    [Header("Serial")]
    public string portName = "COM6";
    public int baudRate = 115200;
    [Header("Total Gaze")]
    public float maxGazeX = 40.0f;
    public float maxGazeY = 50.0f;
    [Header("Position Smoothing")]
    public float positionSmoothTime = 0.20f;
    public float maxPositionSpeed = 250.0f;
    [Header("Eyes")]
    public float eyeSpeed = 12.0f;
    [Header("Head")]
    public float maxHeadX = 12.0f;
    public float maxHeadY = 16.7f;
    public float headDelay = 0.30f;
    public float headSpeed = 2.5f;
    public float headMovementThreshold = 5.0f;
    [Header("Sensor Geometry")]
    public float sensorCenterX = 14.0f;
    public float sensorCenterY = 26.0f;
    public float sensorHalfWidth = 14.0f;
    public float sensorHalfHeight = 26.0f;
    [Header("Debug")]
    public bool connected;
    public bool hasPosition;
    public bool foundRightEye;
    public bool foundLeftEye;
    public int receivedPositions;
    public Vector3 handPosition;
    public Vector3 smoothedPosition;
    public float horizontalInput;
    public float verticalInput;
    public float desiredGazeX;
    public float desiredGazeY;
    public float currentHeadX;
    public float currentHeadY;
    public float currentEyeX;
    public float currentEyeY;
    public Vector3 rightEyeRotation;
    public Vector3 leftEyeRotation;
    public string lastPositionLine;
    public string lastRawLine;
    private Transform rightEye;
    private Transform leftEye;
    private SerialPort serialPort;
    private string serialBuffer = "";
    private Quaternion headNeutralRotation;
    private Quaternion rightEyeNeutralRotation;
    private Quaternion leftEyeNeutralRotation;
    private Vector3 positionVelocity;
    private float headDelayTimer;
    private float previousDesiredGazeX;
    private float previousDesiredGazeY;
    private bool waitingForHead;
    private int lastPosLogSecond = -1;
    private int lastSensorLogSecond = -1;

    void Start()
    {
        Debug.Log("ArduinoHeadPosTracker started");
        rightEye = transform.Find("RightEye");
        leftEye = transform.Find("LeftEye");
        foundRightEye = rightEye != null;
        foundLeftEye = leftEye != null;
        if (rightEye == null)
        {
            Debug.LogError("Could not find child: RightEye");
        }
        else
        {
            rightEyeNeutralRotation = rightEye.localRotation;
        }
        if (leftEye == null)
        {
            Debug.LogError("Could not find child: LeftEye");
        }
        else
        {
            leftEyeNeutralRotation = leftEye.localRotation;
        }
        headNeutralRotation = transform.localRotation;
        OpenSerialPort();
    }

    void OpenSerialPort()
    {
        try
        {
            serialPort = new SerialPort(portName, baudRate);
            serialPort.ReadTimeout = 100;
            serialPort.WriteTimeout = 100;
            serialPort.DtrEnable = true;
            serialPort.RtsEnable = true;
            serialPort.Open();
            connected = true;
            Debug.Log("Arduino connected on " + portName + ", port IsOpen: " + serialPort.IsOpen + ", baudRate: " + baudRate + " baud");
        }
        catch (Exception e)
        {
            connected = false;
            Debug.LogError("Could not open Arduino serial port: " + e.Message);
        }
    }

    void Update()
    {
        ReadSerial();
        if (!hasPosition)
        {
            return;
        }
        float smoothTime = Mathf.Max(0.01f, positionSmoothTime);
        smoothedPosition = Vector3.SmoothDamp(smoothedPosition, handPosition, ref positionVelocity, smoothTime, Mathf.Max(1.0f, maxPositionSpeed), Time.deltaTime);
        CalculateDesiredGaze();
        UpdateHeadDelay();
        UpdateHead();
        UpdateEyes();
        UpdateDebugValues();
    }

    void ReadSerial()
    {
        if (serialPort == null || !serialPort.IsOpen)
        {
            return;
        }
        try
        {
            string incoming = serialPort.ReadExisting();
            if (incoming.Length == 0)
            {
                return;
            }
            serialBuffer += incoming;
            int newlineIndex = serialBuffer.IndexOf('\n');
            while (newlineIndex >= 0)
            {
                string line = serialBuffer.Substring(0, newlineIndex).Trim();
                serialBuffer = serialBuffer.Substring(newlineIndex + 1);
                if (line.Length > 0)
                {
                    lastRawLine = line;
                    ParseSerialLine(line);
                }
                newlineIndex = serialBuffer.IndexOf('\n');
            }
            if (serialBuffer.Length > 2048)
            {
                serialBuffer = "";
                Debug.LogWarning("Serial input buffer reset: incomplete line too long");
            }
        }
        catch (Exception e)
        {
            Debug.LogError("Serial read error: " + e.Message);
        }
    }

    void ParseSerialLine(string line)
    {
        int now = DateTime.Now.Second;
        if (!line.StartsWith("POS,"))
        {
            if (line.StartsWith("TL=") && now != lastSensorLogSecond)
            {
                Debug.Log(line);
                lastSensorLogSecond = now;
            }
            return;
        }
        string[] parts = line.Split(',');
        if (parts.Length != 4)
        {
            return;
        }
        float x;
        float y;
        float z;
        bool xOK = float.TryParse(parts[1], NumberStyles.Float, CultureInfo.InvariantCulture, out x);
        bool yOK = float.TryParse(parts[2], NumberStyles.Float, CultureInfo.InvariantCulture, out y);
        bool zOK = float.TryParse(parts[3], NumberStyles.Float, CultureInfo.InvariantCulture, out z);
        if (!xOK || !yOK || !zOK || float.IsNaN(x) || float.IsNaN(y) || float.IsNaN(z) || float.IsInfinity(x) || float.IsInfinity(y) || float.IsInfinity(z))
        {
            return;
        }
        handPosition = new Vector3(x, y, z);
        if (!hasPosition)
        {
            smoothedPosition = handPosition;
            positionVelocity = Vector3.zero;
            previousDesiredGazeX = 0.0f;
            previousDesiredGazeY = 0.0f;
            waitingForHead = false;
        }
        lastPositionLine = line;
        hasPosition = true;
        receivedPositions++;
        if (now != lastPosLogSecond)
        {
            Debug.Log(line);
            lastPosLogSecond = now;
        }
    }

    void CalculateDesiredGaze()
    {
        horizontalInput = Mathf.Clamp((smoothedPosition.x - sensorCenterX) / Mathf.Max(0.01f, sensorHalfWidth), -1.0f, 1.0f);
        verticalInput = Mathf.Clamp((smoothedPosition.y - sensorCenterY) / Mathf.Max(0.01f, sensorHalfHeight), -1.0f, 1.0f);
        desiredGazeX = verticalInput * maxGazeX;
        desiredGazeY = -horizontalInput * maxGazeY;
    }

    void UpdateHeadDelay()
    {
        float xChange = Mathf.Abs(desiredGazeX - previousDesiredGazeX);
        float yChange = Mathf.Abs(desiredGazeY - previousDesiredGazeY);
        if (!waitingForHead && headDelayTimer <= 0.0f && (xChange > headMovementThreshold || yChange > headMovementThreshold))
        {
            headDelayTimer = Mathf.Max(0.0f, headDelay);
            waitingForHead = true;
        }
        previousDesiredGazeX = desiredGazeX;
        previousDesiredGazeY = desiredGazeY;
        if (headDelayTimer > 0.0f)
        {
            headDelayTimer = Mathf.Max(0.0f, headDelayTimer - Time.deltaTime);
        }
        if (headDelayTimer <= 0.0f)
        {
            waitingForHead = false;
        }
    }

    void UpdateHead()
    {
        float targetHeadX = verticalInput * maxHeadX;
        float targetHeadY = -horizontalInput * maxHeadY;
        if (headDelayTimer <= 0.0f)
        {
            float t = 1.0f - Mathf.Exp(-Mathf.Max(0.0f, headSpeed) * Time.deltaTime);
            currentHeadX = Mathf.Lerp(currentHeadX, targetHeadX, t);
            currentHeadY = Mathf.Lerp(currentHeadY, targetHeadY, t);
        }
        transform.localRotation = headNeutralRotation * Quaternion.Euler(currentHeadX, currentHeadY, 0.0f);
    }

    void UpdateEyes()
    {
        float targetEyeX = desiredGazeX - currentHeadX;
        float targetEyeY = desiredGazeY - currentHeadY;
        float t = 1.0f - Mathf.Exp(-Mathf.Max(0.0f, eyeSpeed) * Time.deltaTime);
        currentEyeX = Mathf.Lerp(currentEyeX, targetEyeX, t);
        currentEyeY = Mathf.Lerp(currentEyeY, targetEyeY, t);
        Quaternion eyeRotation = Quaternion.Euler(currentEyeX, currentEyeY, 0.0f);
        if (rightEye != null)
        {
            rightEye.localRotation = rightEyeNeutralRotation * eyeRotation;
        }
        if (leftEye != null)
        {
            leftEye.localRotation = leftEyeNeutralRotation * eyeRotation;
        }
    }

    void UpdateDebugValues()
    {
        if (rightEye != null)
        {
            rightEyeRotation = rightEye.localEulerAngles;
        }
        if (leftEye != null)
        {
            leftEyeRotation = leftEye.localEulerAngles;
        }
    }

    void OnDisable()
    {
        CloseSerialPort();
    }

    void OnDestroy()
    {
        CloseSerialPort();
    }

    void CloseSerialPort()
    {
        if (serialPort != null)
        {
            try
            {
                if (serialPort.IsOpen)
                {
                    serialPort.Close();
                }
                serialPort.Dispose();
            }
            catch (Exception e)
            {
                Debug.LogWarning("Serial close error: " + e.Message);
            }
            serialPort = null;
        }
        connected = false;
    }
}
#endif
