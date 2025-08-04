using System;
using System.IO.Ports;
using UnityEngine;
using System.Collections;

public class SerialManager : MonoBehaviour
{
    public static SerialManager Instance { get; private set; }

    private SerialPort serial;
    [SerializeField] private string portName = "COM3";
    [SerializeField] private int baudRate = 115200;

    public event Action<string> OnSerialDataReceived;

    private void Awake()
    {
        if (Instance != null && Instance != this)
        {
            Destroy(gameObject);
            return;
        }
        Instance = this;
        DontDestroyOnLoad(gameObject);
    }
    public void WriteSerialAfterDelay(string message, float delay)
    {
        StartCoroutine(WriteSerialCoroutine(message, delay));
    }

    private IEnumerator WriteSerialCoroutine(string message, float delay)
    {
        yield return new WaitForSeconds(delay);
        WriteSerial(message);
    }
    void Start()
    {
        try
        {
            serial = new SerialPort(portName, baudRate);
            serial.ReadTimeout = 1000;
            serial.Open();
            Debug.Log("SerialManager: Puerto serial abierto con éxito: " + portName);
        }
        catch (Exception e)
        {
            Debug.LogError("SerialManager: Error al abrir el puerto serial: " + e.Message);
        }
    }

    void Update()
    {
        if (serial != null && serial.IsOpen && serial.BytesToRead > 0)
        {
            try
            {
                string line = serial.ReadLine();
                OnSerialDataReceived?.Invoke(line);
            }
            catch (Exception)
            {
                // Ignorar errores de timeout, etc.
            }
        }
    }

    public void WriteSerial(string message)
    {
        if (serial != null && serial.IsOpen)
        {
            try
            {
                serial.Write(message);
                Debug.Log("SerialManager: Enviado por UART -> " + message);
            }
            catch (Exception e)
            {
                Debug.LogError("SerialManager: Error al escribir en el puerto: " + e.Message);
            }
        }
    }

    private void OnDestroy()
    {
        if (serial != null && serial.IsOpen)
        {
            serial.Close();
            Debug.Log("SerialManager: Puerto serial cerrado.");
        }
    }
    private void OnApplicationQuit()
    {
        if (serial != null && serial.IsOpen)
        {
            serial.Close();
        }
    }
}