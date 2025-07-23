using UnityEngine;
using System.IO.Ports;
using System;

public class SerialReader : MonoBehaviour
{
    private SerialPort serial;
    private string portName = "COM3"; // Ajusta según tu puerto
    private int baudRate = 115200;

    void Start()
    {
        try
        {
            serial = new SerialPort(portName, baudRate);
            serial.ReadTimeout = 1000; // Aumentar a 1000 ms
            serial.Open();
            Debug.Log("Puerto serial abierto: " + portName);
        }
        catch (Exception e)
        {
            Debug.LogError("Error al abrir el puerto serial: " + e.Message);
        }
    }

    void Update()
    {
        if (serial != null && serial.IsOpen)
        {
            try
            {
                if (serial.BytesToRead > 0)
                {
                    string line = serial.ReadLine();
                    Debug.Log(line);
                    string[] values = line.Split(',');
                    if (values.Length == 13)
                    {
                        int btn_avanzar = int.Parse(values[0]);
                        int joy_x = int.Parse(values[5]);
                        float ax = float.Parse(values[7]);
                        Debug.Log($"btn_avanzar: {btn_avanzar}, joy_x: {joy_x}, ax: {ax}");
                    }
                }
            }
            catch (Exception e)
            {

            }
        }
    }

    void OnDestroy()
    {
        if (serial != null && serial.IsOpen)
        {
            serial.Close();
            Debug.Log("Puerto serial cerrado");
        }
    }
}