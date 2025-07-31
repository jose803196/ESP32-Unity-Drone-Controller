using System;
using System.IO.Ports;
using UnityEditor;
using UnityEngine;

public class SerialReader : MonoBehaviour
{
    private SerialPort serial;
    private string portName = "COM3";
    private int baudRate = 115200;

    [Header("Parámetros de Vuelo")]
    public float moveSpeed = 5.0f; // Velocidad para el movimiento horizontal
    public float verticalSpeed = 3.0f; // Velocidad de ascenso/descenso
    public float rotationSpeed = 10.0f; // Velocidad de giro

    private Rigidbody rb;

    // --- NUEVAS VARIABLES para almacenar los inputs leídos en Update ---
    private float moveForward = 0f;
    private float moveStrafe = 0f; // Strafe = moverse lateralmente
    private float moveVertical = 0f;
    private float rotationYaw = 0f; // Yaw = rotación en el eje Y (giro)

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
    }
    void Start()
    {
        try
        {
            serial = new SerialPort(portName, baudRate);
            serial.ReadTimeout = 1000;
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
                    //Debug.Log(line);
                    string[] values = line.Split(',');
                    if (values.Length == 13)
                    {
                        int btn_avanzar = int.Parse(values[0]);
                        int btn_retroceder = int.Parse(values[1]);
                        int btn_izquierda = int.Parse(values[2]);
                        int btn_derecha = int.Parse(values[3]);
                        int vry_raw = int.Parse(values[5]); // joystick Y para subir/bajar
                        int vrx_raw = int.Parse(values[6]); // joystick X para rotar

                        // --- Procesar y Guardar los Inputs ---

                        // 1. Movimiento Adelante/Atrás (con los botones)
                        // Esto nos da un valor de -1 (retroceder), 0 (quieto), o 1 (avanzar)
                        moveForward = btn_avanzar - btn_retroceder;

                        // 2. Movimiento Lateral (con los botones)
                        // Esto nos da un valor de -1 (izquierda), 0 (quieto), o 1 (derecha)
                        moveStrafe = btn_derecha - btn_izquierda;

                        // 3. Movimiento Vertical y Rotación (con el Joystick)
                        // ¡LA CLAVE DE LA SUAVIDAD! Normalizamos el valor del joystick de 0-4095 a -1 a +1.
                        // Asumimos que 2048 es el centro.

                        // Para el eje Y del Joystick (Ascenso/Descenso)
                        float vry_normalized = (vry_raw - 1860f) / 1860f;
                        // Introducimos una 'zona muerta' para que el joystick no se mueva con pequeños temblores
                        if (Mathf.Abs(vry_normalized) > 0.1f)
                        {
                            moveVertical = vry_normalized;
                        }
                        else
                        {
                            moveVertical = 0f;
                        }

                        // Para el eje X del Joystick (Rotación)
                        float vrx_normalized = (vrx_raw - 1920f) / 1920f;
                        if (Mathf.Abs(vrx_normalized) > 0.1f)
                        {
                            rotationYaw = vrx_normalized;
                        }
                        else
                        {
                            rotationYaw = 0f;
                        }
                    }
                }
            }
            catch (Exception e)
            {

            }
        }
    }
    void FixedUpdate()
    {
        // Salir si el Rigidbody no está asignado
        if (rb == null) return;

        // --- 1. Movimiento Horizontal (Adelante/Atrás y Lateral) ---
        // Creamos un vector de dirección basado en los inputs de los botones.
        Vector3 moveDirection = new Vector3(moveStrafe, 0, moveForward);

        // Normalizamos para evitar movimiento diagonal más rápido.
        moveDirection.Normalize();

        // Convertimos la dirección local del dron a dirección global
        // y aplicamos la fuerza.
        Vector3 worldMoveDirection = transform.TransformDirection(moveDirection);
        rb.AddForce(worldMoveDirection * moveSpeed, ForceMode.Acceleration);


        // --- 2. Movimiento Vertical (Ascenso/Descenso) ---
        // Aplicamos fuerza directamente en el eje Y global.
        Vector3 verticalForce = Vector3.up * moveVertical * verticalSpeed;
        rb.AddForce(verticalForce, ForceMode.Acceleration);


        // --- 3. Rotación (Giro) ---
        // Creamos un torque (fuerza de rotación) en el eje Y.
        Vector3 yawTorque = Vector3.up * rotationYaw * rotationSpeed;
        rb.AddTorque(yawTorque, ForceMode.Acceleration);


        // --- Opcional pero Recomendado: Contrarrestar la Gravedad y Limitar Velocidad ---

        // Aplicar una fuerza hacia arriba para simular la sustentación y que el dron "flote".
        // Esto ayuda a que no caiga como una piedra si no aceleras hacia arriba.
        rb.AddForce(Vector3.up * 9.81f, ForceMode.Acceleration);

        // Limitar la velocidad para que no se descontrole.
        if (rb.velocity.magnitude > moveSpeed * 1.5f)
        {
            rb.velocity = rb.velocity.normalized * moveSpeed * 1.5f;
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