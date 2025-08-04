using UnityEngine;

public class SerialReader : MonoBehaviour
{
    [Header("Parámetros de Vuelo")]
    public float moveSpeed = 10.0f;
    public float verticalSpeed = 10.0f;
    public float rotationSpeed = 15.0f;

    private Rigidbody rb;
    private float moveForward = 0f;
    private float moveStrafe = 0f;
    private float moveVertical = 0f;
    private float rotationYaw = 0f;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
    }
    private void OnEnable()
    {
        if (SerialManager.Instance != null)
        {
            SerialManager.Instance.OnSerialDataReceived += HandleSerialData;
        }
    }

    private void OnDisable()
    {
        if (SerialManager.Instance != null)
        {
            SerialManager.Instance.OnSerialDataReceived -= HandleSerialData;
        }
    }

    private void HandleSerialData(string receivedData)
    {
        string[] values = receivedData.Split(',');
        if (values.Length == 13)
        {
            int btn_avanzar = int.Parse(values[0]);
            int btn_retroceder = int.Parse(values[1]);
            int btn_izquierda = int.Parse(values[2]);
            int btn_derecha = int.Parse(values[3]);
            int btn_joystick = int.Parse(values[4]);
            int vry_raw = int.Parse(values[5]);
            int vrx_raw = int.Parse(values[6]);


            moveForward = btn_avanzar - btn_retroceder;
            moveStrafe = btn_derecha - btn_izquierda;

            float vry_normalized = (vry_raw - 1860f) / 1860f;
            moveVertical = Mathf.Abs(vry_normalized) > 0.1f ? vry_normalized : 0f;

            float vrx_normalized = (vrx_raw - 1920f) / 1920f;
            rotationYaw = Mathf.Abs(vrx_normalized) > 0.1f ? vrx_normalized : 0f;
        }
    }

    void Update()
    {
        // Nada aquí
    }
    void FixedUpdate()
    {
        if (rb == null) return;
        Vector3 moveDirection = new Vector3(moveStrafe, 0, moveForward);
        moveDirection.Normalize();
        Vector3 worldMoveDirection = transform.TransformDirection(moveDirection);
        rb.AddForce(worldMoveDirection * moveSpeed, ForceMode.Acceleration);
        Vector3 verticalForce = Vector3.up * moveVertical * verticalSpeed;
        rb.AddForce(verticalForce, ForceMode.Acceleration);
        Vector3 yawTorque = Vector3.up * rotationYaw * rotationSpeed;
        rb.AddTorque(yawTorque, ForceMode.Acceleration);
        rb.AddForce(Vector3.up * 9.81f, ForceMode.Acceleration);
        if (rb.velocity.magnitude > moveSpeed * 1.5f)
        {
            rb.velocity = rb.velocity.normalized * moveSpeed * 1.5f;
        }
    }
}