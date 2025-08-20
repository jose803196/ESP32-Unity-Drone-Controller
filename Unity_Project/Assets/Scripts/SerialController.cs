using UnityEngine;
using System.Globalization;
using System.IO; // Necesario para trabajar con archivos
using System.Text;

public class SerialReader : MonoBehaviour
{
    [Header("Parámetros de Vuelo")]
    public float moveSpeed = 10.0f;
    public float verticalSpeed = 10.0f;
    public float rotationSpeed = 1.0f;
    [Header("Parámetros del Giroscopio")]
    public float gyroSensitivity = 1.0f;
    public float gyroDeadzone = 5.0f;
    [Header("PID Auto-Nivelación")]
    public float pGain = 0.5f;
    public float iGain = 0f;
    public float dGain = 0.05f;

    // PARÁMETROS DEL REGISTRO DE DATOS
    [Header("Data Logging")]
    [Tooltip("Nombre de la carpeta donde se guardarán los CSV.")]
    public string logFolderName = "FlightDataLogs";

    // VARIABLES INTERNAS 
    private Rigidbody rb;
    private float moveForward, moveStrafe, moveVertical, yawInput, pitchRate, rollRate;
    private PIDController pitchPid, rollPid;

    // VARIABLES PARA EL REGISTRO CSV
    private StringBuilder csvBuilder;
    private string csvFilePath;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        UpdatePIDGains();

        // INICIALIZAMOS EL REGISTRO DE DATOS
        InitializeDataLogger();
    }

    void OnValidate() { UpdatePIDGains(); }
    void UpdatePIDGains()
    {
        pitchPid = new PIDController(pGain, iGain, dGain);
        rollPid = new PIDController(pGain, iGain, dGain);
    }
    private void OnEnable() { if (SerialManager.Instance != null) SerialManager.Instance.OnSerialDataReceived += HandleSerialData; }

    private void OnDisable()
    {
        if (SerialManager.Instance != null) SerialManager.Instance.OnSerialDataReceived -= HandleSerialData;

        // GUARDAMOS LOS DATOS ACUMULADOS
        SaveLogFile();
    }

    private void HandleSerialData(string receivedData)
    {
        string[] values = receivedData.Split(',');
        if (values.Length == 13)
        {
            moveForward = int.Parse(values[0]) - int.Parse(values[1]);
            moveStrafe = int.Parse(values[3]) - int.Parse(values[2]);
            moveVertical = (int.Parse(values[5]) - 1860f) / 1860f;
            yawInput = (int.Parse(values[6]) - 1920f) / 1920f;
            float gx_raw = float.Parse(values[10], CultureInfo.InvariantCulture);
            float gy_raw = float.Parse(values[11], CultureInfo.InvariantCulture);
            pitchRate = Mathf.Abs(gy_raw) > gyroDeadzone ? gy_raw : 0f;
            rollRate = Mathf.Abs(gx_raw) > gyroDeadzone ? gx_raw : 0f;
        }
    }

    void FixedUpdate()
    {
        if (rb == null) return;

        // FÍSICA Y LÓGICA DE VUELO
        Vector3 moveDirection = new Vector3(moveStrafe, 0, moveForward);
        rb.AddForce(transform.TransformDirection(moveDirection.normalized) * moveSpeed, ForceMode.Acceleration);
        rb.AddForce(Vector3.up * moveVertical * verticalSpeed, ForceMode.Acceleration);
        rb.AddTorque(Vector3.up * yawInput * rotationSpeed, ForceMode.Acceleration);
        Vector3 gyroTorque = new Vector3(-pitchRate, 0, rollRate) * gyroSensitivity;
        rb.AddRelativeTorque(gyroTorque, ForceMode.Acceleration);

        if (Mathf.Approximately(pitchRate, 0f) && Mathf.Approximately(rollRate, 0f))
        {
            Vector3 currentAngularVelocity = transform.InverseTransformDirection(rb.angularVelocity);
            float pitchError = -currentAngularVelocity.x;
            float rollError = -currentAngularVelocity.z;
            float pitchCorrection = pitchPid.Update(pitchError, Time.fixedDeltaTime);
            float rollCorrection = rollPid.Update(rollError, Time.fixedDeltaTime);
            Vector3 pidTorque = new Vector3(pitchCorrection, 0, rollCorrection);
            rb.AddRelativeTorque(pidTorque, ForceMode.Acceleration);
        }

        rb.AddForce(Vector3.up * 9.81f, ForceMode.Acceleration);
        if (rb.velocity.magnitude > moveSpeed * 1.5f) { rb.velocity = rb.velocity.normalized * moveSpeed * 1.5f; }

        // REGISTRAMOS LOS DATOS DE ESTE FRAME 
        RecordDataPoint();
    }

    // FUNCIONES PARA EL REGISTRO DE DATOS

    private void InitializeDataLogger()
    {
        // Crear el directorio si no existe.
        string folderPath = Path.Combine(Application.dataPath, logFolderName);
        Directory.CreateDirectory(folderPath);

        // Crear un nombre de archivo único con la fecha y hora.
        string fileName = $"FlightLog_{System.DateTime.Now:yyyy-MM-dd_HH-mm-ss}.csv";
        csvFilePath = Path.Combine(folderPath, fileName);

        // Crear el StringBuilder y escribir la cabecera del CSV.
        csvBuilder = new StringBuilder();
        string header = "timestamp," +
                        "posX,posY,posZ," +
                        "rotW,rotX,rotY,rotZ," +
                        "velX,velY,velZ," +
                        "angVelX,angVelY,angVelZ," +
                        "inputMoveForward,inputMoveStrafe,inputMoveVertical,inputYaw," +
                        "inputPitchRate,inputRollRate\n";
        csvBuilder.Append(header);

        Debug.Log($"Iniciando registro de datos en: {csvFilePath}");
    }

    private void RecordDataPoint()
    {
        string line = string.Format(CultureInfo.InvariantCulture,
            "{0:F4},{1:F4},{2:F4},{3:F4},{4:F4},{5:F4},{6:F4},{7:F4},{8:F4},{9:F4},{10:F4},{11:F4},{12:F4},{13:F4},{14:F4},{15:F4},{16:F4},{17:F4},{18:F4},{19:F4}\n",
            Time.time,
            transform.position.x, transform.position.y, transform.position.z,
            rb.rotation.w, rb.rotation.x, rb.rotation.y, rb.rotation.z,
            rb.velocity.x, rb.velocity.y, rb.velocity.z,
            rb.angularVelocity.x, rb.angularVelocity.y, rb.angularVelocity.z,
            moveForward, moveStrafe, moveVertical, yawInput,
            pitchRate, rollRate
        );
        csvBuilder.Append(line);
    }

    private void SaveLogFile()
    {
        if (csvBuilder == null || csvBuilder.Length <= 0) return;

        File.WriteAllText(csvFilePath, csvBuilder.ToString());
        Debug.Log($"¡Datos guardados! Se ha creado el archivo: {csvFilePath}");

        csvBuilder = null;
    }
}