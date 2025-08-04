using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(AudioSource))]
public class DroneAudioController : MonoBehaviour
{
    private Rigidbody rb;
    private AudioSource audioSource;

    [Header("Configuración de Hélices")]
    [Tooltip("Arrastra aquí los 4 objetos de las hélices del dron")]
    public Transform[] propellers;
    [Tooltip("Velocidad de rotación de las hélices al ralentí (motor encendido pero quieto)")]
    public float minRotationSpeed = 300f;
    [Tooltip("Velocidad máxima de rotación de las hélices a máxima potencia")]
    public float maxRotationSpeed = 3000f;

    [Header("Configuración de Audio del Motor")]
    [Tooltip("El tono del audio cuando el dron está quieto pero encendido.")]
    public float minPitch = 0.5f;
    [Tooltip("El tono del audio cuando el dron va a máxima velocidad.")]
    public float maxPitch = 2.0f;
    [Tooltip("El volumen del audio cuando el dron está quieto.")]
    public float minVolume = 0.4f;
    [Tooltip("El volumen del audio a máxima velocidad.")]
    public float maxVolume = 1.0f;

    [Header("Calibración de Velocidad")]
    [Tooltip("La velocidad del Rigidbody que se considerará 'máxima potencia' para el audio y las hélices.")]
    public float maxSpeedForEffects = 20.0f;

    // --- ¡NUEVAS VARIABLES PARA DETECTAR EL ESTADO DE VUELO! ---
    [Header("Detección de Vuelo")]
    [Tooltip("La capa(s) que el dron considerará como 'suelo'.")]
    public LayerMask groundLayer;
    [Tooltip("La distancia desde el dron hacia abajo para considerarse 'aterrizado'.")]
    public float groundedThreshold = 0.5f;
    [Tooltip("Factor de potencia mínimo cuando el dron está flotando en el aire sin moverse. Mantiene las hélices activas.")]
    [Range(0.0f, 1.0f)]
    public float hoverPowerFactor = 0.15f;
    // --------------------------------------------------------------------

    // Variables internas
    private float currentPowerFactor = 0f;
    private bool isEngineOn = false;
    private bool isGrounded = true; // Asumimos que empieza en el suelo.

    void Awake()
    {
        rb = GetComponent<Rigidbody>();
        audioSource = GetComponent<AudioSource>();
    }

    void Start()
    {
        StartEngine();
    }

    public void StartEngine()
    {
        isEngineOn = true;
        audioSource.Play();
        audioSource.pitch = minPitch;
        audioSource.volume = 0f; // Empezamos en silencio, se activará al despegar.
    }

    void Update()
    {
        if (!isEngineOn) return;

        // --- ¡NUEVA LÓGICA DE DETECCIÓN! ---
        CheckIfGrounded();
        // ------------------------------------

        // 1. Calcular el "factor de potencia" basado en la velocidad.
        float verticalVelocity = Mathf.Abs(rb.velocity.y);
        float horizontalVelocity = new Vector2(rb.velocity.x, rb.velocity.z).magnitude;
        float combinedSpeed = verticalVelocity + horizontalVelocity;
        float speedBasedPower = Mathf.InverseLerp(0, maxSpeedForEffects, combinedSpeed);

        // --- ¡LÓGICA MEJORADA! ---
        // Si estamos en el aire, el factor de potencia NUNCA será menor que el de 'hover'.
        // Si estamos en el suelo, puede ser 0 si la velocidad es 0.
        float basePower = isGrounded ? 0f : hoverPowerFactor;
        currentPowerFactor = Mathf.Max(basePower, speedBasedPower);
        // -----------------------------

        // 2. Aplicar los efectos basados en el factor de potencia.
        UpdateAudio();
        UpdatePropellers();
    }

    // --- ¡NUEVO MÉTODO PARA VERIFICAR LA DISTANCIA AL SUELO! ---
    private void CheckIfGrounded()
    {
        // Lanzamos un rayo invisible desde la posición del dron hacia abajo.
        RaycastHit hit;
        if (Physics.Raycast(transform.position, Vector3.down, out hit, groundedThreshold, groundLayer))
        {
            // Si el rayo choca con algo en la capa 'ground' dentro de la distancia, estamos en el suelo.
            isGrounded = true;
        }
        else
        {
            // Si no, estamos volando.
            isGrounded = false;
        }
    }
    // -----------------------------------------------------------

    private void UpdateAudio()
    {
        float targetPitch = Mathf.Lerp(minPitch, maxPitch, currentPowerFactor);
        float targetVolume = Mathf.Lerp(minVolume, maxVolume, currentPowerFactor);

        // Si estamos en el suelo y sin movernos, forzamos el silencio.
        if (isGrounded && rb.velocity.magnitude < 0.1f)
        {
            targetVolume = 0f;
        }

        audioSource.pitch = Mathf.Lerp(audioSource.pitch, targetPitch, Time.deltaTime * 5f);
        audioSource.volume = Mathf.Lerp(audioSource.volume, targetVolume, Time.deltaTime * 5f);
    }

    private void UpdatePropellers()
    {
        if (propellers == null || propellers.Length == 0) return;

        float rotationSpeed = Mathf.Lerp(minRotationSpeed, maxRotationSpeed, currentPowerFactor);

        // Si estamos en el suelo y casi quietos, apagamos la rotación.
        if (isGrounded && rb.velocity.magnitude < 0.1f)
        {
            rotationSpeed = 0f;
        }

        for (int i = 0; i < propellers.Length; i++)
        {
            if (i % 2 == 0)
            {
                propellers[i].Rotate(Vector3.up, rotationSpeed * Time.deltaTime);
            }
            else
            {
                propellers[i].Rotate(Vector3.up, -rotationSpeed * Time.deltaTime);
            }
        }
    }

    public void StopEngine()
    {
        isEngineOn = false;
        audioSource.Stop();
    }
}