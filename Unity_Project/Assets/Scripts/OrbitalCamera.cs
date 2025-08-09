using UnityEngine;

// Es una buena práctica asegurarse de que este script siempre esté en un objeto con una Cámara.
[RequireComponent(typeof(Camera))]
public class OrbitalCamera : MonoBehaviour
{
    [Header("Objetivo a Seguir")]
    public Transform target;

    [Header("Configuración de Seguimiento Automático")]
    [Tooltip("La distancia a la que la cámara se mantiene detrás del dron.")]
    public float distance = 7.0f;
    [Tooltip("La altura a la que la cámara se mantiene sobre el dron.")]
    public float height = 3.0f;
    [Tooltip("La suavidad con la que la cámara sigue la rotación del dron.")]
    public float rotationDamping = 3.0f;
    [Tooltip("La suavidad con la que la cámara sigue la posición del dron.")]
    public float positionDamping = 5.0f;

    [Header("Rendimiento")]
    [Tooltip("La distancia máxima a la que la cámara renderizará objetos. Valores más bajos mejoran el rendimiento.")]
    public float renderDistance = 250f;

    // El método público para asignar el objetivo sigue siendo necesario.
    public void AssignTarget(Transform newTarget)
    {
        target = newTarget;
        Debug.Log($"Cámara: Nuevo objetivo asignado -> {newTarget.name}");
    }

    void Start()
    {
        // Aplicamos la distancia de renderizado al componente Cámara.
        Camera cam = GetComponent<Camera>();
        if (cam != null)
        {
            cam.farClipPlane = renderDistance;
        }
        else
        {
            Debug.LogError("No se encontró el componente 'Camera' en este objeto.");
        }
    }

    // Usamos LateUpdate para asegurar que la cámara se mueva DESPUÉS de que el dron se haya movido.
    void LateUpdate()
    {
        // Si no hay objetivo, no hacemos nada.
        if (target == null)
        {
            // Ya no mostramos un warning cada frame para no llenar la consola.
            // GameInitializer se encarga de asignarlo.
            return;
        }

        // --- CÁLCULO DE LA POSICIÓN Y ROTACIÓN DESEADAS ---

        // 1. Ángulo deseado: Queremos que la cámara mire en la misma dirección que el dron.
        float desiredYawAngle = target.eulerAngles.y;

        // 2. Altura deseada: La altura actual del dron más la altura fija que definimos.
        float desiredHeight = target.position.y + height;

        // 3. Rotación actual de la cámara
        float currentYawAngle = transform.eulerAngles.y;
        float currentHeight = transform.position.y;

        // --- SUAVIZADO (DAMPING) ---
        // Para que la cámara no se mueva bruscamente, suavizamos el cambio de ángulo y altura.

        // Suavizamos el ángulo de rotación horizontal (yaw).
        currentYawAngle = Mathf.LerpAngle(currentYawAngle, desiredYawAngle, rotationDamping * Time.deltaTime);

        // Suavizamos el movimiento vertical.
        currentHeight = Mathf.Lerp(currentHeight, desiredHeight, positionDamping * Time.deltaTime);

        // --- APLICACIÓN DE LA POSICIÓN FINAL ---

        // Convertimos el ángulo suavizado de vuelta a una rotación.
        Quaternion currentRotation = Quaternion.Euler(0, currentYawAngle, 0);

        // Calculamos la posición de la cámara:
        // Empezamos en la posición del dron.
        Vector3 cameraPosition = target.position;
        // Nos movemos hacia atrás según la rotación.
        cameraPosition -= currentRotation * Vector3.forward * distance;
        // Ajustamos la altura suavizada.
        cameraPosition = new Vector3(cameraPosition.x, currentHeight, cameraPosition.z);

        // Asignamos la nueva posición a la cámara.
        transform.position = cameraPosition;

        // Finalmente, nos aseguramos de que la cámara siempre mire al dron.
        transform.LookAt(target);
    }
}