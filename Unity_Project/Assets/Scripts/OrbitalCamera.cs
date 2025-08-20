using UnityEngine;

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
    public float renderDistance = 500f;

    public void AssignTarget(Transform newTarget)
    {
        target = newTarget;
        Debug.Log($"Cámara: Nuevo objetivo asignado -> {newTarget.name}");
    }

    void Start()
    {
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
    void LateUpdate()
    {
        if (target == null)
        {
            return;
        }

        // CÁLCULO DE LA POSICIÓN Y ROTACIÓN DESEADAS

        // Ángulo deseado
        float desiredYawAngle = target.eulerAngles.y;

        // Altura deseada
        float desiredHeight = target.position.y + height;

        // Rotación actual de la cámara
        float currentYawAngle = transform.eulerAngles.y;
        float currentHeight = transform.position.y;

        // SUAVIZADO (DAMPING)
        currentYawAngle = Mathf.LerpAngle(currentYawAngle, desiredYawAngle, rotationDamping * Time.deltaTime);

        currentHeight = Mathf.Lerp(currentHeight, desiredHeight, positionDamping * Time.deltaTime);

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

        transform.LookAt(target);
    }
}