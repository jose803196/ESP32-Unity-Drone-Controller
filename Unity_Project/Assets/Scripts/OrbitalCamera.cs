using UnityEngine;

public class OrbitalCamera : MonoBehaviour
{
    [Header("Objetivo a Seguir")]
    [Tooltip("El Transform del objeto que la cámara debe orbitar (tu dron).")]
    public Transform target;

    [Header("Configuración de Órbita")]
    [Tooltip("La distancia inicial de la cámara al objetivo.")]
    public float distance = 5.0f;
    [Tooltip("La velocidad de rotación de la cámara con el ratón.")]
    public float rotationSpeed = 120.0f;
    [Tooltip("La suavidad con la que la cámara se mueve a su nueva posición.")]
    public float followSmoothness = 0.05f;

    [Header("Configuración de Zoom")]
    [Tooltip("La velocidad del zoom con la rueda del ratón.")]
    public float zoomSpeed = 4.0f;
    [Tooltip("La distancia mínima y máxima del zoom.")]
    public Vector2 zoomDistanceRange = new Vector2(2f, 15f);

    [Header("Límites de Ángulo (Pitch)")]
    [Tooltip("El ángulo mínimo y máximo en vertical (evita que la cámara pase por debajo del suelo).")]
    public Vector2 pitchMinMax = new Vector2(-40f, 85f);

    // Variables privadas para el estado de la rotación
    private float yaw = 0.0f;   // Rotación horizontal (eje Y)
    private float pitch = 20.0f; // Rotación vertical (eje X)
    private Vector3 currentVelocity = Vector3.zero;

    void LateUpdate()
    {
        // Si no tenemos objetivo, no hacemos nada.
        if (target == null)
        {
            Debug.LogWarning("La cámara orbital no tiene un objetivo (target) asignado.");
            return;
        }

        // --- 1. LÓGICA DE ROTACIÓN (CONTROLADA POR EL RATÓN) ---
        // Si mantienes presionado el botón derecho del ratón (o el izquierdo, si prefieres)...
        if (Input.GetMouseButton(1)) // 0 = Izquierdo, 1 = Derecho, 2 = Central
        {
            // ...leemos el movimiento del ratón y lo aplicamos a nuestros ángulos.
            yaw += Input.GetAxis("Mouse X") * rotationSpeed * Time.deltaTime;
            pitch -= Input.GetAxis("Mouse Y") * rotationSpeed * Time.deltaTime;

            // Limitamos el ángulo vertical para evitar que la cámara de la vuelta por completo.
            pitch = Mathf.Clamp(pitch, pitchMinMax.x, pitchMinMax.y);
        }

        // --- 2. LÓGICA DE ZOOM (RUEDA DEL RATÓN) ---
        distance -= Input.GetAxis("Mouse ScrollWheel") * zoomSpeed;
        distance = Mathf.Clamp(distance, zoomDistanceRange.x, zoomDistanceRange.y);

        // --- 3. CÁLCULO DE LA POSICIÓN Y ROTACIÓN ---
        // Convertimos nuestros ángulos (yaw, pitch) en una rotación real (Quaternion).
        Quaternion rotation = Quaternion.Euler(pitch, yaw, 0);
        // Calculamos la posición deseada de la cámara: está en la posición del objetivo,
        // pero desplazada hacia atrás por la rotación y la distancia.
        Vector3 desiredPosition = target.position - (rotation * Vector3.forward * distance);

        // --- 4. SUAVIZADO DEL MOVIMIENTO ---
        // En lugar de teletransportar la cámara, usamos SmoothDamp para un movimiento
        // fluido y profesional. Evita cualquier tipo de "jitter" o salto.
        transform.position = Vector3.SmoothDamp(transform.position, desiredPosition, ref currentVelocity, followSmoothness);

        // 5. HACER QUE LA CÁMARA MIRE AL OBJETIVO ---
        // Después de calcular nuestra posición final, siempre miramos de vuelta al objetivo.
        // Esto mantiene al dron en el centro.
        transform.LookAt(target.position);
    }
}