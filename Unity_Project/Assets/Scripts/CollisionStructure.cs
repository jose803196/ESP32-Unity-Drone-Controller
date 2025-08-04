using UnityEngine;
using UnityEngine.SceneManagement;

public class CollisionStructure : MonoBehaviour
{
    [Header("Configuracion Reinicio")]
    public float tiempoParaReiniciar = 2.5f;
    public float tiempoEnviarRunning = 0.5f;

    void OnCollisionEnter(Collision collision)
    {
        if (collision.gameObject.CompareTag("Dron"))
        {
            Rigidbody rbDron = collision.gameObject.GetComponent<Rigidbody>();
            if (rbDron != null)
            {
                rbDron.AddForce(Vector3.down * 100f, ForceMode.Impulse);
            }
            EnviarSerial("C");
            if (SerialManager.Instance != null)
            {
                SerialManager.Instance.WriteSerialAfterDelay("R", tiempoEnviarRunning);
            }
            Invoke("ReiniciarNivel", tiempoParaReiniciar);
        }
    }

    void ReiniciarNivel()
    {
        SceneManager.LoadScene(SceneManager.GetActiveScene().buildIndex);
    }
    void EnviarSerial(string mensaje)
    {
        if (SerialManager.Instance != null)
        {
            SerialManager.Instance.WriteSerial(mensaje);
        }
        else
        {
            Debug.LogWarning("Se intentó enviar un mensaje serial, pero no se encontró el SerialManager.");
        }
    }
}