// PIDController.cs - No es un MonoBehaviour, es una clase de ayuda.
public class PIDController
{
    private float pGain, iGain, dGain;
    private float integral;
    private float lastError;

    public PIDController(float p, float i, float d)
    {
        this.pGain = p;
        this.iGain = i;
        this.dGain = d;
    }

    public float Update(float currentError, float deltaTime)
    {
        // P (Proporcional): Reacciona al error actual.
        float pTerm = pGain * currentError;

        // I (Integral): Corrige errores acumulados a lo largo del tiempo (elimina la deriva).
        integral += currentError * deltaTime;
        float iTerm = iGain * integral;

        // D (Derivativo): Amortigua el movimiento prediciendo el error futuro. ¡Evita las oscilaciones!
        float derivative = (currentError - lastError) / deltaTime;
        float dTerm = dGain * derivative;

        lastError = currentError;

        return pTerm + iTerm + dTerm;
    }
}