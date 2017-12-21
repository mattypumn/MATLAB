If you've never wanted to capture color from the passthrough camera and use it in a shader, you can stop reading.

This is something I cobbled together in Unity while prototyping  (i.e. there may be a better way) to avoid copying color back to the CPU, which makes using the passthrough camera color much more efficient.

The general idea is:

1. Render the passthrough camera to a render texture
2. Set up a compute buffer to hold vertex colors (or whatever colors you want)
3. Use vertexID in the vertex shader to find color in the compute buffer
4. Use a sentinel color (0,0,0,0) to indicate that the color should be updated

Now you can copy the color from the passthrough camera on every frame :)

Below are the relevant code snippets.

--
Jeremy

-------------------8<----------------

Capture the passthrough feed as a texture:
public class PreRenderPass : MonoBehaviour {
  public RenderTexture m_texture;

  public void Start() {
    var cam = GetComponent<Camera>();
    m_texture = new RenderTexture(cam.pixelWidth, cam.pixelHeight, 0, RenderTextureFormat.ARGB32);
    Shader.SetGlobalTexture("_PassthroughCam", m_texture);
  }

  public void Update() {
    var cam = GetComponent<Camera>();
    var mask = cam.cullingMask;
    var oldTexture = cam.targetTexture;
    cam.cullingMask = 0;
    cam.targetTexture = m_texture;
    cam.Render();
    cam.targetTexture = oldTexture;
    cam.cullingMask = mask;
  }
}

Relevant bits from the vertex shader:
      // Enable RWStructuredBuffer
      #pragma target 5.0

      // Setup the index into the compute buffer
      struct appdata
      {
        uint vertexId : SV_VertexID;
        ...
      };

      // Uniforms
      RWStructuredBuffer<float4> _GpuColors : register(u1);
      sampler2D _PassthroughCam;

      // Later, in the vertex shader:
        float4 videoColor = tex2Dlod(_PassthroughCam, float4((vertex / vertex.w).xy * .5 + .5, 0, 0));

        // Latch the color when alpha == 0
        float4 color = _GpuColors[v.vertexId];
        if (color.a == 0) {
          vertex = mul(UNITY_MATRIX_MVP, vertex);
          color = videoColor;
          color.a = 1;
          // Write the value back to the compute buffer.
          _GpuColors[v.vertexId] = color;
        }


Compute buffer setup:
  private Color[] m_colors = new Color[kNumColors];
  private ComputeBuffer m_gpuColors;

  public void Start() {
    m_gpuColors = new ComputeBuffer(kNumColors  , sizeof(float) * 4);
    m_gpuColors.SetData(m_colors);
    m_material.SetBuffer("_GpuColors", m_gpuColors);
  }

  public void OnDestroy() {
    if (m_gpuColors != null) {
      m_gpuColors.Dispose();
      m_gpuColors = null;
    }
  }
